#!/usr/bin/env python3
"""
camera_node.py

Merged and optimized camera + frame-transform ROS 2 node.
Replaces realsense.py and camera_to_robot_node.py.

Architecture Highlights:
- Runs RealSense pipeline in a background thread to prevent blocking the ROS 2 executor.
- Confirms tag positions using sample-accumulation + move-detection logic.
- Drops stale tags if unseen for STALE_TIMEOUT_SEC.
- Transforms coordinates internally and publishes to two separate topics.
- Applies a fixed x/y calibration offset and a hardcoded z override to the
  robot-frame output only (see "Robot-frame correction" below) -- the
  camera-frame topic stays raw/uncorrected.
"""

import time
import cv2
import numpy as np
import pyrealsense2 as rs
from cv2 import aruco
import threading

import rclpy
from rclpy.node import Node

# NOTE: Verify your custom message package name and array field name.
# Assuming DetectedObjects has an array field named 'objects' of type DetectedObject.
from panda_cartesian_control_msgs.msg import DetectedObject, DetectedObjects


# ---------------------------------------------------------------------------
# Camera / detection config
# ---------------------------------------------------------------------------
MARKER_LENGTH = 0.05
DICTIONARY = aruco.DICT_APRILTAG_36h11
STREAM_WIDTH, STREAM_HEIGHT, STREAM_FPS = 1280, 720, 15
MANUAL_EXPOSURE = 150
MIN_MARKER_PERIMETER_RATE = 0.01

# ---------------------------------------------------------------------------
# Confirmation / debounce config
# ---------------------------------------------------------------------------
SAMPLES_NEEDED = 15
MOVE_THRESHOLD_M = 0.03        # how far a sample must be from the reference to look like a move
MOVE_CONFIRM_SAMPLES = 5       # how many consistent samples in a row confirm a real move
STALE_TIMEOUT_SEC = 2.0        # drop a tag if it hasn't been re-seen within this long

# ---------------------------------------------------------------------------
# Camera -> robot base transform (from eye-to-hand calibration)
# ---------------------------------------------------------------------------
BASE_T_CAMERA = np.array([
    [ 0.006804, -0.998319, -0.057565,  0.563716],
    [-0.998902, -0.009454,  0.045875,  0.046939],
    [-0.046342,  0.057190, -0.997287,  1.728049],
    [ 0.0,       0.0,       0.0,       1.0],
])

# ---------------------------------------------------------------------------
# Robot-frame correction (applied AFTER the transform above, only on the
# robot_frame topic -- camera_frame stays raw)
# ---------------------------------------------------------------------------
OFFSET_X = -0.009   # meters, empirical x correction (from original hand-tuned offset)
OFFSET_Y = 0.011    # meters, empirical y correction (from original hand-tuned offset)
FIXED_Z = 0.15      # meters, hardcoded z -- the calibration is unreliable in z (transformed
                     # z was landing inside the table), so override it outright rather than
                     # trying to offset-correct it

# ---------------------------------------------------------------------------
# Aruco Setup (Handles both new and old OpenCV APIs)
# ---------------------------------------------------------------------------
new_api = hasattr(aruco, 'ArucoDetector')
if new_api:
    aruco_dict = aruco.getPredefinedDictionary(DICTIONARY)
    detector_params = aruco.DetectorParameters()
    detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    detector_params.minMarkerPerimeterRate = MIN_MARKER_PERIMETER_RATE
    detector_params.aprilTagQuadDecimate = 1.0
    detector_params.aprilTagQuadSigma = 0.8
    aruco_detector = aruco.ArucoDetector(aruco_dict, detector_params)
else:
    aruco_dict = aruco.Dictionary_get(DICTIONARY)
    detector_params = aruco.DetectorParameters_create()
    detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    detector_params.minMarkerPerimeterRate = MIN_MARKER_PERIMETER_RATE
    detector_params.aprilTagQuadDecimate = 1.0
    detector_params.aprilTagQuadSigma = 0.8

def detect_markers(gray):
    if new_api:
        return aruco_detector.detectMarkers(gray)
    return aruco.detectMarkers(gray, aruco_dict, parameters=detector_params)

def rvec_to_quat(rvec):
    R, _ = cv2.Rodrigues(rvec)
    m = np.eye(4); m[:3, :3] = R
    tr = np.trace(m[:3, :3])
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (m[2, 1] - m[1, 2]) / S
        qy = (m[0, 2] - m[2, 0]) / S
        qz = (m[1, 0] - m[0, 1]) / S
    else:
        i = np.argmax([m[0, 0], m[1, 1], m[2, 2]])
        if i == 0:
            S = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
            qw = (m[2, 1] - m[1, 2]) / S
            qx = 0.25 * S
            qy = (m[0, 1] + m[1, 0]) / S
            qz = (m[0, 2] + m[2, 0]) / S
        elif i == 1:
            S = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
            qw = (m[0, 2] - m[2, 0]) / S
            qx = (m[0, 1] + m[1, 0]) / S
            qy = 0.25 * S
            qz = (m[1, 2] + m[2, 1]) / S
        else:
            S = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
            qw = (m[1, 0] - m[0, 1]) / S
            qx = (m[0, 2] + m[2, 0]) / S
            qy = (m[1, 2] + m[2, 1]) / S
            qz = 0.25 * S
    return np.array([qx, qy, qz, qw])

def average_quaternions(quats):
    ref = quats[0]
    aligned = []
    for q in quats:
        if np.dot(q, ref) < 0:
            q = -q
        aligned.append(q)
    avg = np.mean(aligned, axis=0)
    return avg / np.linalg.norm(avg)


# ---------------------------------------------------------------------------
# Core Logic
# ---------------------------------------------------------------------------
class TagAccumulator:
    """Per-tag sample accumulator: confirms a stable position, detects
    genuine moves vs. noise, and tracks staleness."""

    def __init__(self):
        self.positions = []
        self.quats = []
        self.confirmed = False
        self.confirmed_pos = None
        self.confirmed_quat = None

        self.move_candidates_pos = []
        self.move_candidates_quat = []
        self.last_seen = time.time()

    def add(self, tvec, quat):
        self.last_seen = time.time()

        if self.confirmed:
            reference = self.confirmed_pos
        elif self.positions:
            reference = np.median(self.positions, axis=0)
        else:
            reference = None

        if reference is not None:
            deviation = np.linalg.norm(tvec - reference)

            if deviation > MOVE_THRESHOLD_M:
                self.move_candidates_pos.append(tvec)
                self.move_candidates_quat.append(quat)

                if len(self.move_candidates_pos) >= MOVE_CONFIRM_SAMPLES:
                    cand = np.array(self.move_candidates_pos)
                    cand_mean = np.mean(cand, axis=0)
                    cand_spread = np.max(np.linalg.norm(cand - cand_mean, axis=1))

                    if cand_spread < MOVE_THRESHOLD_M:
                        # Genuine move: restart accumulation from new cluster
                        self.positions = list(self.move_candidates_pos)
                        self.quats = list(self.move_candidates_quat)
                        self.move_candidates_pos = []
                        self.move_candidates_quat = []
                        self.confirmed = False
                        self.confirmed_pos = None
                        self.confirmed_quat = None
                    else:
                        # Noise: drop oldest
                        self.move_candidates_pos.pop(0)
                        self.move_candidates_quat.pop(0)
                return False
            else:
                self.move_candidates_pos = []
                self.move_candidates_quat = []
                if self.confirmed:
                    return False

        self.positions.append(tvec)
        self.quats.append(quat)

        if len(self.positions) >= SAMPLES_NEEDED and not self.confirmed:
            self.confirmed_pos = np.mean(self.positions, axis=0)
            self.confirmed_quat = average_quaternions(self.quats)
            self.confirmed = True
            return True
        return False

    def is_stale(self):
        return (time.time() - self.last_seen) > STALE_TIMEOUT_SEC


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Publishers
        self.pub_camera_frame = self.create_publisher(
            DetectedObjects, 'detected_objects/camera_frame', 10)
        self.pub_robot_frame = self.create_publisher(
            DetectedObjects, 'detected_objects/robot_frame', 10)

        self.accumulators = {}
        self.running = True

        self.setup_realsense()

        # Start the blocking camera operations in a background thread
        self.camera_thread = threading.Thread(target=self.camera_loop)
        self.camera_thread.start()

        self.get_logger().info('camera_node ready. Processing running in background thread.')

    def setup_realsense(self):
        self.get_logger().info('Initializing RealSense...')
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(
            rs.stream.color, STREAM_WIDTH, STREAM_HEIGHT, rs.format.bgr8, STREAM_FPS)
        profile = self.pipeline.start(config)

        # Allow auto-exposure to settle
        for _ in range(15):
            self.pipeline.wait_for_frames()

        device = profile.get_device()
        for sensor in device.query_sensors():
            name = sensor.get_info(rs.camera_info.name)
            if 'RGB' in name or 'Color' in name:
                sensor.set_option(rs.option.enable_auto_exposure, 0)
                sensor.set_option(rs.option.exposure, MANUAL_EXPOSURE)
                self.get_logger().info(f"Locked exposure on '{name}' to {MANUAL_EXPOSURE}.")

        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.camera_matrix = np.array(
            [[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=np.float64)
        self.dist_coeffs = np.array(intr.coeffs, dtype=np.float64)

    def transform_to_base(self, tvec):
        """Transform a 3D point from camera frame to robot base frame (raw,
        no correction applied yet)."""
        cam_point = np.array([*tvec, 1.0])
        base_point = BASE_T_CAMERA @ cam_point
        return base_point[:3]

    def apply_robot_frame_correction(self, base_xyz):
        """Apply the empirical x/y calibration offset and hardcode z.
        Only used for the robot_frame topic -- never touches camera_frame."""
        corrected = np.array(base_xyz, dtype=np.float64)
        corrected[0] += OFFSET_X
        corrected[1] += OFFSET_Y
        corrected[2] = FIXED_Z
        return corrected

    def camera_loop(self):
        """Background thread loop for hardware polling and image processing."""
        try:
            while self.running and rclpy.ok():
                # wait_for_frames blocks ONLY this thread, keeping ROS executor free
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                frame = np.asanyarray(color_frame.get_data())
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                display = frame.copy()

                corners, ids, _ = detect_markers(gray)

                if ids is not None and len(ids) > 0:
                    aruco.drawDetectedMarkers(display, corners, ids)
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                        corners, MARKER_LENGTH, self.camera_matrix, self.dist_coeffs)

                    for i, tag_id in enumerate(ids.flatten()):
                        tag_id = int(tag_id)
                        rvec, tvec = rvecs[i][0], tvecs[i][0]
                        quat = rvec_to_quat(rvec)

                        if tag_id not in self.accumulators:
                            self.accumulators[tag_id] = TagAccumulator()

                        acc = self.accumulators[tag_id]
                        just_confirmed = acc.add(tvec, quat)

                        # Draw Visuals
                        cv2.drawFrameAxes(
                            display, self.camera_matrix, self.dist_coeffs,
                            rvec, tvec, MARKER_LENGTH * 0.5)

                        n = len(acc.positions)
                        label = f"id{tag_id}: {n}/{SAMPLES_NEEDED}" + (" CONFIRMED" if acc.confirmed else "")
                        corner_pt = tuple(corners[i][0][0].astype(int))
                        cv2.putText(display, label, corner_pt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        if just_confirmed:
                            self.get_logger().info(f"Tag {tag_id} CONFIRMED.")

                # Drop tags that haven't been re-seen recently (Clean OOP check)
                for tag_id in list(self.accumulators.keys()):
                    if self.accumulators[tag_id].is_stale():
                        self.get_logger().info(f"Tag {tag_id} went stale. Dropping.")
                        del self.accumulators[tag_id]

                # Publish data to ROS
                self.publish_objects()

                # GUI Display
                cv2.imshow('camera_node (q to quit)', display)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.get_logger().info('Quit key pressed in OpenCV window. Shutting down thread...')
                    self.running = False
                    self.get_logger().warn('Please press Ctrl+C in the terminal to kill the ROS node.')
                    break

        except Exception as e:
            self.get_logger().error(f"Error in camera loop: {e}")
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()

    def publish_objects(self):
        """Constructs and publishes the ROS 2 messages."""
        cam_msg = DetectedObjects()
        base_msg = DetectedObjects()

        for tag_id, acc in self.accumulators.items():
            if not acc.confirmed:
                continue

            # Camera Frame Output (raw, no correction)
            cam_obj = DetectedObject()
            cam_obj.id = tag_id
            cam_obj.x, cam_obj.y, cam_obj.z = [float(v) for v in acc.confirmed_pos]
            cam_msg.objects.append(cam_obj)

            # Robot Base Frame Output -- transform, then apply x/y offset + fixed z
            base_xyz = self.transform_to_base(acc.confirmed_pos)
            base_xyz = self.apply_robot_frame_correction(base_xyz)
            base_obj = DetectedObject()
            base_obj.id = tag_id
            base_obj.x, base_obj.y, base_obj.z = [float(v) for v in base_xyz]
            base_msg.objects.append(base_obj)

        self.pub_camera_frame.publish(cam_msg)
        self.pub_robot_frame.publish(base_msg)

    def destroy_node(self):
        self.running = False
        if self.camera_thread.is_alive():
            self.camera_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received, stopping...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()  