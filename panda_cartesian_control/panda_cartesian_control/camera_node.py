#!/usr/bin/env python3
"""
camera_node.py

Dynamic real-time camera + frame-transform ROS 2 node.
Optimized for high-rate (30 FPS) tracking with minimal latency for dynamic replanning.
"""

import time
import cv2
import numpy as np
import pyrealsense2 as rs
from cv2 import aruco
import threading

import rclpy
from rclpy.node import Node

from panda_cartesian_control_msgs.msg import DetectedObject, DetectedObjects


# ---------------------------------------------------------------------------
# Camera / detection config
# ---------------------------------------------------------------------------
MARKER_LENGTH = 0.0475
DICTIONARY = aruco.DICT_APRILTAG_36h11
STREAM_WIDTH, STREAM_HEIGHT, STREAM_FPS = 1280, 720, 30
MANUAL_EXPOSURE = 150
MIN_MARKER_PERIMETER_RATE = 0.01

# ---------------------------------------------------------------------------
# Dynamic tracking & low-latency config
# ---------------------------------------------------------------------------
SAMPLES_NEEDED = 5             # Initial samples to confirm tag on startup (~160ms @ 30 FPS)
MOVE_THRESHOLD_M = 0.03        # Distance threshold (3 cm) to classify as physical move
MOVE_CONFIRM_SAMPLES = 3       # Consecutive frames to confirm real move (~100ms @ 30 FPS)
EMA_ALPHA = 0.4                # Exponential smoothing factor for static/micro-movements
STALE_TIMEOUT_SEC = 1.0        # Rapidly drop tags if hidden or obscured

# ---------------------------------------------------------------------------
# Camera -> robot base transform (eye-to-hand)
# ---------------------------------------------------------------------------
BASE_T_CAMERA = np.array([
    [-0.0000101775, -0.964732004, -0.055322304, 0.558715725],
    [-0.965277885,  -0.002554345,  0.044708214, 0.042059743],
    [-0.046342000,   0.057190000, -0.997287000, 1.728049000],
    [ 0.0,           0.0,          0.0,           1.0],
])

# ---------------------------------------------------------------------------
# Robot-frame correction
# ---------------------------------------------------------------------------
OFFSET_X = 0.0     # meters
OFFSET_Y = 0.0     # meters
FIXED_Z = 0.15     # meters override

# ---------------------------------------------------------------------------
# ArUco Setup
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
# Dynamic Accumulator & Filter
# ---------------------------------------------------------------------------
class TagAccumulator:
    """Low-latency sample filter: tracks position continuously, filters camera noise,
    and seamlessly updates during object motion without dropping stream active status."""

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

        # Phase 1: Initial Lock
        if not self.confirmed:
            self.positions.append(tvec)
            self.quats.append(quat)

            if len(self.positions) >= SAMPLES_NEEDED:
                self.confirmed_pos = np.mean(self.positions, axis=0)
                self.confirmed_quat = average_quaternions(self.quats)
                self.confirmed = True
                return True
            return False

        # Phase 2: Dynamic Tracking & Smooth Updates
        deviation = np.linalg.norm(tvec - self.confirmed_pos)

        if deviation > MOVE_THRESHOLD_M:
            # Shift detected, buffer candidate samples
            self.move_candidates_pos.append(tvec)
            self.move_candidates_quat.append(quat)

            if len(self.move_candidates_pos) >= MOVE_CONFIRM_SAMPLES:
                cand = np.array(self.move_candidates_pos)
                cand_mean = np.mean(cand, axis=0)
                cand_spread = np.max(np.linalg.norm(cand - cand_mean, axis=1))

                if cand_spread < MOVE_THRESHOLD_M:
                    # Valid move confirmed -> update immediately while staying confirmed
                    self.confirmed_pos = cand_mean
                    self.confirmed_quat = average_quaternions(self.move_candidates_quat)
                    self.positions = list(self.move_candidates_pos)
                    self.quats = list(self.move_candidates_quat)
                    self.move_candidates_pos = []
                    self.move_candidates_quat = []
                    return True
                else:
                    # Drop camera noise/glitch outlier
                    self.move_candidates_pos.pop(0)
                    self.move_candidates_quat.pop(0)
        else:
            # Sub-threshold micro-movement -> Exponential Moving Average (EMA) smoothing
            self.confirmed_pos = (1.0 - EMA_ALPHA) * self.confirmed_pos + EMA_ALPHA * tvec
            self.move_candidates_pos = []
            self.move_candidates_quat = []

        return False

    def is_stale(self):
        return (time.time() - self.last_seen) > STALE_TIMEOUT_SEC


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.pub_camera_frame = self.create_publisher(
            DetectedObjects, 'detected_objects/camera_frame', 10)
        self.pub_robot_frame = self.create_publisher(
            DetectedObjects, 'detected_objects/robot_frame', 10)

        self.accumulators = {}
        self.running = True

        self.setup_realsense()

        self.camera_thread = threading.Thread(target=self.camera_loop)
        self.camera_thread.start()

        self.get_logger().info('camera_node ready at 30 FPS (Dynamic Tracking Mode).')

    def setup_realsense(self):
        self.get_logger().info('Initializing RealSense at 30 FPS...')
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(
            rs.stream.color, STREAM_WIDTH, STREAM_HEIGHT, rs.format.bgr8, STREAM_FPS)
        profile = self.pipeline.start(config)

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
        cam_point = np.array([*tvec, 1.0])
        base_point = BASE_T_CAMERA @ cam_point
        return base_point[:3]

    def apply_robot_frame_correction(self, base_xyz):
        corrected = np.array(base_xyz, dtype=np.float64)
        corrected[0] += OFFSET_X
        corrected[1] += OFFSET_Y
        corrected[2] = FIXED_Z
        return corrected

    def camera_loop(self):
        try:
            while self.running and rclpy.ok():
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
                        moved = acc.add(tvec, quat)

                        cv2.drawFrameAxes(
                            display, self.camera_matrix, self.dist_coeffs,
                            rvec, tvec, MARKER_LENGTH * 0.5)

                        label = f"id{tag_id}" + (" [TRACKING]" if acc.confirmed else " [LOCKING]")
                        corner_pt = tuple(corners[i][0][0].astype(int))
                        cv2.putText(display, label, corner_pt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        if moved:
                            self.get_logger().info(f"Tag {tag_id} target position updated.")

                # Remove stale tags
                for tag_id in list(self.accumulators.keys()):
                    if self.accumulators[tag_id].is_stale():
                        self.get_logger().info(f"Tag {tag_id} lost/stale. Dropping.")
                        del self.accumulators[tag_id]

                self.publish_objects()

                cv2.imshow('camera_node (q to quit)', display)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.running = False
                    break

        except Exception as e:
            self.get_logger().error(f"Error in camera loop: {e}")
        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()

    def publish_objects(self):
        cam_msg = DetectedObjects()
        base_msg = DetectedObjects()

        for tag_id, acc in self.accumulators.items():
            if not acc.confirmed:
                continue

            cam_obj = DetectedObject()
            cam_obj.id = tag_id
            cam_obj.x, cam_obj.y, cam_obj.z = [float(v) for v in acc.confirmed_pos]
            cam_msg.objects.append(cam_obj)

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