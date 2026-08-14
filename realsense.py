#!/usr/bin/env python3
"""
Standalone, no ROS. Detects AprilTags, and for each tag ID, accumulates
samples until it has 100, then prints the averaged (confirmed) position and
orientation.

Samples do NOT need to be consecutive frames - dropouts/occlusions don't
reset anything. If the tag is physically moved -- whether before or AFTER
confirmation -- several consistent samples in the new location are detected
as a genuine move (not noise), and the accumulator un-confirms and restarts
from the new position. This keeps running for the lifetime of the script,
so a tag can be re-confirmed as many times as it actually moves.

Install once: pip3 install pyrealsense2 opencv-contrib-python numpy
Run: python3 apriltag_confirm.py
Press 'q' to quit.
"""
import cv2
import numpy as np
import pyrealsense2 as rs
from cv2 import aruco
import json

OUTPUT_FILE = "/tmp/detected_objects.json"

MARKER_LENGTH = 0.05
DICTIONARY = aruco.DICT_APRILTAG_36h11
STREAM_WIDTH, STREAM_HEIGHT = 1280, 720
MANUAL_EXPOSURE = 150
MIN_MARKER_PERIMETER_RATE = 0.01

SAMPLES_NEEDED = 15
MOVE_THRESHOLD_M = 0.03        # how far a sample must be from the reference to look like a move
MOVE_CONFIRM_SAMPLES = 5       # how many consistent samples in a row confirm a real move

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
        qw = 0.25 * S; qx = (m[2,1]-m[1,2])/S; qy = (m[0,2]-m[2,0])/S; qz = (m[1,0]-m[0,1])/S
    else:
        i = np.argmax([m[0,0], m[1,1], m[2,2]])
        if i == 0:
            S = np.sqrt(1.0+m[0,0]-m[1,1]-m[2,2])*2
            qw=(m[2,1]-m[1,2])/S; qx=0.25*S; qy=(m[0,1]+m[1,0])/S; qz=(m[0,2]+m[2,0])/S
        elif i == 1:
            S = np.sqrt(1.0+m[1,1]-m[0,0]-m[2,2])*2
            qw=(m[0,2]-m[2,0])/S; qx=(m[0,1]+m[1,0])/S; qy=0.25*S; qz=(m[1,2]+m[2,1])/S
        else:
            S = np.sqrt(1.0+m[2,2]-m[0,0]-m[1,1])*2
            qw=(m[1,0]-m[0,1])/S; qx=(m[0,2]+m[2,0])/S; qy=(m[1,2]+m[2,1])/S; qz=0.25*S
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

class TagAccumulator:
    def __init__(self):
        self.positions = []
        self.quats = []
        self.confirmed = False
        self.confirmed_pos = None
        self.confirmed_quat = None
        # buffer of candidate samples that look like they belong to a new position
        self.move_candidates_pos = []
        self.move_candidates_quat = []

    def add(self, tvec, quat):
        # reference point to compare new samples against: the confirmed position
        # if we have one, otherwise the median of whatever we've collected so far
        if self.confirmed:
            reference = self.confirmed_pos
        elif self.positions:
            reference = np.median(self.positions, axis=0)
        else:
            reference = None

        if reference is not None:
            deviation = np.linalg.norm(tvec - reference)

            if deviation > MOVE_THRESHOLD_M:
                # doesn't match current reference -- might be noise, might be a real move.
                self.move_candidates_pos.append(tvec)
                self.move_candidates_quat.append(quat)

                if len(self.move_candidates_pos) >= MOVE_CONFIRM_SAMPLES:
                    cand = np.array(self.move_candidates_pos)
                    cand_mean = np.mean(cand, axis=0)
                    cand_spread = np.max(np.linalg.norm(cand - cand_mean, axis=1))

                    if cand_spread < MOVE_THRESHOLD_M:
                        # candidates agree with each other -> genuine move, not noise.
                        # wipe everything (including any prior confirmation) and
                        # restart accumulation using them as the new starting cluster.
                        self.positions = list(self.move_candidates_pos)
                        self.quats = list(self.move_candidates_quat)
                        self.move_candidates_pos = []
                        self.move_candidates_quat = []
                        self.confirmed = False
                        self.confirmed_pos = None
                        self.confirmed_quat = None
                    else:
                        # candidates are scattered -> just noise, drop the oldest and keep waiting
                        self.move_candidates_pos.pop(0)
                        self.move_candidates_quat.pop(0)
                return False
            else:
                # sample matches the current reference fine -- any pending move
                # candidates were noise, discard them
                self.move_candidates_pos = []
                self.move_candidates_quat = []
                if self.confirmed:
                    # already confirmed and still sitting in the same spot, nothing to do
                    return False

        self.positions.append(tvec)
        self.quats.append(quat)

        if len(self.positions) >= SAMPLES_NEEDED and not self.confirmed:
            self.confirmed_pos = np.mean(self.positions, axis=0)
            self.confirmed_quat = average_quaternions(self.quats)
            self.confirmed = True
            return True
        return False

accumulators = {}

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, STREAM_WIDTH, STREAM_HEIGHT, rs.format.bgr8, 15)
profile = pipeline.start(config)

for _ in range(15):
    pipeline.wait_for_frames()

device = profile.get_device()
for sensor in device.query_sensors():
    name = sensor.get_info(rs.camera_info.name)
    if 'RGB' in name or 'Color' in name:
        sensor.set_option(rs.option.enable_auto_exposure, 0)
        sensor.set_option(rs.option.exposure, MANUAL_EXPOSURE)
        print(f"Locked exposure on '{name}' to {MANUAL_EXPOSURE}.")

intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
camera_matrix = np.array([[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]], dtype=np.float64)
dist_coeffs = np.array(intr.coeffs, dtype=np.float64)

print(f"Stream: {STREAM_WIDTH}x{STREAM_HEIGHT}")
print(f"Collecting {SAMPLES_NEEDED} samples per tag before confirming. Press 'q' to quit.\n")

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        display = frame.copy()

        corners, ids, _ = detect_markers(gray)

        if ids is not None and len(ids) > 0:
            aruco.drawDetectedMarkers(display, corners, ids)
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, MARKER_LENGTH, camera_matrix, dist_coeffs)

            for i, tag_id in enumerate(ids.flatten()):
                tag_id = int(tag_id)
                rvec, tvec = rvecs[i][0], tvecs[i][0]
                quat = rvec_to_quat(rvec)

                if tag_id not in accumulators:
                    accumulators[tag_id] = TagAccumulator()
                acc = accumulators[tag_id]
                just_confirmed = acc.add(tvec, quat)

                cv2.drawFrameAxes(display, camera_matrix, dist_coeffs, rvec, tvec, MARKER_LENGTH * 0.5)
                n = len(acc.positions)
                label = f"id{tag_id}: {n}/{SAMPLES_NEEDED}" + (" CONFIRMED" if acc.confirmed else "")
                corner_pt = tuple(corners[i][0][0].astype(int))
                cv2.putText(display, label, corner_pt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if just_confirmed:
                    p, q = acc.confirmed_pos, acc.confirmed_quat
                    print(f"\n=== CONFIRMED tag id {tag_id} (avg of {SAMPLES_NEEDED} samples) ===")
                    print(f"position (m): x={p[0]:.6f} y={p[1]:.6f} z={p[2]:.6f}")
                    print(f"quaternion (x,y,z,w): [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]\n")

                    try:
                        with open(OUTPUT_FILE, 'r') as f:
                            all_tags = json.load(f)
                    except (FileNotFoundError, json.JSONDecodeError):
                        all_tags = {}

                    all_tags[str(tag_id)] = {
                        'x': float(p[0]),
                        'y': float(p[1]),
                        'z': float(p[2]),
                    }

                    with open(OUTPUT_FILE, 'w') as f:
                        json.dump(all_tags, f)


        cv2.imshow('AprilTag confirm (q to quit)', display)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    print("\nStopped.")
