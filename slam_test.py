import os
os.add_dll_directory(r"D:\gstreamer\1.0\msvc_x86_64\bin")
import cv2
import numpy as np
import threading
from skimage.measure import LineModelND, ransac

# Add GStreamer DLL directories

# GStreamer pipeline
pipeline = (
    "udpsrc address=192.168.2.1 port=5600 ! "
    "application/x-rtp, encoding-name=H264, payload=96, clock-rate=90000 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! appsink"
)

# Create VideoCapture object
cap = cv2.VideoCapture(r"D:\PhD_1\bluerov\IMG_6135.MP4", cv2.CAP_GSTREAMER)

# Feature detection parameters
MIN_MATCH_COUNT = 20
MAX_FEATURES = 1000  # Maximum number of features

# Create ORB detector
orb = cv2.ORB_create(nfeatures=MAX_FEATURES)

# Check if video opened successfully
if not cap.isOpened():
    print("Unable to open video source")
    exit()

# Read the first frame
ret, prev_frame = cap.read()
if not ret:
    print("Unable to read video frame")
    exit()

# Downsample the frame
scale_percent = 50  # Reduce frame size to 50%
width = int(prev_frame.shape[1] * scale_percent / 100)
height = int(prev_frame.shape[0] * scale_percent / 100)
dim = (width, height)
prev_frame = cv2.resize(prev_frame, dim, interpolation=cv2.INTER_AREA)

# Convert to grayscale
prev_gray = cv2.cvtColor(prev_frame, cv2.COLOR_BGR2GRAY)

# Detect initial features
prev_pts = cv2.goodFeaturesToTrack(prev_gray, mask=None, maxCorners=MAX_FEATURES,
                                   qualityLevel=0.01, minDistance=7, blockSize=7)
if prev_pts is not None:
    prev_keypoints = [cv2.KeyPoint(f[0][0], f[0][1], 20) for f in prev_pts]
    prev_keypoints, prev_descriptors = orb.compute(prev_gray, prev_keypoints)
else:
    print("Unable to detect initial features")
    exit()

# Initialize trajectory image
trajectory = np.zeros((height, width, 3), dtype=np.uint8)

# Initialize position and orientation
cur_pos = np.zeros((3, 1))
cur_orient = np.eye(3)

# Camera intrinsic matrix (calibrated values)
K = np.array([[1033.56461722950, 0, 932.395121907296],
              [0, 1037.40700075230, 521.775806154061],
              [0, 0, 1]])

# Scale the intrinsic parameters according to the resized frame
scale_factor = scale_percent / 100
K[0, 0] *= scale_factor  # fx
K[1, 1] *= scale_factor  # fy
K[0, 2] *= scale_factor  # cx
K[1, 2] *= scale_factor  # cy

# Enable OpenCV optimization
cv2.setUseOptimized(True)

# Define a lock for thread synchronization
lock = threading.Lock()

# Variable to store the frame with keypoints
frame_with_keypoints = None

def process_frame():
    global prev_gray, prev_keypoints, prev_descriptors, cur_pos, cur_orient, trajectory, frame_with_keypoints
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Downsample the frame
        frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Optical flow to track features
        p0 = np.array([kp.pt for kp in prev_keypoints], dtype=np.float32).reshape(-1, 1, 2)
        p1, st, err = cv2.calcOpticalFlowPyrLK(prev_gray, gray, p0, None)

        # Check if optical flow tracking succeeded
        if p1 is None or len(p1) == 0:
            print("Optical flow tracking failed, re-detecting features")
            prev_pts = cv2.goodFeaturesToTrack(prev_gray, mask=None, maxCorners=MAX_FEATURES,
                                               qualityLevel=0.01, minDistance=7, blockSize=7)
            if prev_pts is not None:
                prev_keypoints = [cv2.KeyPoint(f[0][0], f[0][1], 20) for f in prev_pts]
                prev_keypoints, prev_descriptors = orb.compute(prev_gray, prev_keypoints)
            else:
                print("Unable to detect sufficient features")
            prev_gray = gray.copy()
            continue

        # Select good tracking points
        st = st.reshape(-1)
        good_new = p1[st == 1]
        good_old = p0[st == 1]

        # Re-detect features if tracking points are too few
        if len(good_new) < MIN_MATCH_COUNT:
            print("Too few tracking points, re-detecting features")
            prev_pts = cv2.goodFeaturesToTrack(prev_gray, mask=None, maxCorners=MAX_FEATURES,
                                               qualityLevel=0.01, minDistance=7, blockSize=7)
            if prev_pts is not None:
                prev_keypoints = [cv2.KeyPoint(f[0][0], f[0][1], 20) for f in prev_pts]
                prev_keypoints, prev_descriptors = orb.compute(prev_gray, prev_keypoints)
            else:
                print("Unable to detect sufficient features")
            prev_gray = gray.copy()
            continue

        # Reshape arrays
        good_new = good_new.reshape(-1, 2)
        good_old = good_old.reshape(-1, 2)

        # Compute the essential matrix
        E, mask_E = cv2.findEssentialMat(good_old, good_new, K, method=cv2.RANSAC, prob=0.999, threshold=1.0)

        # Recover pose from the essential matrix
        if E is not None and E.shape == (3, 3):
            _, R, t, mask_pose = cv2.recoverPose(E, good_old, good_new, K)

            # Use mask_pose to filter valid points
            mask_pose = mask_pose.ravel().astype(bool)
            good_new = good_new[mask_pose]
            good_old = good_old[mask_pose]

            # Lock when updating shared variables
            with lock:
                cur_orient = cur_orient @ R
                cur_pos += cur_orient @ t

                # Normalize the position for visualization (since scale is unknown)
                traj_scale = 1  # Adjust this scale factor as needed for visualization
                x, y, z = cur_pos.flatten() * traj_scale

                # Map coordinates to image size
                draw_x = int(x) + width // 2
                draw_y = height - (int(z) + height // 2)

                # Ensure coordinates are within image bounds
                if 0 <= draw_x < width and 0 <= draw_y < height:
                    cv2.circle(trajectory, (draw_x, draw_y), 1, (0, 0, 255), 2)
        else:
            print("Unable to compute the essential matrix")
            prev_gray = gray.copy()
            continue

        # Draw keypoints
        with lock:
            frame_with_keypoints_local = frame.copy()
            keypoints_to_draw = [cv2.KeyPoint(pt[0], pt[1], 1) for pt in good_new]
            frame_with_keypoints_local = cv2.drawKeypoints(frame_with_keypoints_local, keypoints_to_draw, None, color=(0, 255, 0), flags=0)
            frame_with_keypoints = frame_with_keypoints_local

        # Update previous frame information
        prev_gray = gray.copy()
        prev_keypoints = [cv2.KeyPoint(pt[0], pt[1], 20) for pt in good_new]

def display():
    while True:
        with lock:
            # Copy shared variables under lock
            trajectory_copy = trajectory.copy()
            frame_with_keypoints_copy = frame_with_keypoints.copy() if frame_with_keypoints is not None else None

        # Display trajectory
        cv2.imshow('Trajectory', trajectory_copy)

        # Display frame with keypoints
        if frame_with_keypoints_copy is not None:
            cv2.imshow('Frame with Keypoints', frame_with_keypoints_copy)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release resources
    cap.release()
    cv2.destroyAllWindows()

# Start the processing thread
thread = threading.Thread(target=process_frame)
thread.start()

# Display the results
display()

thread.join()
