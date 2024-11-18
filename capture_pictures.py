import os
import sys
os.add_dll_directory(r"D:\gstreamer\1.0\msvc_x86_64\bin")
import cv2
import numpy as np
import time

# 添加scikit-image库
from skimage.measure import LineModelND, ransac

# GStreamer pipeline
pipeline = (
    "udpsrc address=192.168.2.1 port=5600 ! "
    "application/x-rtp, encoding-name=H264, payload=96, clock-rate=90000 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! appsink"
)

# 创建VideoCapture对象
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
save_path = "yolo_training_data_rock_4"
if not os.path.exists(save_path):
    os.makedirs(save_path)
cnt = len(os.listdir(save_path))
if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()
current_time = time.time()
camera_matrix = np.array([[1033.56461722950, 0, 932.395121907296], [0, 1037.40700075230, 521.775806154061], [0, 0, 1]])
dist_coeffs = np.array([0.00121849320859622, 0.0161138009538012, 0, 0])
laplace_filter = -np.array([[0, 1, 0], [1, -5, 1], [0, 1, 0]])

last_time = time.time()
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame")
        break

    # 校正畸变
    calibrated_frame = cv2.undistort(frame, camera_matrix, dist_coeffs)
    
    # 显示结果
    cv2.imshow("GStreamer Video with Lines", calibrated_frame)

    # 手动捕获图像
    key = cv2.waitKey(1) & 0xFF
    # if key == ord('c'):
    #     cv2.imwrite(os.path.join(save_path, f"img_{cnt}.jpg"), frame)
    #     cnt += 1
    current_time = time.time()
    if current_time - last_time > 1:
        cv2.imwrite(os.path.join(save_path, f"img_{cnt}.jpg"), frame)
        cnt += 1
        last_time = current_time
    if key == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
