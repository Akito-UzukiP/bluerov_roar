import os
import sys
os.add_dll_directory(r"D:\gstreamer\1.0\msvc_x86_64\bin")
import cv2
import numpy as np
import time
from pupil_apriltags import Detector
# 添加scikit-image库
from skimage.measure import LineModelND, ransac
from pymavlink import mavutil,mavwp
import time
import math
import json
import tqdm
import ultralytics
model = ultralytics.YOLO("yolo11x.pt")
obj_classes = {}

rc_channel = {"pitch": 1,
                "roll": 2,
                "vz": 3,
                "yaw": 4,
                "vx": 5,
                "vy": 6,
                "camera_tilt": 8,}
master = mavutil.mavlink_connection('udpin:192.168.2.1:14560',baudrate=115200)
master.wait_heartbeat()
def arm_rov():
    """
    ARM ROV
        Current Understanding: Allows motors to move
        NOTE: Messages can still be sent with out arming the rov
        NOTE: No acknowledge function call is needed as  built in acknowledge is used
    """
    # Arm
    # master.arducopter_arm() or:
    master.arducopter_arm()
    master.mav.command_long_send(
        master.target_system,
        master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1, 0, 0, 0, 0, 0, 0)

    # wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')
def disarm_rov():
    """
    DISARM ROV
        Current Understanding: Disables motors from moving
        NOTE: Should be used for saftey precations
        NOTE: No acknowledge function call is needed as  built in acknowledge is used

    """
    # Disarm
    # master.arducopter_disarm() or:
    master.mav.command_long_send(master.target_system,master.target_component,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0, 0, 0, 0, 0, 0, 0, 0)

    # wait until disarming confirmed
    master.motors_disarmed_wait()
    print("Disarmed")
    system_armed = False
def set_rc_channel_pwm(channel_id, pwm=1500):
    """ 
    SEND THRUST COMMAND

        WARNING: If not used correctly seriouse dammange can be caused to BLUEROV
        NOTE: Thrust commands range from 1100 - 1900 and are (0) at 1500.   

        INPUT:
            channel_id
                - BLUEROV uses channels stored in rc_channels dictionary.  Mapping is declared there.
            pwm
                - initially set to 1500... therefore no thrust
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return
    
    # pwm = max(1400, min(1600, pwm))

    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.
    



# GStreamer pipeline
pipeline = (
    "udpsrc address=192.168.2.1 port=5601 ! "
    "application/x-rtp, encoding-name=H264, payload=96, clock-rate=90000 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! appsink"
)

# 创建VideoCapture对象
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
cnt = 0
if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()
if not os.path.exists('cablirate_pictures3'):
    os.makedirs('cablirate_pictures3')
current_time = time.time()
camera_matrix = np.array([[1033.56461722950, 0, 932.395121907296], [0, 1037.40700075230, 521.775806154061], [0, 0, 1]])

apriltag_square_length = 57  # mm
world_corners = np.array([
    [-apriltag_square_length/2, -apriltag_square_length/2, 0],
    [ apriltag_square_length/2, -apriltag_square_length/2, 0],
    [ apriltag_square_length/2,  apriltag_square_length/2, 0],
    [-apriltag_square_length/2,  apriltag_square_length/2, 0]
], dtype=np.float32)



#dist_coeffs = np.array([0.00121849320859622, 0.0161138009538012, 0, 0])
dist_coeffs = np.zeros((4, 1))
laplace_filter = -np.array([[0, 1, 0], [1, -5, 1], [0, 1, 0]])
at_detector = Detector(
   families="tag36h11",
   nthreads=8,
   quad_decimate=1.5,
   quad_sigma=0.8,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0,

)
axis_length = 57  # 以米为单位的轴长
# 定义用于绘制RGB坐标系的3D点（单位向量）
axis_points = np.float32([
    [axis_length, 0, 0],    # X轴，红色
    [0, axis_length, 0],    # Y轴，绿色
    [0, 0, axis_length],   # Z轴，蓝色
]).reshape(-1, 3)
target_apriltag_pos = 1080 / 2
fourcc = cv2.VideoWriter_fourcc(*'MJPG')  # 编码格式
out = cv2.VideoWriter('output.avi', fourcc, 15.0, (int(1920), int(1080)))  # 视频输出文件
point_data = [] # 存储相对位置数据，将apriltag视为原点，计算自己的相对位置
xyz_data = [] # 存储绝对位置数据，将apriltag视为绝对原点，计算自己的绝对位置
import tqdm
pbar = tqdm.tqdm()
while True:
    ret, img = cap.read()
    if not ret:
        print("Failed to grab img")
        break

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    tags = at_detector.detect(gray, camera_params=(1033.56461722950, 1037.40700075230, 932.395121907296, 521.775806154061))

    if len(tags) == 0:
        set_rc_channel_pwm(rc_channel["camera_tilt"], 1500)  # 无tag，停止运动
    else:
        for tag in tags:
            #print(tag)
            # 绘制AprilTag的角点
            for idx in range(len(tag.corners)):
                cv2.line(img, tuple(tag.corners[idx-1, :].astype(int)),
                         tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))

            # 获取图像坐标系中的角点
            image_corners = tag.corners.astype(np.float32)
            pose_r, pose_t = tag.pose_R, tag.pose_t
            center = tuple(tag.center.astype(int))

            # solvePnP
            ret, rvec, tvec = cv2.solvePnP(world_corners, image_corners, camera_matrix, dist_coeffs)
            rmat, _ = cv2.Rodrigues(rvec)
            # 画RGB坐标轴
            #imgpts, jac = cv2.projectPoints(axis_points, pose_r, pose_t, camera_matrix, dist_coeffs)
            imgpts, jac = cv2.projectPoints(axis_points, rmat, tvec, camera_matrix, dist_coeffs)
            img = cv2.line(img, center, tuple(imgpts[0].ravel().astype(int)), (0, 0, 255), 3)
            img = cv2.line(img, center, tuple(imgpts[1].ravel().astype(int)), (0, 255, 0), 3)
            img = cv2.line(img, center, tuple(imgpts[2].ravel().astype(int)), (255, 0, 0), 3)

            if tag.center[1] != target_apriltag_pos:
                error = target_apriltag_pos - tag.center[1]
                set_rc_channel_pwm(rc_channel["camera_tilt"], int(1500 + error))
            try:
                Rinv = rmat.T
                Tinv = -np.dot(Rinv, tvec)
                print(Tinv)
                pov_xyz = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=np.float32).dot(Rinv)
                pov_point =  Rinv @ np.array([0, 0, 0], dtype=np.float32).reshape(-1, 3) + Tinv
                print(tvec)
                break
            except:
                continue
            pbar.set_description(f"pov_point: {pov_point}, pov_xyz: {pov_xyz}")
            pbar.update(1)

    # Perform object detection
    results = model(img)
    
    # 绘制检测结果
    for result in results:
        boxes = result.boxes.cpu().numpy()
        for box in boxes:
            # 获取边界框坐标
            x1, y1, x2, y2 = box.xyxy[0].astype(int)
            # 获取置信度
            confidence = box.conf[0]
            # 获取类别
            class_id = int(box.cls[0])
            class_name = result.names[class_id]
            
            # 绘制边界框
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 准备标签文本
            label = f'{class_name}: {confidence:.2f}'
            
            # 绘制标签背景
            (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            cv2.rectangle(img, (x1, y1 - label_height - 10), (x1 + label_width, y1), (0, 255, 0), -1)
            
            # 绘制标签文本
            cv2.putText(img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)
    
    # 缩放并显示图像
    #img = cv2.resize(img, (int(1920/2), int(1080/2)))
    cv2.imshow('preview', img)
    # out.write(img)  # 记录视频
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
out.release()