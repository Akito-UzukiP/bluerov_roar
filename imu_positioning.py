from pymavlink import mavutil, mavwp
import time
import sys
import keyboard
from pymavlink.quaternion import QuaternionBase
import numpy as np
import math
import tqdm

# 定义RC通道
rc_channel = {
    "pitch": 1,
    "roll": 2,
    "vz": 3,   # 垂直速度（上/下）
    "yaw": 4,
    "vx": 5,   # 前进速度
    "vy": 6,   # 侧向速度
    "light": 9  # 灯光控制通道
}

#imu_offset = np.array([-0.01, -0.07, 0, 0, 0, 0], dtype=np.float64)

# 连接到蓝鲸ROV
master = mavutil.mavlink_connection('udpin:192.168.2.1:14560', baudrate=115200)
master.wait_heartbeat()  # 等待心跳信号

def calculate_accelerate_gyro(raw_imu, roll, pitch, yaw) -> list:
    """
    计算经过重力补偿后的x、y、z方向的加速度和陀螺仪数据。
    """
    GFORCE = 9.80665  # 重力加速度
    # 获取加速度计数据并转换为m/s^2
    accs = np.array([raw_imu.xacc, raw_imu.yacc, raw_imu.zacc], dtype=np.float64)
    accs = accs / 1000 * GFORCE  # 从mg转换为m/s^2
    #accs += imu_offset[:3]  # 应用偏移

    # 将姿态角从度转换为弧度
    roll_rad = math.radians(roll)
    pitch_rad = math.radians(pitch)
    yaw_rad = math.radians(yaw)

    # 计算旋转矩阵
    R_roll = np.array([
        [1, 0, 0],
        [0, np.cos(roll_rad), -np.sin(roll_rad)],
        [0, np.sin(roll_rad), np.cos(roll_rad)]
    ])

    R_pitch = np.array([
        [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
        [0, 1, 0],
        [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
    ])

    R_yaw = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [np.sin(yaw_rad), np.cos(yaw_rad), 0],
        [0, 0, 1]
    ])

    # 总的旋转矩阵
    R = R_yaw @ R_pitch @ R_roll

    # 地球坐标系下的重力矢量
    gravity_earth = np.array([0, 0, GFORCE])

    # 将重力矢量转换到机体坐标系
    gravity_body = R.T @ gravity_earth  # 使用转置矩阵将矢量从地球坐标系转换到机体坐标系

    # 从加速度计测量值中减去重力分量
    accs_corrected = accs + gravity_body

    # 获取并处理陀螺仪数据
    gyros = np.array([raw_imu.xgyro, raw_imu.ygyro, raw_imu.zgyro], dtype=np.float64)
    # gyros = gyros / 1000  # 从mdeg/s转换为deg/s
    # gyros += imu_offset[3:]

    # 合并加速度和陀螺仪数据
    sensor_data = np.concatenate((accs_corrected, gyros))

    return sensor_data, gravity_body

pbar = tqdm.tqdm()
import time
timestamps = []
dt = 0+0.0000001
v = np.array([0, 0, 0], dtype=np.float64)
while True:
    raw_imu = master.recv_match(type='RAW_IMU', blocking=True)
    rpy = master.recv_match(type='AHRS2', blocking=True)
    roll = math.degrees(rpy.roll)
    pitch = math.degrees(rpy.pitch)
    yaw = math.degrees(rpy.yaw)
    #print(raw_imu)

    accs, gravity = calculate_accelerate_gyro(raw_imu, roll, pitch, yaw)
    timestamps.append(time.time())
    if len(timestamps) > 1:
        timestamp_diff = sum([timestamps[i] - timestamps[i-1] for i in range(1, len(timestamps))]) / (len(timestamps) - 1)
        dt = timestamp_diff
    # 对于不显著的运动，删除小于0.1的值
    #accs = np.where(np.abs(accs) < 0.1, 0, accs)
    v = v + accs[:3] * dt

    pbar.set_description(f"X: {accs[0]:.2f}, Y: {accs[1]:.2f}, Z: {accs[2]:.2f}, Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f},  V: {v}")
    pbar.update(1)
