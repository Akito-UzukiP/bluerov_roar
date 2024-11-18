from pymavlink import mavutil
import time

# 创建 MAVLink 连接
master = mavutil.mavlink_connection('udp:192.168.2.1:14550')
print("MAVLink connection established.")
# 等待心跳包，以确认与飞行控制器连接
master.wait_heartbeat()
print("Heartbeat received. Connection established.")

def set_servo_pwm(servo_channel, pwm_value):
    """
    发送命令来设置特定舵机的 PWM 值。

    :param servo_channel: 要控制的舵机通道号 (通常 1-8)。
    :param pwm_value: PWM 值 (范围 1000-2000)。
    """
    master.mav.command_long_send(
        master.target_system,       # 目标系统 ID
        master.target_component,    # 目标组件 ID
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # 命令
        0,                          # 确认（不需要）
        servo_channel,              # 参数 1: 舵机通道
        pwm_value,                  # 参数 2: PWM 值
        0, 0, 0, 0, 0              # 剩余的参数不使用
    )
def arm_vehicle():
    """
    解锁飞控
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # 确认
        1,  # 1 表示解锁，0 表示上锁
        0, 0, 0, 0, 0, 0  # 剩余参数不使用
    )
    # 等待状态变更
    master.motors_armed_wait()
    print("Vehicle armed!")

def disarm_vehicle():
    """
    上锁飞控
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # 确认
        0,  # 1 表示解锁，0 表示上锁
        0, 0, 0, 0, 0, 0  # 剩余参数不使用
    )

    # 等待状态变更
    master.motors_disarmed_wait()
    print("Vehicle disarmed!")   



arm_vehicle()
print('vehicle armed')
set_servo_pwm(1, 1600)
print('servo 1 set to 1600')