from pymavlink import mavutil,mavwp
import time
import math
import json
import tqdm
rc_channel = {"pitch": 1,
                "roll": 2,
                "vz": 3,
                "yaw": 4,
                "vx": 5,
                "vy": 6}
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
    
    pwm = max(1400, min(1600, pwm))

    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values)                  # RC channel list, in microseconds.

def view_message(msg=None,returndata=False,display_output=True):
    """
    VIEW MESSAGES   NOTE: will NOT loop
        By default a server communicating on a Mavlink channel will broadcast a specific set of mssages.  This fuction will print those messages to the terminal.
            NOTE: Connection obviously required
            NOTE: That messages shown are only limited to only those being broadcasted.  Non default messages must be requested to be broadcasted, and then asked to be printed to terminal to be shown or checked.

        Messages found in XML files (dialect)
            Available messages can be found in the xml files of those used during code generation.  
            i.e. common.xml: https://github.com/mavlink/mavlink/blob/master/message_definitions/v1.0/common.xml
            found between the <messages></messages> tags 

            Ever message is defined with a <message></message> (without the (s)) tag including an id and name attribute. Messages contain field tags (essentially a function parameter) with the folling attributed:    
                - type
                - name
                - enum
                - display
                - print format


        USEFUL MESSAGES:
            ATTITUDE
            BATTERY_STATUS['voltages'][0]
            LOCAL_POSITION_NED
            SYSTEM_TIME
            RAW_IMU
            RC_CHANNELS
            MISSION_CURRENT --> gives what seq it is on
            SYS_STATUS['voltage_battery']

        INPUT: 
            msg     type: STRING
                - set to None by default. This will broadcast all messages
                - if defined specific message then only that message will be shown on terminal
            forthismanyiterations       type: INT
                - defines how many loops the while loop will execute before exiting
                - type: int
                - if forthismanyiterations is set to 0 then the while loop will repeat forever. This is used for continuous printing it desired.
            atthisrate      type: INT
                - by default set to 0.1
                - NOTE: this default will cause the incoming messages to be delayed.  Needs to be set to 0 for there to be no delay
        OUTPUT: 
            - Print to terminal MAVLINK messages being 
            - tp
    """
    output = None
    try:
        output = master.recv_match(type=msg).to_dict()
        if display_output:
            print(output) # need t comment this out when actually implimenting
    except:
        pass
    if returndata and output != None:
        return output

# 初始化变量
position = [0.0, 0.0, 0.0]  # x, y, z
velocity = [0.0, 0.0, 0.0]  # vx, vy, vz
prev_time = None
trajectory = []

# 主循环
for _ in tqdm.tqdm(range(100)):  # 假设采集1000次数据
    current_time = time.time()
    if prev_time is None:
        dt = 0.1  # 初始时间间隔
    else:
        dt = current_time - prev_time
    prev_time = current_time

    # 获取消息数据
    imu_data = view_message("RAW_IMU")
    print(imu_data)
    ahrs2_data = view_message("AHRS2")
    #ahrs_data = view_message("AHRS")
    # 提取加速度数据（假设单位为mg，需要转换为m/s^2）
    xacc = imu_data['xacc'] * 9.80665 / 1000.0
    yacc = imu_data['yacc'] * 9.80665 / 1000.0
    zacc = imu_data['zacc'] * 9.80665 / 1000.0

    # 提取姿态角
    roll = ahrs2_data['roll']
    pitch = ahrs2_data['pitch']
    yaw = ahrs2_data['yaw']

    # 计算旋转矩阵
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    R = [
        [cp * cy, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [cp * sy, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr]
    ]

    # 将加速度从机体系转换到全局坐标系
    acc_body = [xacc, yacc, zacc]
    acc_global = [
        sum(R[0][i] * acc_body[i] for i in range(3)),
        sum(R[1][i] * acc_body[i] for i in range(3)),
        sum(R[2][i] * acc_body[i] for i in range(3))
    ]

    # 更新速度和位置
    velocity = [velocity[i] + acc_global[i] * dt for i in range(3)]
    position = [position[i] + velocity[i] * dt for i in range(3)]

    # 记录轨迹
    trajectory.append({
        'time': current_time,
        'position': position.copy(),
        'velocity': velocity.copy(),
        'orientation': {'roll': roll, 'pitch': pitch, 'yaw': yaw},
        'accs':acc_body
    })

    # 等待下一次采样
    time.sleep(0.05)

# 将轨迹保存到文件
with open('trajectory.json', 'w') as f:
    json.dump(trajectory, f, indent=4)