from pymavlink import mavutil

# 连接到监听端口
#listener = mavutil.mavlink_connection('udpin:192.168.2.1:14560', baudrate=115200)\
listener = mavutil.mavlink_connection('udpin:localhost:14445')
# QGroundControl 的默认组件 ID
QGC_COMPONENT_ID = 190  # 常见的 QGroundControl ID
ids = [190, 194]
# 持续监听并过滤消息
while True:
    message = listener.recv_match(blocking=True)
    if message:
        print(message.get_srcComponent())
        # 检查消息的来源
        if message.get_srcComponent() in ids:
            print(f"QGroundControl sent: {message}")
# 持续接收并打印消息
while True:
    message = listener.recv_match(blocking=True)
    if message:
        #print(f"Received message: {message}")
        print(message.get_srcComponent())
