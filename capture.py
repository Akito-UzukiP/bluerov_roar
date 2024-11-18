import cv2
# ffmpeg -protocol_whitelist "file,udp,rtp" -i "H264 USB Camera_ USB Camera.sdp" -f mpegts udp://localhost:1234
# 读取本地 UDP 视频流
cap = cv2.VideoCapture("udp://localhost:1234")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
    
    # 显示视频帧
    cv2.imshow('Video Stream', frame)
    
    # 按 'q' 退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
