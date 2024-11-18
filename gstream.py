import os
import sys
os.add_dll_directory(r"D:\gstreamer\1.0\msvc_x86_64\bin")
import cv2
import ultralytics

# GStreamer pipeline
pipeline = (
    "udpsrc address=192.168.2.1 port=5600 ! "
    "application/x-rtp, encoding-name=H264, payload=96, clock-rate=90000 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! appsink"
)


# Create a VideoCapture object
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame")
        break

    # Display the frame
    cv2.imshow("GStreamer Video", frame)

    # Exit the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close any OpenCV windows
cap.release()
cv2.destroyAllWindows()
