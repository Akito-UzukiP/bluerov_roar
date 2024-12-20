import os
import sys
os.add_dll_directory(r"D:\gstreamer\1.0\msvc_x86_64\bin")
import cv2
import numpy as np
import time
from pymavlink import mavutil, mavwp
import math
import json
import tqdm
import ultralytics


# Load the YOLO model
model = ultralytics.YOLO("./best.pt")
obj_classes = {}

rc_channel = {
    "pitch": 1,
    "roll": 2,
    "vz": 3,
    "yaw": 4,
    "vx": 5,
    "vy": 6,
    "camera_tilt": 8,
}

# Establish MAVLink connection
master = mavutil.mavlink_connection('udpin:192.168.2.1:14560', baudrate=115200)
master.wait_heartbeat()

def arm_rov():
    """
    ARM ROV
    """
    master.arducopter_arm()
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')

def disarm_rov():
    """
    DISARM ROV
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print("Disarmed")
    system_armed = False

def set_rc_channel_pwm(channel_id, pwm=1500):
    """ 
    SEND THRUST COMMAND
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return
    
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    #print(rc_channel_values)
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values                   # RC channel list, in microseconds.
    )
def set_rc_channels_pwm_list(pwm_list=None):
    """ 
    Send thrust command for all 18 RC channels.

    Args:
        pwm_list (list): A list of 18 PWM values. 
                         Each value must be in the range 1100 to 1900, 
                         or 65535 to disable the channel. 
                         Defaults to 65535 for all channels if not provided.
    """
    # Default all channels to 65535 (disabled) if no input is given
    if pwm_list is None:
        pwm_list = [65535] * 18
    
    # Ensure the list has exactly 18 values
    if len(pwm_list) != 18:
        print("Error: PWM list must contain exactly 18 values.")
        return
    
    # Clamp valid PWM values to the range 1100-1900 or keep them as 65535
    rc_channel_values = [
        pwm if pwm == 65535 else max(1100, min(1900, pwm)) 
        for pwm in pwm_list
    ]

    # Send RC override message
    master.mav.rc_channels_override_send(
        master.target_system,                # target_system
        master.target_component,             # target_component
        *rc_channel_values                   # RC channel list.
    )

# GStreamer pipeline
pipeline = (
    "udpsrc address=192.168.2.1 port=5601 ! "
    "application/x-rtp, encoding-name=H264, payload=96, clock-rate=90000 ! "
    "rtph264depay ! avdec_h264 ! videoconvert ! appsink"
)

# Create VideoCapture object
cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
if not cap.isOpened():
    print("Error: Could not open video stream")
    exit()

# Define target position for object (center of the frame)
target_object_pos = 1080 / 2

# Video writer settings
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter('output.avi', fourcc, 15.0, (1920, 1080))

# Initialize progress bar
pbar = tqdm.tqdm()

while True:
    ret, img = cap.read()
    if not ret:
        print("Failed to grab image")
        break

    # Perform object detection
    results = model(img, verbose=False)
    
    # Check if any objects are detected
    objects_detected = False
    for result in results:
        boxes = result.boxes.cpu().numpy()
        if len(boxes) > 0:
            objects_detected = True
        for box in boxes:
            # Get bounding box coordinates
            x1, y1, x2, y2 = box.xyxy[0].astype(int)
            # Get confidence
            confidence = box.conf[0]
            # Get class_id and class_name
            class_id = int(box.cls[0])
            class_name = result.names[class_id]
            
            # Draw bounding box
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Prepare label text
            label = f'{class_name}: {confidence:.2f}'
            
            # Draw label background
            (label_width, label_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
            cv2.rectangle(img, (x1, y1 - label_height - 10), (x1 + label_width, y1), (0, 255, 0), -1)
            #print(f"Object detected: {label}, total_area: {abs(x2 - x1) * abs(y2 - y1)}")
            # Draw label text
            cv2.putText(img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)

            # Compute center of bounding box
            object_center_x = (x1 + x2) / 2
            object_center_y = (y1 + y2) / 2

            # Compute error between target position and object center position
            error = target_object_pos - object_center_y
            error_x = 1920 / 2 - object_center_x
            gain_x = -0.1 # reverse
            gain = 0.1
            gain_width = 4
            error = gain * error
            error_x = gain_x * error_x
            target_width = 150
            error_width =  target_width - abs(x2 - x1)
        
            print(error_width)
            pwms = [65535] * 18
            pwms[rc_channel["vx"] - 1] = int(1500 + error_width * gain_width)
            #print(f"Error_y: {error:.2f}, Error_x: {error_x:.2f}")
            # Adjust RC channel accordingly
            #set_rc_channel_pwm(rc_channel["camera_tilt"], int(1500 + error))
            pwms[rc_channel["pitch"] - 1] = int(1500 + error)
            pwms[rc_channel["yaw"] - 1] = int(1500 + error_x)
            # set_rc_channel_pwm(rc_channel["pitch"], int(1500 + error))
            # set_rc_channel_pwm(rc_channel["yaw"], int(1500 + error_x))
            #print(pwms)
            set_rc_channels_pwm_list(pwms)
            # Update progress bar
            pbar.set_description(f"Object center: ({object_center_x:.2f}, {object_center_y:.2f}), Pitch: {1500 + error}, Yaw: {1500 + error_x}, Object width:{abs(x2 - x1)}, Object height:{abs(y2 - y1)}")
            pbar.update(1)
            break
    
    if not objects_detected:
        # No objects detected, stop movement
        #set_rc_channel_pwm(rc_channel["camera_tilt"], 1500)
        # set_rc_channel_pwm(rc_channel["pitch"], 1500)
        # set_rc_channel_pwm(rc_channel["yaw"], 1500)
        channels_input = [65535] * 18
        # channels_input[rc_channel["pitch"] - 1] = 1500
        # channels_input[rc_channel["yaw"] - 1] = 1500
        set_rc_channels_pwm_list(channels_input)

    # Display the image
    cv2.imshow('preview', img)
    out.write(img)  # Record video
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
out.release()
cv2.destroyAllWindows()
