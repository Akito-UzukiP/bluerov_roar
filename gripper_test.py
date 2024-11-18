from pymavlink import mavutil

# Connect to the flight controller
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
master.wait_heartbeat()

# Define a function to set the PWM value of a servo channel
def set_servo_pwm(servo_n, pwm_value):
    master.mav.command_long_send(
        master.target_system,  # Target system
        master.target_component,  # Target component
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # Command
        0,  # Confirmation
        servo_n,  # Servo channel number
        pwm_value,  # PWM value
        0, 0, 0, 0, 0  # Other parameters remain default
    )

# Control the PWM value of servo channel 10 (Aux 2) to control the gripper
set_servo_pwm(10, 1900)  # Open the gripper
