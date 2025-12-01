# Code is used to transfer data from Pi to ESP

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
import serial
import time
import math

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

WHEEL_RADIUS = 0.035  # meters
BASE_WIDTH = 0.20     # distance between wheels, meters

# --- Global State and Serial Connection ---
ser = None
last_command_time = 0.0
timeout_duration = 0.5 # Stop motors if no command received for this long (seconds)

def twist_callback(msg):
    """
    Called every time a new velocity command (Twist message) is received.
    Translates linear/angular velocity into left/right wheel velocities.
    """
    global last_command_time
    last_command_time = rospy.get_time()

    linear_x = msg.linear.x    # Forward/Backward speed (m/s)
    angular_z = msg.angular.z  # Turning speed (rad/s)
    # The linear and angular components are summed/differenced
    v_right = linear_x + (angular_z * BASE_WIDTH / 2.0)
    v_left = linear_x - (angular_z * BASE_WIDTH / 2.0)
    command_string = "L:{:.3f},R:{:.3f};\n".format(v_left, v_right)

    if ser and ser.is_open:
        try:
            ser.write(command_string.encode('utf-8'))
            # rospy.loginfo(f"Sent: {command_string.strip()}") # Uncomment for debugging
        except serial.SerialException as e:
            rospy.logerr("Serial write error: {}".format(e))

def setup_serial():
    """Initializes the serial connection."""
    global ser
    rospy.loginfo("Attempting to connect to serial port {}...".format(SERIAL_PORT))
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        # Give the connection time to establish and flush buffers
        time.sleep(2)
        ser.flush()
        rospy.loginfo("Serial connection established successfully.")
    except serial.SerialException as e:
        rospy.logerr("Could not open serial port {}: {}".format(SERIAL_PORT, e))
        rospy.logwarn("Make sure the ESP32 is plugged in and recognized.")

def shutdown_hook():
    """Stops the robot and closes the serial port safely."""
    rospy.loginfo("Shutting down Pi Serial Commander. Sending stop command...")
    if ser and ser.is_open:
        try:
            # Send zero velocity command (L:0.0, R:0.0)
            ser.write("L:0.0,R:0.0;\n".encode('utf-8'))
            ser.close()
        except serial.SerialException as e:
            rospy.logwarn("Error closing serial port: {}".format(e))
    rospy.loginfo("Serial port closed.")

def main():
    """Main execution function."""
    rospy.init_node('pi_serial_commander', anonymous=True)

    # Set up serial connection
    setup_serial()

    # Register shutdown hook
    rospy.on_shutdown(shutdown_hook)

    # Subscribe to the velocity topic (this is where ROS Navigation publishes commands)
    rospy.Subscriber('/cmd_vel', Twist, twist_callback)
    rospy.loginfo("Subscribed to /cmd_vel topic.")

    rate = rospy.Rate(10) # 10 Hz loop rate for watchdog

    while not rospy.is_shutdown():
        # Watchdog: If no command has been received recently, stop the motors
        if ser and ser.is_open:
            if rospy.get_time() - last_command_time > timeout_duration:
                # Send zero velocity command to prevent runaway robot
                stop_command = "L:0.0,R:0.0;\n"
                ser.write(stop_command.encode('utf-8'))
                # rospy.logwarn("Watchdog timeout: Sent stop command.") # Uncomment for debugging

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
