# Gives data from esp to raspberry Pi

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
import time
import re
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf.broadcaster import TransformBroadcaster
import tf.transformations as tf

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0'  # Must match the commander node
BAUD_RATE = 115200

# --- ROS Topics and Frames ---
ODOM_TOPIC = '/odom'
ODOM_FRAME = 'odom'
BASE_FRAME = 'base_link'

# --- Global State and Serial Connection ---
ser = None

def setup_serial():
    """Initializes the serial connection."""
    global ser
    rospy.loginfo("Attempting to connect to serial port {}...".format(SERIAL_PORT))
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1) # Shorter timeout for reading
        time.sleep(2)
        ser.flush()
        rospy.loginfo("Serial connection established successfully.")
    except serial.SerialException as e:
        rospy.logerr("Could not open serial port {}: {}".format(SERIAL_PORT, e))
        rospy.logwarn("Make sure the ESP32 is plugged in and ready.")

def parse_odometry_message(line):
    """
    Parses a string of odometry data from the ESP32.
    Expected format from ESP32: "Odom:X:0.500,Y:0.000,Th:0.123;\n"
    Returns (x, y, theta) or None on failure.
    """
    # Regular expression to find X, Y, and Theta values
    match = re.search(r"Odom:X:(-?\d+\.\d+),Y:(-?\d+\.\d+),Th:(-?\d+\.\d+);", line)
    if match:
        try:
            x = float(match.group(1))
            y = float(match.group(2))
            theta = float(match.group(3))
            return x, y, theta
        except ValueError:
            rospy.logwarn("Failed to convert parsed odometry data to float.")
    # rospy.logwarn(f"Received unparsable line: {line.strip()}")
    return None

def main():
    """Main execution function."""
    rospy.init_node('serial_reader', anonymous=True)

    # Set up serial connection
    setup_serial()

    # Create publishers and broadcasters
    odom_pub = rospy.Publisher(ODOM_TOPIC, Odometry, queue_size=50)
    tf_broadcaster = TransformBroadcaster()

    rate = rospy.Rate(50) # Publishing rate (Hz) for odometry

    if not ser or not ser.is_open:
        rospy.logerr("Serial port is not open. Odometry reader exiting.")
        return

    while not rospy.is_shutdown():
        try:
            # Read a full line from the serial port
            line = ser.readline().decode('utf-8')
            
            if line:
                odom_data = parse_odometry_message(line)
                
                if odom_data:
                    current_time = rospy.Time.now()
                    x, y, theta = odom_data

                    # --- Publish the TF (Transform) for Odometry ---
                    # The transform links the robot base to the starting odometry frame
                    q = tf.quaternion_from_euler(0, 0, theta) # Convert yaw (theta) to Quaternion
                    
                    t = TransformStamped()
                    t.header.stamp = current_time
                    t.header.frame_id = ODOM_FRAME
                    t.child_frame_id = BASE_FRAME
                    t.transform.translation.x = x
                    t.transform.translation.y = y
                    t.transform.translation.z = 0.0
                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]
                    
                    tf_broadcaster.sendTransform(t)

                    # --- Publish the Odometry Message ---
                    odom = Odometry()
                    odom.header.stamp = current_time
                    odom.header.frame_id = ODOM_FRAME
                    odom.child_frame_id = BASE_FRAME
                    
                    # Set the position
                    odom.pose.pose.position.x = x
                    odom.pose.pose.position.y = y
                    odom.pose.pose.position.z = 0.0
                    odom.pose.pose.orientation = Quaternion(*q)

                    # Velocity data (ESP32 should send this too, but for now we omit it)
                    # odom.twist.twist.linear.x = 0.0 
                    # odom.twist.twist.angular.z = 0.0 

                    odom_pub.publish(odom)

        except serial.SerialException as e:
            rospy.logerr("Serial Read Error: {}".format(e))
            time.sleep(1) # Wait before trying again
        except Exception as e:
            rospy.logerr("An unexpected error occurred: {}".format(e))
            
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
