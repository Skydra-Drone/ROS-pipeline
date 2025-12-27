#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range
import serial
import time
import math
import random

# --- CONFIG ---
LIDAR_PORT = None # None = Simulation
SIMULATION_MODE = (LIDAR_PORT is None)
MIN_RANGE = 0.1
MAX_RANGE = 12.0
FIELD_OF_VIEW = 0.04

def lidar_publisher():
    rospy.init_node('lidar_node', anonymous=True)
    range_pub = rospy.Publisher('/lidar/range', Range, queue_size=10)
    rate = rospy.Rate(100) # 100 Hz
    ser = None
    time_offset = 0.0

    if SIMULATION_MODE:
        rospy.loginfo("Running in SIMULATION mode (LiDAR)")
    else:
        rospy.loginfo(f"Connecting to LiDAR on {LIDAR_PORT}")

    while not rospy.is_shutdown():
        range_value = None

        if not SIMULATION_MODE:
            # (Simplified serial reading logic for brevity, assuming TFMini style)
            if ser is None or not ser.is_open:
                try:
                    ser = serial.Serial(LIDAR_PORT, 115200, timeout=1)
                except Exception:
                    time.sleep(1)
                    continue
            # Read logic would go here... for now let's focus on simulation fallback
            pass 
        else:
            # Simulate hovering
            base_altitude = 10.0
            range_value = base_altitude + 2.0 * math.sin(time_offset) + random.gauss(0, 0.05)
            time_offset += 0.01

        if range_value is not None:
            range_msg = Range()
            range_msg.header.stamp = rospy.Time.now()
            range_msg.header.frame_id = "lidar_frame"
            range_msg.radiation_type = Range.INFRARED
            range_msg.field_of_view = FIELD_OF_VIEW
            range_msg.min_range = MIN_RANGE
            range_msg.max_range = MAX_RANGE
            range_msg.range = max(MIN_RANGE, min(MAX_RANGE, range_value))
            range_pub.publish(range_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        lidar_publisher()
    except rospy.ROSInterruptException:
        pass
