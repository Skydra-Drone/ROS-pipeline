#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix, Imu, Range
import time

def fake_drone_publisher():
    rospy.init_node('fake_drone_publisher', anonymous=True)
    
    gps_pub = rospy.Publisher('/mavros/global_position/global', NavSatFix, queue_size=10)
    imu_pub = rospy.Publisher('/mavros/imu/data', Imu, queue_size=10)
    # Note: geolocation_node listens to /mavros/global_position/rel_alt which is a Float64 usually,
    # OR we decided to use Range from /lidar/range in the plan.
    # The existing geolocation_node.py listens to /mavros/global_position/rel_alt as Range which is WEIRD.
    # Standard rel_alt is std_msgs/Float64. MAVROS rel_alt is usually float.
    # Let's align with the existing geolocation_node code which expects Range on /mavros/global_position/rel_alt (Step 24).
    range_pub = rospy.Publisher('/mavros/global_position/rel_alt', Range, queue_size=10)
    
    rate = rospy.Rate(10)

    rospy.loginfo("Publishing fake drone data...")

    while not rospy.is_shutdown():
        header = rospy.Header()
        header.stamp = rospy.Time.now()

        # GPS
        gps_msg = NavSatFix()
        gps_msg.header = header
        gps_msg.latitude = 40.7128
        gps_msg.longitude = -74.0060
        gps_msg.altitude = 100

        # IMU (Level)
        imu_msg = Imu()
        imu_msg.header = header
        imu_msg.orientation.x = 0
        imu_msg.orientation.y = 0
        imu_msg.orientation.z = 0
        imu_msg.orientation.w = 1

        # Range (20m up)
        range_msg = Range()
        range_msg.header = header
        range_msg.range = 20.0
        range_msg.min_range = 0.1
        range_msg.max_range = 50.0

        gps_pub.publish(gps_msg)
        imu_pub.publish(imu_msg)
        range_pub.publish(range_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        fake_drone_publisher()
    except rospy.ROSInterruptException:
        pass
