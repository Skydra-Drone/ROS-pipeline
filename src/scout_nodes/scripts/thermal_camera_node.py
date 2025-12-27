#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time

# --- CONFIG ---
CAMERA_SOURCE = None  # None = Simulation
SIMULATION_MODE = (CAMERA_SOURCE is None)
TEMP_MIN = 15.0
TEMP_MAX = 40.0

def thermal_camera_publisher():
    rospy.init_node('thermal_camera_node', anonymous=True)
    thermal_pub = rospy.Publisher('/thermal/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(10) # 10 Hz
    cap = None

    if not SIMULATION_MODE:
        rospy.loginfo(f"Connecting to thermal camera: {CAMERA_SOURCE}")
    else:
        rospy.loginfo("Running in SIMULATION mode (thermal)")

    while not rospy.is_shutdown():
        thermal_image = None

        if not SIMULATION_MODE:
            if cap is None or not cap.isOpened():
                cap = cv2.VideoCapture(CAMERA_SOURCE)
                if not cap.isOpened():
                    time.sleep(5)
                    continue
            ret, frame = cap.read()
            if not ret:
                cap.release()
                cap = None
                continue
            # Simple assumption: input is already mapped or we apply colormap
            if len(frame.shape) == 2:
                thermal_image = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            else:
                thermal_image = frame
        else:
            thermal_image = generate_simulated_thermal()

        if thermal_image is not None:
            try:
                ros_image = bridge.cv2_to_imgmsg(thermal_image, "bgr8")
                ros_image.header.stamp = rospy.Time.now()
                ros_image.header.frame_id = "thermal_camera_frame"
                thermal_pub.publish(ros_image)
            except CvBridgeError as e:
                rospy.logerr(f"CvBridge Error: {e}")

        rate.sleep()

def generate_simulated_thermal():
    width, height = 320, 240
    thermal_data = np.random.normal(25, 3, (height, width))
    
    # Random hot spots
    num_hotspots = np.random.randint(1, 4)
    for _ in range(num_hotspots):
        x = np.random.randint(50, width - 50)
        y = np.random.randint(50, height - 50)
        Y, X = np.ogrid[:height, :width]
        dist = np.sqrt((X - x)**2 + (Y - y)**2)
        hotspot = 10 * np.exp(-dist**2 / (2 * 20**2))
        thermal_data += hotspot

    thermal_normalized = np.clip(thermal_data, TEMP_MIN, TEMP_MAX)
    thermal_normalized = ((thermal_normalized - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * 255).astype(np.uint8)
    return cv2.applyColorMap(thermal_normalized, cv2.COLORMAP_JET)

if __name__ == '__main__':
    try:
        thermal_camera_publisher()
    except rospy.ROSInterruptException:
        pass
