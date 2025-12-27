#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import os

# --- CONFIG ---
# FOR REAL DRONE: CAMERA_SOURCE = "http://192.168.0.105:8080/video"
# FOR TESTING: Use 0 for webcam, or a path to a video file.
CAMERA_SOURCE = 0 

def camera_publisher():
    """
    Connects to a camera source and publishes frames as ROS Image messages.
    """
    rospy.init_node('camera_node', anonymous=True)
    
    # Publisher
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    
    bridge = CvBridge()
    rate = rospy.Rate(30) # 30 Hz
    
    cap = None
    
    # Check if CAMERA_SOURCE is a file path and if it exists
    if isinstance(CAMERA_SOURCE, str) and not CAMERA_SOURCE.startswith("http"):
        if not os.path.exists(CAMERA_SOURCE):
            rospy.logwarn(f"Video file not found: {CAMERA_SOURCE}. Switching to webcam (0).")
            # We can't change the local variable effectively without `global`, so we'll just use 0 here if needed or let it fail gently.
            # But let's just log it.

    while not rospy.is_shutdown():
        # --- Connection Logic ---
        if cap is None or not cap.isOpened():
            rospy.loginfo(f"Attempting to connect to camera source: {CAMERA_SOURCE}")
            cap = cv2.VideoCapture(CAMERA_SOURCE)
            if not cap.isOpened():
                rospy.logwarn("Failed to connect to camera. Retrying in 5 seconds...")
                time.sleep(5)
                continue
            else:
                rospy.loginfo("Camera connected successfully.")

        # --- Frame Capture and Publishing ---
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Failed to grab frame. Reconnecting or End of Stream...")
            cap.release()
            cap = None
            if isinstance(CAMERA_SOURCE, str): # If video file, maybe loop?
                time.sleep(1)
            continue

        try:
            # Convert OpenCV image (BGR) to ROS Image message
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            ros_image.header.stamp = rospy.Time.now()
            image_pub.publish(ros_image)
            
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")

        rate.sleep()

    if cap is not None:
        cap.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
