#!/usr/bin/env python3
import rospy
import cv2
import torch
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Import our custom messages
# Ensure custom_msgs is built and sourced!
from custom_msgs.msg import YoloDetection, YoloDetectionArray

class YoloDetectorNode:
    def __init__(self):
        rospy.init_node('yolo_detector_node', anonymous=True)

        # --- Model and Device Setup ---
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        rospy.loginfo(f"Using device: {self.device}")
        
        # Construct path to model file in the scripts directory (or wherever the node is running)
        # This ensures we look for 'yolov8n.pt' locally first.
        # If not found, Ultralytics will download it to the current working directory.
        model_name = "yolov8n.pt"
        
        # You can place the model explicitly in the package to avoid re-downloading
        # self.model = YOLO(os.path.join(os.path.dirname(__file__), model_name)) 
        
        self.model = YOLO(model_name) 
        
        # Explicit transfer to device, though Ultralytics usually handles auto-selection
        self.model.to(self.device)

        rospy.loginfo(f"YOLO model loaded successfully on {self.device}.")

        self.bridge = CvBridge()
        
        self.detection_pub = rospy.Publisher('/yolo/detections', YoloDetectionArray, queue_size=10)
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback, queue_size=1, buff_size=2**24)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # Inference
        # classes=[0] for Person
        results = self.model.predict(
            cv_image,
            device=self.device,
            verbose=False,
            classes=[0], 
            conf=0.5
        )

        detection_array_msg = YoloDetectionArray()
        detection_array_msg.header = msg.header

        if results[0].boxes is not None:
            for box in results[0].boxes:
                detection_msg = YoloDetection()
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                detection_msg.x1 = int(x1)
                detection_msg.y1 = int(y1)
                detection_msg.x2 = int(x2)
                detection_msg.y2 = int(y2)
                detection_msg.confidence = float(box.conf[0])
                detection_msg.class_name = "PERSON"
                detection_array_msg.detections.append(detection_msg)

        self.detection_pub.publish(detection_array_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = YoloDetectorNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
