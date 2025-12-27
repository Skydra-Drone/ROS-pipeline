#!/usr/bin/env python3
import rospy
import socket
import json
import threading
from std_msgs.msg import String
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix, BatteryState
from custom_msgs.msg import YoloDetectionArray

# --- CONFIG ---
BASE_STATION_IP = "192.168.1.100" # CHANGE THIS to Base Station IP
BASE_PORT = 5000
DRONE_PORT = 5001

class ScoutCommunicationNode:
    def __init__(self):
        rospy.init_node('communication_node_scout', anonymous=True)
        
        # --- Publishers (Internal ROS) ---
        self.command_pub = rospy.Publisher('/mission/command', String, queue_size=10)
        
        # --- Subscribers (From ROS to Network) ---
        rospy.Subscriber('/mission/target_coordinates', GeoPoint, self.target_callback)
        rospy.Subscriber('/yolo/detections', YoloDetectionArray, self.detection_callback)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        # Add more telemetry as needed
        
        # --- Network Setup ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP for telemetry
        
        # Start command listener thread
        self.running = True
        self.listen_thread = threading.Thread(target=self.listen_for_commands)
        self.listen_thread.daemon = True
        self.listen_thread.start()
        
        rospy.loginfo("Scout Communication Node Started")

    def send_packet(self, data_type, content):
        packet = {
            "type": data_type,
            "content": content,
            "timestamp": rospy.get_time()
        }
        try:
            msg = json.dumps(packet).encode('utf-8')
            self.sock.sendto(msg, (BASE_STATION_IP, BASE_PORT))
        except Exception as e:
            rospy.logerr_throttle(5, f"Network Error: {e}")

    def target_callback(self, msg):
        data = {"lat": msg.latitude, "lon": msg.longitude, "alt": msg.altitude}
        self.send_packet("TARGET_FOUND", data)

    def detection_callback(self, msg):
        # We only send simple detection alerts to save bandwidth
        if msg.detections:
            self.send_packet("DETECTION_ALERT", {"count": len(msg.detections)})

    def gps_callback(self, msg):
        data = {"lat": msg.latitude, "lon": msg.longitude, "alt": msg.altitude}
        self.send_packet("TELEMETRY", data)

    def listen_for_commands(self):
        # Listen for incoming TCP/UDP commands from Base Station
        # Using UDP for simplicity here, but TCP is better for reliable commands
        command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        command_sock.bind(("0.0.0.0", DRONE_PORT))
        
        while self.running and not rospy.is_shutdown():
            try:
                data, addr = command_sock.recvfrom(1024)
                command_str = data.decode('utf-8')
                rospy.loginfo(f"Received Command: {command_str}")
                
                # Publish to internal mission topic
                self.command_pub.publish(command_str)
            except Exception as e:
                rospy.logerr(f"Listener Error: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ScoutCommunicationNode()
    node.run()
