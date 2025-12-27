#!/usr/bin/env python3
import rospy
import socket
import json
import threading
from std_msgs.msg import String
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix

# --- CONFIG ---
SCOUT_IP = "192.168.1.101" # CHANGE THIS to Scout Drone IP
DRONE_PORT = 5001
BASE_PORT = 5000

class BaseCommunicationNode:
    def __init__(self):
        rospy.init_node('communication_node_base', anonymous=True)
        
        # --- Publishers (To GUI) ---
        self.scout_gps_pub = rospy.Publisher('/scout/telemetry/gps', NavSatFix, queue_size=10)
        self.target_pub = rospy.Publisher('/scout/mission/target_coordinates', GeoPoint, queue_size=10)
        self.alert_pub = rospy.Publisher('/scout/alerts', String, queue_size=10)
        
        # --- Subscribers (From GUI) ---
        rospy.Subscriber('/ui/mission_command', String, self.ui_command_callback)
        
        # --- Network Setup ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", BASE_PORT))
        
        self.running = True
        self.listen_thread = threading.Thread(target=self.listen_for_data)
        self.listen_thread.daemon = True
        self.listen_thread.start()
        
        rospy.loginfo("Base Station Communication Node Started")

    def ui_command_callback(self, msg):
        # Forward UI commands to the Scout Drone
        try:
            self.sock.sendto(msg.data.encode('utf-8'), (SCOUT_IP, DRONE_PORT))
            rospy.loginfo(f"Sent to Scout: {msg.data}")
        except Exception as e:
            rospy.logerr(f"Send Error: {e}")

    def listen_for_data(self):
        while self.running and not rospy.is_shutdown():
            try:
                data, addr = self.sock.recvfrom(4096)
                packet = json.loads(data.decode('utf-8'))
                
                ptype = packet.get("type")
                content = packet.get("content")
                
                if ptype == "TELEMETRY":
                    msg = NavSatFix()
                    msg.latitude = content["lat"]
                    msg.longitude = content["lon"]
                    msg.altitude = content["alt"]
                    self.scout_gps_pub.publish(msg)
                    
                elif ptype == "TARGET_FOUND":
                    msg = GeoPoint()
                    msg.latitude = content["lat"]
                    msg.longitude = content["lon"]
                    msg.altitude = content["alt"]
                    self.target_pub.publish(msg)
                    rospy.loginfo(f"TARGET RECEIVED: {content}")
                    
                elif ptype == "DETECTION_ALERT":
                    self.alert_pub.publish(f"Detected {content['count']} objects")
                    
            except Exception as e:
                rospy.logerr_throttle(5, f"Receive Error: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = BaseCommunicationNode()
    node.run()
