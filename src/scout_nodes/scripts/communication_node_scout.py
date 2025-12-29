#!/usr/bin/env python3
import rospy
import socket
import json
import threading
import time
from pymavlink import mavutil
from std_msgs.msg import String
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix, BatteryState
from custom_msgs.msg import YoloDetectionArray, TargetCoordinatesArray

# --- CONFIG ---
BASE_STATION_IP = "192.168.1.100" 
BASE_PORT = 14550 # Standard MAVLink UDP port (QGC)

class ScoutCommunicationNode:
    def __init__(self):
        rospy.init_node('communication_node_scout', anonymous=True)
        
        # --- Publishers (Internal ROS) ---
        self.command_pub = rospy.Publisher('/mission/command', String, queue_size=10)
        
        # --- Subscribers (From ROS to Network) ---
        rospy.Subscriber('/mission/target_coordinates', TargetCoordinatesArray, self.target_callback)
        rospy.Subscriber('/yolo/detections', YoloDetectionArray, self.detection_callback)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_callback)
        
        # --- MAVLink Setup ---
        # Connection string: udpout points TO the GCS
        connection_string = f"udpout:{BASE_STATION_IP}:{BASE_PORT}"
        rospy.loginfo(f"Connecting to GCS via MAVLink at {connection_string}")
        
        self.master = mavutil.mavlink_connection(connection_string, source_system=1, source_component=1)
        
        # Sending Heartbeat Thread
        self.running = True
        self.heartbeat_thread = threading.Thread(target=self.send_heartbeat_loop)
        self.heartbeat_thread.daemon = True
        self.heartbeat_thread.start()
        
        rospy.loginfo("Scout Communication Node Started (MAVLink)")

    def send_heartbeat_loop(self):
        while self.running and not rospy.is_shutdown():
            # Send heartbeat (MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC)
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_QUADROTOR,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                0, 0, 0)
            time.sleep(1)

    def target_callback(self, msg):
        # Send unique targets as Named Value Floats
        # We allow up to 9 targets: TGT0_LAT, TGT0_LON, etc.
        count = len(msg.diff_targets)
        
        # Send Count
        self.master.mav.named_value_float_send(
            time.time_ns() // 1000000, # Boot time (ms)
            "TGT_COUNT".encode('utf-8'), 
            float(count)
        )
        
        for i, t in enumerate(msg.diff_targets):
            if i > 9: break # Limit to 10 targets for cleanliness
            
            # Send Latitude
            name_lat = f"TGT{i}_LAT".encode('utf-8')
            self.master.mav.named_value_float_send(
                time.time_ns() // 1000000,
                name_lat,
                t.latitude
            )
            
            # Send Longitude
            name_lon = f"TGT{i}_LON".encode('utf-8')
            self.master.mav.named_value_float_send(
                time.time_ns() // 1000000,
                name_lon,
                t.longitude
            )

    def detection_callback(self, msg):
        # Send simple alert count if something is detected
        if msg.detections:
            self.master.mav.named_value_int_send(
                time.time_ns() // 1000000,
                "DET_COUNT".encode('utf-8'),
                len(msg.detections)
            )

    def gps_callback(self, msg):
        # We assume MAVROS is already sending GLOBAL_POSITION_INT to GCS.
        # But if we need to send raw NavSatFix from custom GPS:
        # self.master.mav.gps_raw_int_send(...)
        pass

    def run(self):
        # We can also listen for incoming commands here if needed
        rospy.spin()

if __name__ == '__main__':
    node = ScoutCommunicationNode()
    node.run()
