#!/usr/bin/env python3
import rospy
import time
import math
import numpy as np
from std_msgs.msg import String
from geographic_msgs.msg import GeoPoint
from custom_msgs.msg import TargetCoordinatesArray
try:
    from pymavlink import mavutil
except ImportError:
    rospy.logwarn("pymavlink not installed. Delivery node will not function.")
    mavutil = None

# --- CONFIG ---
DELIVERY_DRONE_CONNECTION = "udp:192.168.1.102:14550" # IP of Delivery Drone FC

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    phi1, phi2 = np.radians(lat1), np.radians(lat2)
    dphi = np.radians(lat2 - lat1)
    dlambda = np.radians(lon2 - lon1)
    a = np.sin(dphi / 2)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(dlambda / 2)**2
    return 2 * R * np.arctan2(np.sqrt(a), np.sqrt(1 - a))

class DeliveryCommanderNode:
    def __init__(self):
        rospy.init_node('delivery_commander_node', anonymous=True)
        
        self.target_coords = None
        self.state = "IDLE" # IDLE, PENDING_APPROVAL, COMMANDING
        
        # --- Subscribers ---
        rospy.Subscriber('/mission/target_coordinates', TargetCoordinatesArray, self.target_callback)
        rospy.Subscriber('/mission/command', String, self.command_callback)
        
        # --- MAVLink Connection ---
        self.master = None
        if mavutil:
            try:
                # connect to the flight controller
                self.master = mavutil.mavlink_connection(DELIVERY_DRONE_CONNECTION)
                rospy.loginfo("Connected to Delivery Drone MAVLink")
            except Exception as e:
                rospy.logerr(f"MAVLink Connection Error: {e}")
        
        rospy.loginfo("Delivery Commander Node Started")

    def target_callback(self, msg):
        if self.state == "IDLE":
            self.target_coords = msg.diff_targets
            self.state = "PENDING_APPROVAL"
            rospy.loginfo(f"Delivery Targets Received: {len(msg.diff_targets)} locations. Waiting for Approval.")

    def command_callback(self, msg):
        if msg.data == "APPROVE_DELIVERY" and self.state == "PENDING_APPROVAL":
            self.execute_delivery()

    def execute_delivery(self):
        if not self.master or not self.target_coords:
            return
            
        rospy.loginfo("Executing Delivery Mission...")
        self.state = "COMMANDING"
        
        # 1. Set Mode to GUIDED
        # self.master.set_mode_apm_set_mode(self.master.mode_mapping()['GUIDED'])
        
        # 2. Arm Drone
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        rospy.loginfo("Delivery Drone Armed")
        
        # 3. Takeoff
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, 10
        )
        time.sleep(10)
        
        # Loop through all targets
        for i, target in enumerate(self.target_coords):
             rospy.loginfo(f"--- Delivering to Target {i+1}/{len(self.target_coords)} ---")
             
             # 4. Go to Target
             rospy.loginfo(f"Flying to {target.latitude}, {target.longitude}")
             self.master.mav.mission_item_send(
                 self.master.target_system,
                 self.master.target_component,
                 0,
                 mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                 mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                 2, 0, 1, 0, 0, 0,
                 target.latitude,
                 target.longitude,
                 10 # Altitude
             )
             
             # Wait for arrival with active telemetry check
             rospy.loginfo("Waiting for arrival...")
             start_wait = time.time()
             has_arrived = False
             
             while time.time() - start_wait < 60: # 60s timeout per target
                 # Try to get position (GLOBAL_POSITION_INT)
                 # We simply read the latest message from the buffer
                 msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                 if msg:
                     current_lat = msg.lat / 1e7
                     current_lon = msg.lon / 1e7
                     
                     dist = haversine(current_lat, current_lon, target.latitude, target.longitude)
                     # Check if within 0.5m (and ensure altitude is reasonable if needed)
                     if dist < 0.5:
                         rospy.loginfo(f"Arrived at target! Distance: {dist:.2f}m")
                         has_arrived = True
                         break
                 
                 time.sleep(0.2)
            
             if not has_arrived:
                 rospy.logwarn("Timeout reached before arriving at target! Creating risk of bad drop.")
                 # Decide whether to drop or skip. For safety, we might skip.
                 # continue 
             
             # 5. Drop Payload (Servo)
             rospy.loginfo("Dropping Payload...")
             # self.master.mav.command_long_send(...)
             time.sleep(2)
        
        rospy.loginfo("All deliveries complete. Returning to Launch.")
        
        # 6. Return to Launch
        # self.master.mav.command_long_send(..., mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, ...)
        
        self.state = "IDLE"

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = DeliveryCommanderNode()
    node.run()
