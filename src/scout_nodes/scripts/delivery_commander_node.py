#!/usr/bin/env python3
import rospy
import time
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
             
             # Wait for arrival (mock sleep for now since we lack telemetry check loop)
             time.sleep(15) 
             
             # 5. Drop Payload (Servo)
             rospy.loginfo("Dropping Payload...")
             # self.master.mav.command_long_send(...)
             time.sleep(2)
        
        rospy.loginfo("All deliveries complete. Returning to Launch.")
        
        # logic to wait for arrival would go here...
        
        # 5. Drop Payload (Servo)
        # self.master.mav.command_long_send(...)
        
        # 6. Return to Launch
        # self.master.mav.command_long_send(..., mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, ...)
        
        self.state = "IDLE"

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = DeliveryCommanderNode()
    node.run()
