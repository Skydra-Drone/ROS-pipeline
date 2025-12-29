#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import String
from geographic_msgs.msg import GeoPoint
from mavros_msgs.msg import State, GlobalPositionTarget, CommandBool, SetMode
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Twist
import xml.etree.ElementTree as ET
from shapely.geometry import Point, Polygon
import numpy as np
import os

class ScoutDecisionNode:
    def __init__(self):
        rospy.init_node('decision_node_scout', anonymous=True)
        
        # State
        self.state = "IDLE" # IDLE, SEARCHING, TRACKING, RTL
        self.current_pose = None
        self.mavros_state = State()
        
        # --- Subscribers ---
        rospy.Subscriber('/mission/command', String, self.command_callback)
        rospy.Subscriber('/mission/target_coordinates', GeoPoint, self.target_callback)
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        
        # --- Publishers ---
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        
        # --- Services ---
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        rospy.loginfo("Scout Decision Node Started")

    def state_callback(self, msg):
        self.mavros_state = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def command_callback(self, msg):
        cmd = msg.data
        rospy.loginfo(f"Logic Received Command: {cmd}")
        
        if cmd == "START_SEARCH":
            self.state = "SEARCHING"
            self.start_mission()
        elif cmd == "RETURN_HOME":
            self.state = "RTL"
            self.set_mode("AUTO.RTL")
        elif cmd == "LAND":
            self.set_mode("AUTO.LAND")

    def target_callback(self, msg):
        if self.state == "SEARCHING":
            rospy.loginfo("Target Found! Switching to TRACKING/LOITER mode.")
            self.state = "TRACKING"
            # In a real scenario, we might send a loiter command slightly above the target
            # For now, let's just hold position (LOITER)
            self.set_mode("AUTO.LOITER")

    def parse_kml_boundary(self, kml_path):
        """Extracts polygon coordinates from a KML file."""
        tree = ET.parse(kml_path)
        root = tree.getroot()
        
        # Namespace handling for KML
        ns = {'kml': 'http://www.opengis.net/kml/2.2'}
        
        coords = []
        # Find all coordinates in Polygon/outerBoundaryIs/LinearRing/coordinates
        # This is a basic parser; might need adjustment for complex KMLs
        for coordinates in root.findall('.//kml:coordinates', ns):
            text = coordinates.text.strip()
            for line in text.split():
                parts = line.split(',')
                if len(parts) >= 2:
                    lon, lat = float(parts[0]), float(parts[1])
                    coords.append((lat, lon)) # Store as Lat, Lon
            break # Assume single polygon for now
            
        return coords

    def generate_scan_path(self, polygon_coords, spacing_m=10):
        """Generates a lawnmower path inside the polygon."""
        if not polygon_coords:
            return []
            
        poly = Polygon([(lat, lon) for lat, lon in polygon_coords])
        min_lat, min_lon, max_lat, max_lon = poly.bounds
        
        waypoints = []
        
        # Conversion factors (approximate)
        lat_deg_len = 111000.0
        
        current_lat = min_lat
        scan_west_east = True
        
        while current_lat <= max_lat:
            # Create a horizontal line at current_lat
            # We iterate longitude in small steps to find entry/exit points
            # A more robust way is using line intersection, but grid sampling is easier for now
            
            # Simple Grid Sampling method:
            # Check points every 'spacing' along the latitude line
            row_points = []
            current_lon = min_lon
            lon_step = spacing_m / (lat_deg_len * np.cos(np.radians(current_lat)))
            
            while current_lon <= max_lon:
                p = Point(current_lat, current_lon)
                if poly.contains(p):
                    row_points.append((current_lat, current_lon))
                current_lon += lon_step
            
            if row_points:
                if not scan_west_east:
                    row_points.reverse()
                
                waypoints.extend(row_points)
                scan_west_east = not scan_west_east
            
            # Move North
            current_lat += spacing_m / lat_deg_len
            
        rospy.loginfo(f"Generated {len(waypoints)} waypoints for scan.")
        return waypoints

    def start_mission(self):
        if not self.mavros_state.armed:
            self.arming_client(True)
        
        self.set_mode("GUIDED")
        time.sleep(1)
        
        self.set_mode("GUIDED")
        time.sleep(1)
        
        # Check for KML file
        kml_path = rospy.get_param("~kml_file", "/home/ashish/catkin_ws/src/scout_nodes/config/search_area.kml")
        
        mission_waypoints = []
        
        if os.path.exists(kml_path):
            rospy.loginfo(f"Loading search area from: {kml_path}")
            try:
                boundary = self.parse_kml_boundary(kml_path)
                # Generate path with 20m spacing (adjust based on FOV)
                mission_waypoints = self.generate_scan_path(boundary, spacing_m=20)
            except Exception as e:
                rospy.logerr(f"Failed to process KML: {e}")
        else:
            rospy.logwarn("KML file not found, using default takeoff.")
            # Default behavior (Takeoff and hold)
            mission_waypoints = [(self.current_pose.pose.position.x, self.current_pose.pose.position.y)] # Just dummy
            
            # Simple takeoff logic (Mock: Send a waypoint 5m up)
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = 0
            pose.pose.position.y = 0
            pose.pose.position.z = 20 # Safety altitude
            
            for _ in range(10):
                self.setpoint_pub.publish(pose)
                time.sleep(0.1)
            return

        # Execute Generated Mission
        rospy.loginfo("Starting Autonomous Scan...")
        for i, (lat, lon) in enumerate(mission_waypoints):
            # Send waypoint (Ideally convert Lat/Lon to Local ENU if using setpoint_position/local)
            # OR use mavros/setpoint_position/global (NavSatFix)
            # For this example, assuming we have a converter or using a Global Setpoint topic.
            # ERROR: self.setpoint_pub expects PoseStamped (Local ENU). 
            # We need to convert Lat/Lon to Local ENU or use Global Setpoint.
            # Simplified: Logging the waypoint target for now, assuming MAVROS global usage in real impl.
            
            rospy.loginfo(f"Going to Waypoint {i+1}: {lat}, {lon}")
            
            # Note: To actually fly this, we need 'geographic_lib' or MAVROS global setpoint.
            # self.global_pub.publish(...) 
            
            time.sleep(2) # Mock flight time between points

    def set_mode(self, mode):
        try:
            self.set_mode_client(0, mode)
        except rospy.ServiceException as e:
            rospy.logerr(f"Mode switch failed: {e}")

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Main Control Loop
            # If in SEARCHING mode, we would update waypoints here
            # For now, just maintain connection
            rate.sleep()

if __name__ == '__main__':
    node = ScoutDecisionNode()
    node.run()
