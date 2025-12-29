#!/usr/bin/env python3
import rospy
import message_filters
import numpy as np
from scipy.spatial.transform import Rotation
from pyproj import Proj, Transformer

# Import all the necessary message types
from custom_msgs.msg import YoloDetectionArray, TargetCoordinatesArray
from sensor_msgs.msg import NavSatFix, Imu, Range
from geographic_msgs.msg import GeoPoint

def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points 
    on the earth (specified in decimal degrees)
    """
    R = 6371000  # Radius of earth in meters
    phi1, phi2 = np.radians(lat1), np.radians(lat2)
    dphi = np.radians(lat2 - lat1)
    dlambda = np.radians(lon2 - lon1)
    a = np.sin(dphi / 2)**2 + np.cos(phi1) * np.cos(phi2) * np.sin(dlambda / 2)**2
    return 2 * R * np.arctan2(np.sqrt(a), np.sqrt(1 - a))

class GeolocationNode:
    def __init__(self):
        rospy.init_node('geolocation_node', anonymous=True)
        rospy.loginfo("Geolocation Node Started")

        # --- Camera Intrinsic Parameters ---
        # IMPORTANT: You MUST replace these with the actual values for your camera.
        # You can find these by running a camera calibration process.
        # For now, these are plausible estimates for a standard USB webcam.
        self.fx = 600.0  # Focal length in x
        self.fy = 600.0  # Focal length in y
        self.cx = 320.0  # Principal point x (image width / 2)
        self.cy = 240.0  # Principal point y (image height / 2)

        # --- ROS Publishers ---
        # We will publish the sorted list of target coordinates here
        self.target_pub = rospy.Publisher('/mission/target_coordinates', TargetCoordinatesArray, queue_size=10)

        # --- Synchronized Subscribers ---
        # Create subscribers for each topic
        detection_sub = message_filters.Subscriber('/yolo/detections', YoloDetectionArray)
        gps_sub = message_filters.Subscriber('/mavros/global_position/global', NavSatFix)
        imu_sub = message_filters.Subscriber('/mavros/imu/data', Imu)
        lidar_sub = message_filters.Subscriber('/mavros/global_position/rel_alt', Range) # Using rel_alt for now as a stand-in for LiDAR

        # The ApproximateTimeSynchronizer is the key to this node.
        # It takes a list of subscribers and a queue size, and a time slop (in seconds).
        # It will only call the callback when it has received a message from each topic
        # that is within the 'slop' time of the others.
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [detection_sub, gps_sub, imu_sub, lidar_sub],
            queue_size=10,
            slop=0.1,  # Allow 100ms difference between message timestamps
            allow_headerless=False # Ensure all messages have timestamps
        )

        # Register the callback function
        self.ts.registerCallback(self.synchronized_callback)

        # --- Coordinate System Transformer ---
        # Used to convert from a local ENU (East, North, Up) frame to global Lat/Lon
        self.wgs84 = "EPSG:4326"  # Standard Lat/Lon

    def synchronized_callback(self, detections_msg, gps_msg, imu_msg, lidar_msg):
        """
        This callback is triggered only when a synchronized set of messages is received.
        """
        # Step 0: Check if there are any detections
        if not detections_msg.detections:
            return # No detections, do nothing

        # --- Process ALL detections ---
        calculated_targets = []

        # Optimization: Pre-calculate common values
        drone_quat = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        drone_rot = Rotation.from_quat(drone_quat)
        
        # Local projection setup
        local_projection = f"+proj=tmerc +lat_0={drone_lat} +lon_0={drone_lon} +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
        transformer = Transformer.from_crs(local_projection, self.wgs84, always_xy=True)

        for detection in detections_msg.detections:
            # Step 1: Pixel Coordinates
            px = (detection.x1 + detection.x2) / 2.0
            py = (detection.y1 + detection.y2) / 2.0

            # Step 2: Camera Vector
            vec_cam = np.array([(px - self.cx) / self.fx, (py - self.cy) / self.fy, 1.0])
            vec_cam = vec_cam / np.linalg.norm(vec_cam)

            # Step 3: Rotate to World
            vec_world = drone_rot.apply(vec_cam)

            # Step 4: Ground Intersect
            if vec_world[2] <= 0:
                continue # Pointing up

            scale = height_above_ground / vec_world[2]
            north_m = vec_world[0] * scale
            east_m = vec_world[1] * scale

            # Step 5: Convert to GPS
            t_lon, t_lat = transformer.transform(east_m, north_m)
            
            # Calculate distance for sorting
            dist = haversine(drone_lat, drone_lon, t_lat, t_lon)
            
            # Create GeoPoint
            p = GeoPoint()
            p.latitude = t_lat
            p.longitude = t_lon
            p.altitude = 0.0
            
            calculated_targets.append({'point': p, 'dist': dist})

        if not calculated_targets:
            return

        # Sort by distance (ascending)
        calculated_targets.sort(key=lambda x: x['dist'])

        # Create output message
        final_msg = TargetCoordinatesArray()
        final_msg.diff_targets = [x['point'] for x in calculated_targets]

        self.target_pub.publish(final_msg)
        rospy.loginfo(f"Published {len(final_msg.diff_targets)} targets. Closest is {calculated_targets[0]['dist']:.2f}m away.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = GeolocationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
