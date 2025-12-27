#!/usr/bin/env python3
import rospy
import message_filters
import numpy as np
from scipy.spatial.transform import Rotation
from pyproj import Proj, Transformer

# Import all the necessary message types
from custom_msgs.msg import YoloDetectionArray
from sensor_msgs.msg import NavSatFix, Imu, Range
from geographic_msgs.msg import GeoPoint

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
        # We will publish the final calculated GPS coordinate here
        self.target_pub = rospy.Publisher('/mission/target_coordinates', GeoPoint, queue_size=10)

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

        # For simplicity, we'll just process the first detection in the list
        detection = detections_msg.detections[0]

        rospy.loginfo("--- Synchronized Data Received ---")

        # --- Step 1: Extract and Prepare Data ---
        # Get the center of the bounding box
        pixel_x = (detection.x1 + detection.x2) / 2.0
        pixel_y = (detection.y1 + detection.y2) / 2.0

        # Get the drone's GPS position
        drone_lat = gps_msg.latitude
        drone_lon = gps_msg.longitude

        # Get the drone's height above ground from LiDAR/Range sensor
        # The 'range' field holds the distance in meters.
        height_above_ground = lidar_msg.range
        if height_above_ground <= 0.1: # Sanity check for bad readings
            rospy.logwarn("Invalid height from LiDAR, skipping calculation.")
            return

        # Get the drone's orientation from the IMU
        orientation_q = imu_msg.orientation
        # Convert the quaternion to a rotation object
        rotation = Rotation.from_quat([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        
        # --- Step 2: Camera Geometry Calculation (Vector Math) ---
        # Convert pixel coordinates to a 3D vector in the camera's coordinate frame.
        # The camera frame is typically: X right, Y down, Z forward.
        vector_camera_frame = np.array([
            (pixel_x - self.cx) / self.fx,
            (pixel_y - self.cy) / self.fy,
            1.0  # The Z component is 1 because this is a direction vector
        ])

        # Normalize the vector to have a length of 1 (unit vector)
        vector_camera_frame = vector_camera_frame / np.linalg.norm(vector_camera_frame)

        # --- Step 3: Coordinate Frame Transformations ---
        # The drone's body frame (FLU: Forward, Left, Up) needs to be aligned with the
        # camera frame (RDF: Right, Down, Forward). This often requires a static rotation.
        # Assuming camera points straight forward, this can sometimes be simplified,
        # but a full implementation would use a static transform here.
        # For now, we'll assume a simple forward-facing camera.

        # Rotate the vector from the camera frame to the drone's body frame (FLU).
        # Then rotate from the drone's body frame to the world frame (NED: North, East, Down).
        # The 'rotation' object from the IMU directly gives us the body-to-world rotation.
        vector_world_frame = rotation.apply(vector_camera_frame)

        # --- Step 4: Ground Intersection Calculation ---
        # Check if the vector is pointing downwards. If not, it can't intersect the ground.
        # In the NED frame, the 'Down' component is the 3rd element (index 2).
        if vector_world_frame[2] <= 0:
            rospy.logwarn("Detection is not pointing towards the ground.")
            return

        # Calculate the scaling factor to project the vector onto the ground plane.
        # This is based on similar triangles.
        scale_factor = height_above_ground / vector_world_frame[2]

        # Calculate the offset in meters in the North and East directions.
        north_offset_m = vector_world_frame[0] * scale_factor
        east_offset_m = vector_world_frame[1] * scale_factor

        rospy.loginfo(f"Calculated offset: {north_offset_m:.2f}m North, {east_offset_m:.2f}m East")

        # --- Step 5: Final GPS Coordinate Calculation ---
        # Define a local projection centered on the drone's current location
        local_projection = f"+proj=tmerc +lat_0={drone_lat} +lon_0={drone_lon} +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
        
        # Create a transformer to convert from our local (East, North) to global (Lat, Lon)
        transformer = Transformer.from_crs(local_projection, self.wgs84, always_xy=True)

        # Transform the (East, North) offset to a new (Lon, Lat) coordinate
        target_lon, target_lat = transformer.transform(east_offset_m, north_offset_m)

        # --- Publish the Result ---
        target_point = GeoPoint()
        target_point.latitude = target_lat
        target_point.longitude = target_lon
        target_point.altitude = 0 # Altitude on the ground

        self.target_pub.publish(target_point)
        rospy.loginfo(f"Published Target GPS: Lat {target_lat:.6f}, Lon {target_lon:.6f}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = GeolocationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
