#!/usr/bin/env python3
import rospy
import time
from std_msgs.msg import String
from geographic_msgs.msg import GeoPoint
from mavros_msgs.msg import State, GlobalPositionTarget, CommandBool, SetMode
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Twist

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

    def start_mission(self):
        if not self.mavros_state.armed:
            self.arming_client(True)
        
        self.set_mode("GUIDED")
        time.sleep(1)
        
        # Simple takeoff logic (Mock: Send a waypoint 5m up)
        # Real logic would use /mavros/cmd/takeoff
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 5
        
        # Send a few times to ensure it's received
        for _ in range(10):
            self.setpoint_pub.publish(pose)
            time.sleep(0.1)

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
