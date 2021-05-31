#!/usr/bin/env python

"""@trajectory_msg_converter_mavros.py
This node converts Fast-Planner reference trajectory message to  Position Target Message which is accepted by MAVROS
Authors: Arpit Amin, Codex Labs
"""

# Imports
import rospy
#from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint # for geometric_controller
from mavros_msgs.msg import PositionTarget #for MAVROS
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
from geometry_msgs.msg import Transform, Twist, Accel
from nav_msgs.msg import Odometry


class MessageConverter:

    def __init__(self):
        rospy.init_node('trajectory_msg_converter')

        fast_planner_traj_topic = rospy.get_param('~fast_planner_traj_topic', 'planning/pos_cmd')

        traj_pub_topic = rospy.get_param('~traj_pub_topic', '/mavros/setpoint_raw/local')
        odom_topic = rospy.get_param('~odom_topic', '/airsim_node/PX4/odom_local_enu')
        # Publisher 
        self.traj_pub = rospy.Publisher(traj_pub_topic, PositionTarget, queue_size=10)
        self.odom = None
        self.has_odom = False
        self.publish = False
        # Subscriber for Fast-Planner reference trajectory
        rospy.Subscriber(fast_planner_traj_topic, PositionCommand, self.fastPlannerTrajCallback, tcp_nodelay=True)
        rospy.Subscriber(odom_topic, Odometry, self.odomCallback, tcp_nodelay=True)
        rospy.spin()

    def rawmsg(self, x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_dot):
        raw_msg = PositionTarget()
        raw_msg.header.frame_id = "world" #May need to change this, potentially wrong
        raw_msg.header.stamp = rospy.Time.now()
        #Coordinate Frame
        #uint8 FRAME_LOCAL_NED = 1
        #uint8 FRAME_LOCAL_OFFSET_NED = 7
        #uint8 FRAME_BODY_NED = 8
        #uint8 FRAME_BODY_OFFSET_NED = 9
        raw_msg.coordinate_frame = 1
        raw_msg.type_mask = 0 # refer to ROS message definition of Position Target
        raw_msg.position.x = x
        raw_msg.position.y = y
        raw_msg.position.z = z
        raw_msg.velocity.x = vx
        raw_msg.velocity.y = vy
        raw_msg.velocity.z = vz
        raw_msg.velocity.y = vy
        raw_msg.acceleration_or_force.x = ax
        raw_msg.acceleration_or_force.y = ay
        raw_msg.acceleration_or_force.z = az
        raw_msg.yaw = yaw
        raw_msg.yaw_rate = yaw_dot
        return raw_msg



    def blank_msg(self, odom):
        raw_msg = PositionTarget()
        raw_msg.header.frame_id = "world" #May need to change this, potentially wrong
        raw_msg.header.stamp = rospy.Time.now()
        #Coordinate Frame
        #uint8 FRAME_LOCAL_NED = 1
        #uint8 FRAME_LOCAL_OFFSET_NED = 7
        #uint8 FRAME_BODY_NED = 8
        #uint8 FRAME_BODY_OFFSET_NED = 9
        raw_msg.coordinate_frame = 1
        raw_msg.type_mask = 0 # refer to ROS message definition of Position Target
        raw_msg.position.x = odom.pose.pose.position.x
        raw_msg.position.y = odom.pose.pose.position.y
        raw_msg.position.z = odom.pose.pose.position.z
        raw_msg.velocity.x = 0
        raw_msg.velocity.y = 0
        raw_msg.velocity.z = 0
        raw_msg.velocity.y = 0
        raw_msg.acceleration_or_force.x = 0
        raw_msg.acceleration_or_force.y = 0
        raw_msg.acceleration_or_force.z = 0
        raw_msg.yaw = 0
        raw_msg.yaw_rate = 0
        return raw_msg

    def odomCallback(self, msg):
        if self.no_trajectory:
            raw_msg = self.blank_msg(msg)
            self.traj_pub.publish(raw_msg)

    def fastPlannerTrajCallback(self, msg):
        self.no_trajectory = False
        raw_msg = self.rawmsg(msg.position.x, msg.position.y, msg.position.z,
        msg.velocity.x, msg.velocity.y, msg.velocity.z, msg.acceleration.x, msg.acceleration.y, msg.acceleration.z,
        msg.yaw, msg.yaw_dot)
        self.traj_pub.publish(raw_msg)
        self.no_trajectory = True

if __name__ == '__main__':
    obj = MessageConverter()

    
  