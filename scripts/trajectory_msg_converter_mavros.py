#!/usr/bin/env python

# Imports
import rospy
#from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint # for geometric_controller
from mavros_msgs.msg import PositionTarget #for MAVROS
from quadrotor_msgs.msg import PositionCommand # for Fast-Planner
from geometry_msgs.msg import Transform, Twist, Accel, PoseStamped
from nav_msgs.msg import Odometry, Path
import time
from mavros_msgs.srv import *
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult
# from mavros_test_common import MavrosTestCommon

class MessageConverter:
    def __init__(self):
        rospy.init_node('trajectory_msg_converter')
        fast_planner_traj_topic = rospy.get_param('~fast_planner_traj_topic', 'planning/pos_cmd')
        global fast_planner_traj_topic_2 
        fast_planner_traj_topic_2 = fast_planner_traj_topic
        traj_pub_topic = rospy.get_param('~traj_pub_topic', '/mavros/setpoint_raw/local')
        odom_topic = rospy.get_param('~odom_topic', '/airsim_node/PX4/odom_local_enu')
        trigger_topic = rospy.get_param('~trigger_topic', '/traj_start_trigger')
        # Publisher 
        self.traj_pub = rospy.Publisher(traj_pub_topic, PositionTarget, queue_size=10)
        self.exploration_triggered = False
        fast_traj = fast_planner_traj_topic
        traj_pub_2 = traj_pub_topic
        odom_topic_2 = odom_topic
        trig_topic_2 = trigger_topic
        #if the drone is not in exploration mode, send blank message 
        # to keep offboard mode

        
    
    def executeMission(self):
        rospy.loginfo("Fast planner trajectory initialization complete")
        rospy.Subscriber(fast_planner_traj_topic_2, PositionCommand, self.fastPlannerTrajCallback, tcp_nodelay=True)
        """ rospy.Subscriber(odom_topic_2, Odometry, self.odomCallback, tcp_nodelay=True)
        rospy.Subscriber(trig_topic_2, PoseStamped, self.triggerCallback, tcp_nodelay=True) """
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
        raw_msg.acceleration_or_force.x = ax
        raw_msg.acceleration_or_force.y = ay
        raw_msg.acceleration_or_force.z = az
        raw_msg.yaw = -yaw
        raw_msg.yaw_rate = 0
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

    #sends a box to mavros/setpoint/local to get the imu ready for VINS
    def odomInitTrajMsg(self):
        t_end = time.time() + 60 * 0.1
        while time.time() < t_end:
          raw_msg = self.rawmsg(0,0,0.61,0,0,0.5,0,0,0.1,0,0)
          rospy.loginfo("Publishing first point")
          self.traj_pub.publish(raw_msg)
        rospy.sleep(0.1)
        
        t_end = time.time() + 60 * 0.1
        while time.time() < t_end:
          raw_msg = self.rawmsg(1,0,0.61,0.5,0,0,0.1,0,0,0.5,0.0)
          rospy.loginfo("Publishing second point")
          self.traj_pub.publish(raw_msg)
        rospy.sleep(0.1)

        t_end = time.time() + 60 * 0.1
        while time.time() < t_end:
          raw_msg = self.rawmsg(1,1,0.61,0,0.5,0,0,0.1,0,0.5,0.0)
          rospy.loginfo("Publishing third point")
          self.traj_pub.publish(raw_msg)
        rospy.sleep(0.1)

        t_end = time.time() + 60 * 0.1
        while time.time() < t_end:
          raw_msg = self.rawmsg(0,1,0.61,-0.50,0,0,-0.1,0,0,0.5,0.0)
          rospy.loginfo("Publishing fourth point")
          self.traj_pub.publish(raw_msg)
        rospy.sleep(0.1)

        t_end = time.time() + 60 * 0.1
        while time.time() < t_end:
          raw_msg = self.rawmsg(0,0,0.61,0,-0.50,0,0.0,-0.1,0,0.5,0.0)
          rospy.loginfo("Publishing fifth point")
          self.traj_pub.publish(raw_msg)
        rospy.sleep(0.1)
        

    def odomCallback(self, msg):
        if not self.exploration_triggered:
            raw_msg = self.blank_msg(msg)
            self.traj_pub.publish(raw_msg)

    def fastPlannerTrajCallback(self, msg):
        raw_msg = self.rawmsg(msg.position.x, msg.position.y, msg.position.z,
        msg.velocity.x, msg.velocity.y, msg.velocity.z, msg.acceleration.x, msg.acceleration.y, msg.acceleration.z,
        msg.yaw, msg.yaw_dot)
        self.traj_pub.publish(raw_msg)
    
    def triggerCallback(self, msg):
        self.exploration_triggered = True
    
    

    def flyTraj(self):
        #Timer to ensure that commander is armed and in offboard mode before sending setpoints
        # x  = 60
        # while(x >= 0):
        #     rospy.loginfo("Please wait %d seconds before arming the drone" % x)
        #     rospy.sleep(1)
        #     x = x - 1
        #Drone flies box pattern to get imu setup
        
        x = 10
        while x >= 0:
          rospy.sleep(1)
          rospy.loginfo("takeoff mode set, time to start %d" % x)
          x = x - 1
        t_end = time.time() + 60 * 1.5
        while time.time() < t_end:  
           """ if(time.time() + 10 > t_end):
               rospy.loginfo("Attemptiing to start FUEL")
               obj.executeMission()  """
           rospy.loginfo("Entered while loop to run odom conversion")
           self.odomInitTrajMsg()

    def setTakeoff(self):
       rospy.wait_for_service('mavros/cmd/takeoff')
       try:
          takeoffService = rospy.ServiceProxy(
            '/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
          takeoffService(altitude=0.61, latitude=47.641468,
                       longitude=-122.140165, min_pitch=0, yaw=1.0)
          rospy.loginfo("Taking off to 0.61 m")
       except rospy.ServiceException as e: 
          print ("Service takeoff call failed: %s" %e)
    
    def setLoiterMode(self):
       rospy.wait_for_service('/mavros/set_mode')
       try:
          flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
          isModeChanged = flightModeService(custom_mode='AUTO.LOITER')  # return true or false
       except rospy.ServiceException as e:
          print("service set_mode call failed: %s. AUTO.LOITER Mode could not be set")
        
    def sendNavGoal(self):
        client = actionlib.SimpleActionClient('/traj_start_trigger', MoveBaseAction)
        rospy.loginfo("Waiting for move base server")
        rospy.loginfo(client)
        #client.wait_for_server()
        rospy.loginfo("Server setup is successful")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'world' 
        goal.target_pose.pose.position.x = 5.0
        goal.target_pose.pose.position.y = -5.0
        goal.target_pose.pose.position.z = 0.61
        goal.target_pose.pose.orientation.z = 0.727
        goal.target_pose.pose.orientation.w = 0.686
        rospy.loginfo("Ready to send goal")
        client.send_goal(goal)
        rospy.loginfo("Finished sending nav goal")
        #client.wait_for_result()

    def setOffboardMode(self):
        rospy.loginfo("Entered the offboard mode function")
        self.odomInitTrajMsg()
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
            response = flightModeService(custom_mode='OFFBOARD')
            return response.mode_sent
        except rospy.ServiceException as e:
            print
            "service set_mode call failed: %s. Offboard Mode could not be set." % e
            return False
        
        
        

if __name__ == '__main__':
    rospy.loginfo("Preparing to run the message converter")
    obj = MessageConverter()
    x = 30
    obj.setTakeoff()
    while x >= 0: 
      rospy.loginfo("%d seconds to setup rviz" % x)
      rospy.sleep(1)
      x = x - 1
    obj.setOffboardMode()
    obj.flyTraj()
    obj.setLoiterMode()
    obj.setTakeoff()
    obj.sendNavGoal()
    obj.setOffboardMode()
    # x = 10
    # while x >= 0:
    #    rospy.sleep(1)
    #    rospy.loginfo("Please set to offboard mode and preapre fuel in %d seconds" % x)
    #    x = x - 1
    obj.executeMission() #this is causing an error because traj topics are not global
    rospy.loginfo("Program end")
    
    

    
  