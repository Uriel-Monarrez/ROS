#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Float64

class JointPub(object):
    def __init__(self):

        self.publishers_array = []
        self.pub_ankle1_r = rospy.Publisher('/darwin/j_ankle1_r_position_controller/command', Float64, queue_size=10)
        self.pub_ankle2_r = rospy.Publisher('/darwin/j_ankle2_r_position_controller/command', Float64, queue_size=10)
        self.pub_pelvis_r = rospy.Publisher('/darwin/j_pelvis_r_position_controller/command', Float64, queue_size=10)
        self.pub_tibia_r = rospy.Publisher('/darwin/j_tibia_r_position_controller/command', Float64, queue_size=10)
        self.pub_thigh2_r = rospy.Publisher('/darwin/j_thigh2_r_position_controller/command', Float64, queue_size=10)
        self.pub_thigh1_r = rospy.Publisher('/darwin/j_thigh1_r_position_controller/command', Float64, queue_size=10)

                
        self.publishers_array.append(self.pub_ankle1_r)
        self.publishers_array.append(self.pub_ankle2_r)
        self.publishers_array.append(self.pub_pelvis_r)
        self.publishers_array.append(self.pub_tibia_r)
        self.publishers_array.append(self.pub_thigh2_r)
        self.publishers_array.append(self.pub_thigh1_r)




        self.init_pos = [0.0,0.0,0.0,0.0,0.0,0.0]

    def set_init_pose(self):
        """
        Sets joints to initial position [0,0,0]
        :return:
        """
        self.check_publishers_connection()
        self.move_joints(self.init_pos)


    def check_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while (self.pub_ankle1_r.get_num_connections() == 0):
            rospy.logdebug("No susbribers to pub_ankle1_r yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("pub_ankle1_r Publisher Connected")

        while (self.pub_ankle2_r.get_num_connections() == 0):
            rospy.logdebug("No susbribers to pub_ankle2_r yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("pub_ankle2_r Publisher Connected")

        while (self.pub_pelvis_r.get_num_connections() == 0):
            rospy.logdebug("No susbribers to pub_pelvis_r yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("pub_pelvis_r Publisher Connected")

        while (self.pub_tibia_r.get_num_connections() == 0):
            rospy.logdebug("No susbribers to pub_tibia_r yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("pub_tibia_r Publisher Connected")

        while (self.pub_thigh2_r.get_num_connections() == 0):
            rospy.logdebug("No susbribers to pub_thigh2_r yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("pub_thigh2_r Publisher Connected")

        while (self.pub_thigh1_r.get_num_connections() == 0):
            rospy.logdebug("No susbribers to pub_thigh1_r yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("pub_thigh1_r Publisher Connected")

        rospy.logdebug("All Publishers READY")

    def joint_mono_des_callback(self, msg):
        rospy.loginfo("Coso ese->"+str(msg.joint_state.position))
        
        self.move_joints(msg.joint_state.position)

    def move_joints(self, joints_array):

        i = 0
        for publisher_object in self.publishers_array:
          joint_value = Float64()
          joint_value.data = joints_array[i]
          rospy.logdebug("JointsPos>>"+str(joint_value))
          
          publisher_object.publish(joint_value)
          i += 1


    def start_loop(self, rate_value = 2.0):
        rospy.loginfo("Start Loop")
        pos1 = [0.0,0.0,1.6]
        pos2 = [0.0,0.0,-1.6]
        position = "pos1"
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
          if position == "pos1":
            self.move_joints(pos1)
            position = "pos2"
          else:
            self.move_joints(pos2)
            position = "pos1"
          rate.sleep()

    def start_sinus_loop(self, rate_value = 2.0):
        rospy.logdebug("Start Loop")
        w = 0.0
        x = 2.0*math.sin(w)
        #pos_x = [0.0,0.0,x]
        #pos_x = [x, 0.0, 0.0]
        pos_x = [0.0, x, 0.0,0.0, 0.0, 0.0]
        rate = rospy.Rate(rate_value)
        while not rospy.is_shutdown():
            self.move_joints(pos_x)
            w += 0.05
            x = 2.0 * math.sin(w)
            #pos_x = [0.0, 0.0, x]
            #pos_x = [x, 0.0, 0.0]
            pos_x = [0.0, x, 0.0,0.0, 0.0, 0.0]
            rate.sleep()


if __name__=="__main__":
    rospy.init_node('joint_publisher_node')
    joint_publisher = JointPub()
    rate_value = 50.0
    rospy.loginfo("joints listos")
    #joint_publisher.start_loop(rate_value)
    joint_publisher.start_sinus_loop(rate_value)