#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Point
from std_msgs.msg import String
from jsk_recognition_msgs.msg import HumanSkeletonArray
from jsk_recognition_msgs.msg import HumanSkeleton
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav

class gesture_rotate():
    def __init__(self):
        rospy.loginfo('activate')
        self.camera_height = 480
        self.camera_width = 640

        self.skeleton_sub = rospy.Subscriber('/edgetpu_human_pose_estimator/output/skeletons',HumanSkeletonArray, self.skeleton_cb)
        self.takeoff_pub = rospy.Publisher('/quadrotor/teleop_command/takeoff',Empty, queue_size = 10)
        self.nav_pub = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=1)
        self.xy_vel = rospy.get_param("xy_vel", 0.2)
        self.yaw_vel = rospy.get_param("yaw_vel", 0.2)
        
        self.skeletons = HumanSkeletonArray()
        self.right_hand_up_flag = False
        self.left_hand_up_flag = False
        self.bone_names = []
        self.bones = []
        self.start = Point()
        self.end = Point()
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timerCallback)

    def skeleton_cb(self,msg):
        self.skeletons = msg
        if self.skeletons.skeletons == []:
            self.bone_names = []
            self.bones = []
        # if the borns are not found, coral TPU generates nothing but an "empty" skeleton
        # if self.skeletons.skeletons != []:
        #     for skeleton in self.skeletons.skeletons:
        #         self.bone_names = skeleton.bone_names
        #         self.bones = skeleton.bones
        # else:
        #     self.bone_names = []
        #     self.bones = []


    def search_bone(self):
        target_shoulder_bones =  ['left shoulder', 'right shoulder']
        target_right_wrist = ['right wrist']
        target_left_wrist = ['left wrist']
        #initialize
        find_shoulder_bone_name = None
        self.shoulder_bone_coordinates = []
        self.right_wrist_coordinates = []
        self.left_wrist_coordinates =[]

        # if len(self.bone_names) == 0 or len(self.bones) == 0:
        #      rospy.loginfo("no bones")
        #      #return
        if self.skeletons.skeletons != []:
            for skeleton in self.skeletons.skeletons:
                self.bone_names = skeleton.bone_names
                self.bones = skeleton.bones

                for i, bone_name in enumerate(self.bone_names):
                    # rospy.logerr(f"bone_name {bone_name}")
                    # if '->' in bone_name:
                    parts = bone_name.split('->')
                    start_bone_name = parts[0].strip()
                    end_bone_name = parts[1].strip()
                    bone = self.bones[i]

                    if start_bone_name in target_right_wrist:
                        self.right_wrist_coordinates = bone.start_point
                    elif end_bone_name in target_right_wrist:
                        self.right_wrist_coordinates = bone.end_point

                    if start_bone_name in target_left_wrist:
                        self.left_wrist_coordinates = bone.start_point
                    elif end_bone_name in target_left_wrist:
                        self.left_wrist_coordinates = bone.end_point

                    if start_bone_name in target_shoulder_bones:
                        find_shoulder_bone_name = start_bone_name
                        self.shoulder_bone_coordinates = bone.start_point
                    elif end_bone_name in target_shoulder_bones:
                        find_shoulder_bone_name = end_bone_name
                        self.shoulder_bone_coordinates = bone.end_point

                if self.right_wrist_coordinates and self.shoulder_bone_coordinates:
                    rospy.loginfo(f"Right Wrist: {self.right_wrist_coordinates.y}")
                    rospy.loginfo(f"{find_shoulder_bone_name}: {self.shoulder_bone_coordinates.y}")

                    if self.right_wrist_coordinates.y < self.shoulder_bone_coordinates.y:
                        self.right_hand_up_flag = True
                        print("Right Hand Up")

                if self.left_wrist_coordinates and self.shoulder_bone_coordinates:
                    rospy.loginfo(f"Left Wrist: {self.left_wrist_coordinates.y}")
                    rospy.loginfo(f"{find_shoulder_bone_name}: {self.shoulder_bone_coordinates.y}")

                    if self.left_wrist_coordinates.y < self.shoulder_bone_coordinates.y:
                        self.left_hand_up_flag = True
                        print("Left Hand Up")
                else:
                    pass
                # rospy.logwarn("Required bones not found.")
        else:
            rospy.loginfo("no bones")



    def cmd_vel(self):
        self.nav_msg = FlightNav()
        self.nav_msg.control_frame = FlightNav.WORLD_FRAME
        self.nav_msg.target = FlightNav.COG
        # rospy.loginfo("self.right_hand_up_flag: {}".format(self.right_hand_up_flag))
        # rospy.loginfo("self.left_hand_up_flag: {}".format(self.left_hand_up_flag))
        if self.right_hand_up_flag:
            if self.camera_width/2 < self.right_wrist_coordinates.x:
                self.nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                self.nav_msg.target_omega_z = -self.yaw_vel
                self.nav_pub.publish(self.nav_msg)
                self.right_hand_up_flag = False
                # rospy.loginfo("reset self.right_hand_up_flag: {}".format(self.right_hand_up_flag))
            else:
                self.nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                self.nav_msg.target_omega_z = self.yaw_vel
                self.nav_pub.publish(self.nav_msg)
                self.right_hand_up_flag = False

        elif self.left_hand_up_flag:
            if self.camera_width/2 < self.left_wrist_coordinates.x:
                self.nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                self.nav_msg.target_omega_z = -self.yaw_vel
                self.nav_pub.publish(self.nav_msg)
                self.left_hand_up_flag = False
            else:
                self.nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                self.nav_msg.target_omega_z = self.yaw_vel
                self.nav_pub.publish(self.nav_msg)
                self.left_hand_up_flag = False


    def timerCallback(self,event):
        try:
            self.search_bone()
            self.cmd_vel()
        except Exception as e:
            pass
           # rospy.logerr(f"Error in timerCallback: {e}")


if __name__ == '__main__':
    rospy.init_node("gesture_rotate")
    node = gesture_rotate()
    rospy.spin()
