#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
# import keyboard
from geometry_msgs.msg import Point
from std_msgs.msg import String
from jsk_recognition_msgs.msg import HumanSkeletonArray
from jsk_recognition_msgs.msg import HumanSkeleton
from jsk_recognition_msgs.msg import RectArray
from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist


class Move_to_human():
    def __init__(self):
        rospy.loginfo('activate')

        self.camera_height = 720 #Going to modify later because this size is for tello
        self.camera_width =  960 #Going to modify later because this size is for tello as well
        self.fly_flag = False
        # self.cmd_vel_pub = rospy.Publisher('/quadrotor/teleop_command/cmd_vel', Twist, queue_size = 10) #add '/quadrotor/cmd_vel' as a topic to send velocity command to the quadrotor
        # self.cmd_msg = Twist()
        self.
        self.takeoff_sub = rospy.Subscriber('/quadrotor/teleop_command/takeoff',Empty, self.takeoff_cb) #add '/quadrotor/teleop_command/takeoff' as a topic
        self.land_sub = rospy.Subscriber('/quadrotor/teleop_command/land', Empty, self.land_cb)
        self.rect_sub = rospy.Subscriber('/human/output/rects', RectArray, self.rect_cb)
        self.rects = RectArray()
        self.max_index = 0
        self.max_rect_area = 0.0
        self.max_rect = RectArray()
        self.rect_pos = Vector3()
        self.max_param_area = 370000 #rospy.getparam("~max_param_area", 1.0)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timerCallback)
        self.first_flag = True
        self.rotate_flag  =True
        self.n = 0
        self.m = 0
        self.cnt = 0
        self.rotate_cnt = 0

        self.skeleton_sub = rospy.Subscriber('/edgetpu_human_pose_estimator/output/skeletons',HumanSkeletonArray, self.skeleton_cb)
        self.takeoff_pub = rospy.Publisher('/quadrotor/teleop_command/takeoff',Empty, queue_size = 10)
        self.skeletons = HumanSkeletonArray()
        self.handsup_flag = False
        self.bone_names = []
        self.bones = []
        self.start = Point()
        self.end = Point()
        self.

    def skeleton_cb(self,msg):
        self.skeletons = msg
        #if borns are not found, coral TPU generates nothing but an "empty" skeleton)
        if self.skeletons.skeletons != []:
            for skeleton in self.skeletons.skeletons:
                self.bone_names = skeleton.bone_names
                self.bones = skeleton.bones
        else:
            self.bone_names = []
            self.bones = []

    def search_bone(self):
        target_shoulder_bones = ['left shoulder', 'right shoulder']
        target_wrist_bones = ['left wrist', 'right wrist']
        find_shoulder_bone_name = None
        find_wrist_bone_name = None
        shoulder_bone_coordinates = []
        wrist_bone_coordinates = []

        if len(self.bone_names) == 0 or len(self.bones) == 0:
            rospy.loginfo("no bones")
            return
        else:
            for i, bone_name in enumerate(self.bone_names):
                rospy.logerr(f"bone_name {bone_name}")
                if '->' in bone_name:
                    parts = bone_name.split('->')
                    start_bone_name = parts[0].strip()
                    end_bone_name = parts[1].strip()

                    bone = self.bones[i]

                    if start_bone_name in target_wrist_bones:
                        find_wrist_bone_name = start_bone_name
                        wrist_bone_coordinates = bone.start_point
                    elif end_bone_name in target_wrist_bones:
                        find_wrist_bone_name = end_bone_name
                        wrist_bone_coordinates = bone.end_point

                    if start_bone_name in target_shoulder_bones:
                        find_shoulder_bone_name = start_bone_name
                        shoulder_bone_coordinates = bone.start_point
                    elif end_bone_name in target_shoulder_bones:
                        find_shoulder_bone_name = end_bone_name
                        shoulder_bone_coordinates = bone.end_point

            if wrist_bone_coordinates and shoulder_bone_coordinates:
                rospy.loginfo(f"Found wrist bone: {find_wrist_bone_name}, Coordinates: {wrist_bone_coordinates.y}")
                rospy.loginfo(f"Found shoulder bone: {find_shoulder_bone_name}, Coordinates: {shoulder_bone_coordinates.y}")
                if wrist_bone_coordinates.y < shoulder_bone_coordinates.y:
                    self.handsup_flag = True
                    self.msgs_pub.publish("Hands up")
                    print("Hands up")
            else:rospy.logwarn("Required bones not found")


    def cmd_vel(self):
        pass

        
    def rect_cb(self,msg):
        self.rects = msg
        self.n = len (self.rects.rects)

    def takeoff_cb(self,msg):
        self.fly_flag = True

    def land_cb(self,msg):
        self.fly_flag = False

    def halt(self):
        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.linear.y = 0.0
        self.cmd_msg.linear.z = 0.0
        self.cmd_msg.angular.x = 0.0
        self.cmd_msg.angular.y = 0.0
        self.cmd_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_msg)


    def max_rect_cal(self):
        max_area = 0
        for i in range(self.n):
            rect = self.rects.rects[i]
            tmp_area = rect.height*rect.width
            if tmp_area >= max_area:
                max_area = tmp_area
                self.max_index = i
        self.max_rect = self.rects.rects[self.max_index]
        self.max_rect_area = max_area


    def rotate_degree_cal(self):
        self.rect_pos.x = (self.max_rect.x + (self.max_rect.width/2)) -(self.camera_width/2)
        self.rect_pos.y = -(self.max_rect.y + (self.max_rect.height/2)) + (self.camera_height/2)
        self.yaw = self.rect_pos.x * (math.pi/self.camera_width)* 0.8
        print(self.yaw)
        self.cmd_msg.angular.z = self.yaw
        self.cmd_vel_pub.publish(self.cmd_msg)
        rospy.sleep(1.0)
        self.halt()
        print("degree_halt")
        rospy.loginfo(self.rect_pos.x)
        rospy.loginfo(self.rect_pos.y)
        self.rotate_flag = False
    #人間の枠を常に中心に持っていく
    def move_to_human(self):
        if self.max_param_area <= self.max_rect_area:
            self.cmd_msg.linear.y = 0.0
            self.cmd_vel_pub.publish(self.cmd_msg)
            print("close_halt")
            rospy.loginfo(self.max_rect_area)
            self.palm_flag = True
        else:
            self.cmd_msg.linear.y = -(0.1/self.max_rect_area*0.001)+0.4
            rospy.loginfo(self.max_rect_area)
            self.cmd_vel_pub.publish(self.cmd_msg)
            self.rotate_cnt += 1
            if self.rotate_cnt >= 4:
                self.rotate_flag = True
                self.rotate_cnt = 0



    def timerCallback(self,event):
        if self.fly_flag:
            if self.first_flag:
                print("sleep_start")
                rospy.sleep(2.0)
                self.halt()
                print("init_halt")
                rospy.sleep(2.0)
                self.first_flag = False
                print("sleep_stop")

            if not self.palm_flag:
                if self.n >= 1:
                    self.max_rect_cal()
                    if self.rotate_flag:
                        self.rotate_degree_cal()
                    self.move_to_human()
                else:
                    self.cnt += 1
                    if self.cnt >=10:
                        self.halt()
                        self.cnt = 0
            else:
                self.palm_land_pub.publish()
                print("palm")


if __name__ == '__main__':
    rospy.init_node("Move_to_human")
    node = Move_to_human()
    rospy.spin()
