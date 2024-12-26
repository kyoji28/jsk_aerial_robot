#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Point
from std_msgs.msg import String
from jsk_recognition_msgs.msg import HumanSkeletonArray
from jsk_recognition_msgs.msg import HumanSkeleton
from std_msgs.msg import Empty

class gesture_react():
    def __init__(self):
        self.msgs_pub = rospy.Publisher('chatter', String, queue_size = 10)
        self.skeleton_sub = rospy.Subscriber('/edgetpu_human_pose_estimator/output/skeletons',HumanSkeletonArray, self.skeleton_cb)
        self.takeoff_pub = rospy.Publisher('/quadrotor/teleop_command/takeoff',Empty, queue_size = 10)
        self.skeletons = HumanSkeletonArray()
        self.handsup_flag = False
        self.bone_names = []
        self.bones = []
        self.start = Point()
        self.end = Point()
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timerCallback)

    def skeleton_cb(self,msg):
        self.skeletons = msg
        # if the borns are not found, coral TPU generates nothing but an "empty" skeleton
        if self.skeletons.skeletons != []:
            for skeleton in self.skeletons.skeletons:
                self.bone_names = skeleton.bone_names
                self.bones = skeleton.bones
        else:
            self.bone_names = []
            self.bones = []

    # search "left wrist" or "right wrist" in self.bone_names
    # search "left shoulder" or "right shoulder" in self.bone_names
    # Task 1 if there are these 4 bones in self.bone_names, get the bone's coordinates in self.bones
    # Task 2 you should find whether the target bone is before or after arrow "->"
    # Task 3 compare the shoulder bone's coodinate with the wrist bone's coodinate
    def search_bone(self):
        target_shoulder_bones =  ['left shoulder', 'right shoulder']
        target_wrist_bones = ['left wrist', 'right wrist']
        #initialize
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
                # Is this condition true ? plz check it!
                if wrist_bone_coordinates.y <  shoulder_bone_coordinates.y:
                    self.handsup_flag = True
                    self.msgs_pub.publish("Hands up")
                    print("Hands up")
            else:
                rospy.logwarn("Required bones not found.")
                #if wrist_bone_coordinates.y >shoulder_bone_coordinates.y:                                                                                                                            
                # self.msgs_pub.publish("success!!")                                                                                                                                                       
                # print("success")                                                                                                                                                                         

    def cmd_vel(self):
        pass
        # Task 4 if wrist_bone_coordinates.y > shoulder_bone_coordinates.y is true, publish cmd_vel to tello                                                                                               


    def takeoff(self):
        if self.handsup_flag == True:
            self.takeoff_pub.publish(Empty())
            self.handsup_flag = False



    def timerCallback(self,event):
        try:
            self.search_bone()
            self.takeoff()
        except Exception as e:
            pass
           # rospy.logerr(f"Error in timerCallback: {e}")


if __name__ == '__main__':
    rospy.init_node("gesture_react")
    node = gesture_react()
    rospy.spin()
