#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Empty
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from geometry_msgs.msg import Point
from jsk_recognition_msgs.msg import RectArray
from geometry_msgs.msg import Vector3
from aerial_robot_msgs.msg import FlightNav

class call_approach():
    def __init__(self):
        rospy.loginfo("activate")
        self.camera_height = 480
        self.camera_width = 640 #check later <---ok!!
        self.fly_flag = False

        self.nav_msg = FlightNav()
        self.nav_msg.control_frame = FlightNav.WORLD_FRAME
        self.nav_msg.target = FlightNav.COG
        self.nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
        self.nav_pub = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=1)
        self.xy_vel = rospy.get_param("xy_vel", 0.2)

        self.takeoff_sub = rospy.Subscriber('/quadrotor/teleop_command/takeoff', Empty, self.takeoff_cb)
        self.land_sub = rospy.Subscriber('/quadrotor/teleop_command/land', Empty, self.land_cb)
        self.rect_sub = rospy.Subscriber('/edgetpu_face_detector/output/rects', RectArray, self.rect_cb)
        self.approach_sub = rospy.Subscriber('/quadrotor/teleop_command/approach',Empty, self.approach_cb)
        self.away_sub = rospy.Subscriber('/quadrotor/teleop_command/away',Empty,self.away_cb)
        self.rects = RectArray()
        self.max_index = 0
        self.max_rect_area = 0.0
        self.max_rect = RectArray()
        self.rect_pos = Vector3()
        self.max_param_area = 20000
        self.min_param_area = 10000
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timerCallback)
        # self.first_flag = True
        self.approach_flag = False
        self.away_flag = False
        self.n = 0
        self.cnt = 0
        self.away_cnt = 0

    def rect_cb(self,msg):
        self.rects = msg
        self.n = len(self.rects.rects)

    def takeoff_cb(self,msg):
        self.fly_flag = True

    def land_cb(self,msg):
        self.fly_flag = False

    def approach_cb(self,msg):
        self.approach_flag = True
        self.away_flag = False

    def away_cb(self,msg):
        self.away_flag = True
        self.approach_flag = False

    def halt(self):
        self.nav_msg.target_vel_x = 0.0
        self.nav_msg.target_vel_y = 0.0
        self.nav_pub.publish(self.nav_msg)

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
        #rospy.loginfo(self.max_rect_area)

    def adjust_distance(self):
        if self.max_param_area > self.max_rect_area:
            if self.min_param_area < self.max_rect_area:
                self.halt()
                rospy.loginfo("Good Distance:{}".format(self.max_rect_area))

            elif self.min_param_area > self.max_rect_area:
                self.nav_msg.target_vel_x = self.xy_vel
                self.nav_msg.target_vel_y = self.xy_vel
                self.nav_pub.publish(self.nav_msg)
                rospy.loginfo("Approachikng:{}".format(self.max_rect_area))

        elif self.max_param_area < self.max_rect_area:
            rospy.loginfo("Stay:{}".format(self.max_rect_area))
            # self.nav_msg.target_vel_y = self.xy_vel
            # self.nav_pub.publish(self.nav_msg)
            # rospy.loginfo("Getting Away:{}".format(self.max_rect_area))

    def timerCallback(self,event):
        if self.fly_flag:
            if self.n >= 1:
                if self.approach_flag:
                    self.max_rect_cal()
                    self.adjust_distance()
                    self.cnt += 1
                    print(self.cnt)

                    if self.cnt > 50:
                        self.cnt = 0
                        self.approach_flag = False
                    else:
                        pass
                elif self.away_flag:
                    self.max_rect_cal()
                    self.nav_msg.target_vel_x = -self.xy_vel
                    self.nav_msg.target_vel_y = -self.xy_vel
                    self.nav_pub.publish(self.nav_msg)
                    rospy.loginfo("Getting Away:{}".format(self.max_rect_area))
                    self.away_cnt += 1
                    print(self.away_cnt)

                    if self.away_cnt > 50:
                        self.away_cnt = 0
                        self.away_flag = False
            else:
                pass
        else:
            pass


if __name__ == '__main__':
    rospy.init_node("call_approach")
    node = call_approach()
    rospy.spin()
