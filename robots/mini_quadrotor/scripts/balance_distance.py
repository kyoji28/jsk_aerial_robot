#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
# import keyboard
from geometry_msgs.msg import Point
from std_msgs.msg import String
from jsk_recognition_msgs.msg import RectArray
from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav

class balance_distance():
    def __init__(self):
        rospy.loginfo('activate')
        self.camera_height = 480
        self.camera_width =  640
        self.fly_flag = False

        self.nav_msg = FlightNav()
        self.nav_msg.control_frame = FlightNav.WORLD_FRAME
        self.nav_msg.target = FlightNav.COG
        self.nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
        self.nav_pub = rospy.Publisher('/quadrotor/uav/nav', FlightNav, queue_size=1)
        self.xy_vel = rospy.get_param("xy_vel", 0.2)
        self.yaw_vel = rospy.get_param("yaw_vel", 0.2)

        self.takeoff_sub = rospy.Subscriber('/quadrotor/teleop_command/takeoff',Empty, self.takeoff_cb)
        self.land_sub = rospy.Subscriber('/quadrotor/teleop_command/land', Empty, self.land_cb)
        self.rect_sub = rospy.Subscriber('/edgetpu_face_detector/output/rects', RectArray, self.rect_cb)
        self.rects = RectArray()
        self.max_index = 0
        self.max_rect_area = 0.0
        self.max_rect = RectArray()
        self.rect_pos = Vector3()
        self.max_param_area = 100000 #rospy.getparam("~max_param_area", 1.0)
        self.min_param_area = 40000
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timerCallback)
        self.first_flag = True
        self.rotate_flag  =True
        self.n = 0
        self.cnt = 0
        self.rotate_cnt = 0


    def rect_cb(self,msg):
        self.rects = msg
        self.n = len (self.rects.rects)
        #rospy.loginfo(self.n)

    def takeoff_cb(self,msg):
        self.fly_flag = True

    def land_cb(self,msg):
        self.fly_flag = False

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


    # def rotate_degree_cal(self):
    #     self.rect_pos.x = (self.max_rect.x + (self.max_rect.width/2)) -(self.camera_width/2)
    #     self.rect_pos.y = -(self.max_rect.y + (self.max_rect.height/2)) + (self.camera_height/2)
    #     self.yaw = self.rect_pos.x * (math.pi/self.camera_width)* 0.8
    #     print(self.yaw)
    #     self.cmd_msg.angular.z = self.yaw
    #     self.cmd_vel_pub.publish(self.cmd_msg)
    #     rospy.sleep(1.0)
    #     self.halt()
    #     print("degree_halt")
    #     rospy.loginfo(self.rect_pos.x)
    #     rospy.loginfo(self.rect_pos.y)
    #     self.rotate_flag = False
    # #人間の枠を常に中心に持っていく


    def balance_distance(self):
        if self.max_param_area > self.max_rect_area:
            if self.min_param_area < self.max_rect_area:
                self.halt()
                rospy.loginfo("Good Distance:{}".format(self.max_rect_area))

            elif self.min_param_area > self.max_rect_area:
                self.nav_msg.target_vel_x = self.xy_vel
                self.nav_pub.publish(self.nav_msg)
                rospy.loginfo("Approaching:{}".format(self.max_rect_area))

        elif self.max_param_area < self.max_rect_area:
            self.nav_msg.target_vel_x = -self.xy_vel
            self.nav_pub.publish(self.nav_msg)
            rospy.loginfo("Getting Away:{}".format(self.max_rect_area))


            # rospy.loginfo(self.max_rect_area)
            # self.rotate_cnt += 1
            # if self.rotate_cnt >= 4:
            #     self.rotate_flag = True
            #     self.rotate_cnt = 0



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

            if self.n >= 1:
                self.max_rect_cal()
                self.balance_distance()

        else:
            pass


if __name__ == '__main__':
    rospy.init_node("balance_distance")
    node = balance_distance()
    rospy.spin()
