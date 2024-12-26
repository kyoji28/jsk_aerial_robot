#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from std_msgs.msg import String
from std_msgs.msg import Empty
from speech_recognition_msgs.msg import SpeechRecognitionCandidates

class voice_operate():
    def __init__(self):

        # self.fly_flag = False
        # self.arm_flag = False
        
        self.start_pub = rospy.Publisher('/quadrotor/teleop_command/start', Empty, queue_size = 10)
        self.takeoff_pub = rospy.Publisher('/quadrotor/teleop_command/takeoff',Empty, queue_size = 10)
        self.land_pub = rospy.Publisher('/quadrotor/teleop_command/land',Empty,queue_size = 10)
        self.voice_text_sub = rospy.Subscriber('/speech_to_text', SpeechRecognitionCandidates, self.voice_cb)
        

    def voice_cb(self, msg):
        print(msg.transcript[0])
        command = msg.transcript[0]
        confidence =  msg.confidence[0]

        if "start" in command:
            print("Start, {}".format(confidence))
            # self.arm_flag = True
            self.start_pub.publish(Empty())
            
        if "take" in command and "off" in command:
            print("Takeoff, {}".format(confidence))
            # TODO1: pusblish takeoff command
            # self.fly_flag = True
            self.takeoff_pub.publish((Empty())

        #TODO2: other command: land, move, etc...
        if "land" in command:
            print("Land,{}".format(confidence))
            # self.fly_flag = False
            self.land_pub.publish(Empty())


        

if __name__ == '__main__':
    rospy.init_node("voice_operate")
    node = voice_operate()
    rospy.spin()

