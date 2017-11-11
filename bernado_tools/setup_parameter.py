#!/usr/bin/env python

import sys
import os
import rospy
import rospkg


#topics
rospy.set_param("move","/servobus/move")


rospy.set_param("API_AI_TOKEN","708eb3aa03904152a4889008c48e4c3c")
 
#right arm
rospy.set_param('right_pinky','6')
rospy.set_param('right_ring','5')
rospy.set_param('right_majeure' ,'4')
rospy.set_param('right_index','2')
rospy.set_param('right_thumb','3')
rospy.set_param('right_wrist','7')
rospy.set_param('right_bicep','8')
rospy.set_param('right_bicep_rotate','9')
rospy.set_param('right_shoulder_up','10')
rospy.set_param('right_shoulder_side','11')

#left arm
rospy.set_param('left_pinky','6')
rospy.set_param('left_ring','5')
rospy.set_param('left_majeure' ,'4')
rospy.set_param('left_index','2')
rospy.set_param('left_thumb','3')
rospy.set_param('left_wrist','7')
rospy.set_param('left_bicep','8')
rospy.set_param('left_bicep_rotate','9')
rospy.set_param('left_shoulder_up','10')
rospy.set_param('left_shoulder_side','11')


#head movement

rospy.set_param("head_updown",'12')
rospy.set_param("head_leftright",'13')
rospy.set_param("jaw",'26')
rospy.set_param("eyex",'22')
rospy.set_param("eyes_updown",'24')



# Some visual and Speech coordinators

rospy.set_param("Hi there, friend!", True)
rospy.set_param("I like you too.", True)
rospy.set_param("I know you can't mean that",True)
rospy.set_param("Oh yes ur age is",True)


print("Parameter Server Setup for LeftArm done")


