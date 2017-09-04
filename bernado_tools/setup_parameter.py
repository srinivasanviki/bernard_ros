#!/usr/bin/env python

import sys
import os
import rospy
import rospkg


#topics
rospy.set_param("right","/servobus/right")
rospy.set_param("left","/servobus/left")
 
#right arm
rospy.set_param("joint_names",['right_pinky','right_ring','right_majeure','right_index','right_thumb','right_wrist','right_bicep','right_bicep_rotate','right_shoulder_up','right_shoulder_side'])
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
rospy.set_param("joint_names",['left_pinky','left_ring','left_majeure','left_index','left_thumb','left_wrist','left_bicep','left_bicep_rotate','left_shoulder_up','left_shoulder_side'])
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

print("Parameter Server Setup for LeftArm done")


