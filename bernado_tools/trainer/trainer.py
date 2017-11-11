#!/usr/bin/env python

import rospy
from bernard_message.msg import Bernard
from sensor_msgs.msg import JointState
import threading

class BernadController():
    def __init__(self):
        self.lock = threading.Lock()
        self.jointcommand = JointState()
        rospy.init_node('trainer', anonymous=True)
        self.joints={"left_bicep":0.0,
                     "left_bicep_rotate":0.0,
                     "left_shoulder_up":0.0,
                     "left_shoulder_side":0.0,
                     "head_updown":0.0,
                     "head_leftright":0.0,
                     "jaw":0.0,
                     "eyex":0.0,
                     "eyes_updown":0.0,
                     "right_bicep":0.0,
                     "right_bicep_rotate":0.0,
                     "right_shoulder_up":0.0,
                     "right_shoulder_side":0.0}

        self.thread = threading.Thread(target=self.joint_state_listener)
        self.thread.start()


    def joint_state_listener(self):
        rospy.Subscriber('joint_states', JointState, self.joint_state_callback)
        rospy.spin()

    def initialize_bernard(self,port,goal):
        bernard=Bernard()
        bernard.port.data=port
        bernard.rotation.data=goal
        return bernard

    def publish_data(self,bernard):
        rospy.Publisher(rospy.get_param("move"),Bernard,queue_size=10).publish(bernard)

    def update_joint_state(self,joint_name,position):
        self.joints[joint_name]=position

    def joint_state_callback(self,msg):
        self.lock.acquire()
        joint_names=msg.name
        positions=msg.position
        count=0
        for joint_name in joint_names:
            position=positions[count]
            degrees=self.convert_radians_to_degrees(position)
            self.update_joint_state(joint_name,degrees)
            count=count+1
        self.publish_joint()
        self.lock.release()

    def convert_radians_to_degrees(self,radians):
        degrees=0
        if radians:
            degrees=float(radians)*(180/3.14)
        return abs(degrees)

    def publish_joint(self):
        for joint,position in self.joints.iteritems():
            if position!=0.0:
                print joint,position
                port=rospy.get_param(joint)
                bernard=self.initialize_bernard(int(port),int((position)))
                self.publish_data(bernard)

if __name__=="__main__":
    bernard=BernadController()
