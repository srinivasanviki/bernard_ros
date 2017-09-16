#!/usr/bin/env python

import speech_recognition as sr
import rospy
import threading
from std_msgs.msg import String



class ReadInput():
    def __init__(self):
        rospy.init_node('api_ai', anonymous=True)
        self.speech_recognizer=sr.Recognizer()
        self.speech_recognizer.energy_threshold=4000
        self.speech_publisher=rospy.Publisher("speech",String, queue_size=10)
        self.thread = threading.Thread(target=self.run_background)
        self.thread.start()

    def get_text(self,recognizer, audio):
        try:
            speech_text=recognizer.recognize_google(audio)
            if speech_text:
               self.speech_publisher.publish(str(speech_text)) # call api ai for speech to text
        except sr.UnknownValueError:
            print("Oops! Didn't catch that")

    def run_background(self):
        self.speech_recognizer.listen_in_background(sr.Microphone(),self.get_text)
        rospy.spin()

if __name__=="__main__":
    speech_transform=ReadInput()

