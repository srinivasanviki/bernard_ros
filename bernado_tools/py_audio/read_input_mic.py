#!/usr/bin/env python

import speech_recognition as sr
import rospy
# from hark_msgs.msg import HarkSource
import apiai
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Header
import numbers
from Queue import Queue
import threading
from speech_recognition_apiai import RecognizeSpeech

import sys



STORAGE_PATH= "/home/viki/Desktop/test_hark/audio"
CREATION_PATH="/home/viki/Desktop/bernado_ros/bernado_tools/audio"
Extraction_PATH="/home/viki/Desktop/bernado_ros/bernado_tools/extracted_path"

filter_timestamp=[]



buffer_queue = Queue()
class ReadInput():
    def __init__(self):
        rospy.init_node('speech', anonymous=True)
        self.seconds=0
        self.buffer={}
        self.already_greeted=False
        self.bernard_greeted=False
        self.speech_recognizer=sr.Recognizer()
        self.speech_recognizer.energy_threshold=3000
        # audio_thread=threading.Thread(target=self.process_audio)
        self.bernard_initialized=False
        self.speech_rec=RecognizeSpeech()
        self.pub=rospy.Publisher('joint_states', JointState, queue_size=10)
        self.initiate_vision=rospy.Publisher('/initiate_vision/vision', String, queue_size=10)
        self.name_initialization=False
        self.read_speech_mic()
        # self.sound_localization=rospy.Subscriber("HarkSource",HarkSource, self.read_localization_results)

        # audio_thread.start()
        rospy.spin()


    def parse_audio(self,recognizer,audio):
        try:
            text=recognizer.recognize_sphinx(audio)
            self.greeted_bernard(text)
        except sr.UnknownValueError:
             print("Sorry sir, but, I could not understand what you said!")

    def read_speech_mic(self):
        import pyaudio
        import speech_recognition as sr
        with sr.Microphone(0) as source:
            while 1:
                try:
                    audio = self.speech_recognizer.listen(source)
                    text=self.speech_recognizer.recognize_google(audio)
                    self.greeted_bernard(text)
                except LookupError as e:
                    print("Could not understand audio")
                except sr.UnknownValueError:
                    print("Sorry sir, but, I could not understand what you said!")



    # def convert_wav_text(self,file_name):
    #     import httplib
    #     import time
    #     try:
    #         with sr.WavFile("%s/%s"%(CREATION_PATH,file_name)) as source:
    #             audio = self.speech_recognizer.record(source)
    #         return self.speech_recognizer.recognize_google(audio)
    #     except httplib.HTTPException as e:
    #            print ("Exception")
    #
    #
    # def convert_to_mono(self):
    #     import os
    #     from subprocess import call
    #     dirs = os.listdir(Extraction_PATH)
    #     if len(dirs)>0:
    #         call(["sox", "%s/%s"%(Extraction_PATH,dirs[0]), "-c 2","%s/%s"%(CREATION_PATH,dirs[0])])

    def greeted_bernard(self,text):
        import os
        import json
        dirs = os.listdir(CREATION_PATH)
        # try:
        #     for file_name in dirs:
        #         if "sep" in file_name:
        # text=self.convert_wav_text(file_name)
        print text
        response=self.speech_rec.read_api_ai(text)
        print response
        speech_resp=json.loads(response)
        speech=speech_resp.get("result").get("fulfillment").get("speech")

        if "my name" in text:
            print text
            name=text.split("is")[1]
            print name
            self.initiate_vision.publish(name)

        if self.bernard_initialized:
            return speech,False
        else:
            if "Bernard" in text:
                self.bernard_initialized=True
                print("Initialized")
                self.initiate_vision.publish("Start Recognition")


    # def clear_buffer(self):
    #     for timestamp in filter_timestamp:
    #         if self.buffer.get(timestamp):
    #             del self.buffer[timestamp]
    #
    # def remove_file(self):
    #     import os
    #     dirs = os.listdir(CREATION_PATH)
    #     for file in dirs:
    #         os.remove("%s/%s"%(CREATION_PATH,file))

    # def process_audio(self):
    #     while True:
    #         for timestamp,audio_buffer in self.buffer.copy().items():
    #             self.extract_audio(timestamp)
    #             self.convert_to_mono()
    #             response,greeted=self.greeted_bernard()
    #             filter_timestamp.append(timestamp)
    #             self.remove_file()
    #             # if greeted:
    #             #     self.already_greeted=True
    #             #     if len(audio_buffer)>0:
    #             #         self.publish_joint_states(math.radians(max(audio_buffer)),"head_leftright")
    #             #         self.clear_buffer()
    #             #     if self.already_greeted:
    #             #         self.recognize.convert_text_speech(response)
    #             if response != None:
    #                 self.speech_rec.convert_text_speech(response)
    #                 self.publish_joint_states(40,"jaw")
    #
    # def read_localization_results(self,msg):
    #     timestamp=msg.header.stamp.secs
    #     audio_buffer=[]
    #     for src in msg.src:
    #         azimuth=src.azimuth
    #         audio_buffer.append(90-azimuth)
    #     self.buffer[timestamp]=audio_buffer


    def publish_joint_states(self,goal,part):
        import math
        rate = rospy.Rate(10)
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = [part]
        joint_state.position = [math.radians(int(goal))]
        joint_state.velocity = []
        joint_state.effort = []
        self.pub.publish(joint_state)
        rate.sleep()

    # def extract_audio(self,sec):
    #     from pydub import AudioSegment
    #     import os
    #     import datetime
    #     files = os.listdir(STORAGE_PATH)
    #     for file in files:
    #         if "sep" in file:
    #             name=os.path.splitext(file)[0].split("_")[1].split("-")
    #             date="%s %s"%(name[0],name[1])
    #             time=datetime.datetime.strptime(date, '%Y%m%d %H%M%S')
    #             time_source=datetime.datetime.fromtimestamp(sec)
    #             elapsed=time_source-time
    #             newAudio = AudioSegment.from_wav("%s/%s"%(STORAGE_PATH,file))
    #             duration=int(newAudio.duration_seconds)*1000
    #             source_duration=abs(elapsed.total_seconds())
    #             source_duration=int(source_duration)*1000
    #             newAudio = newAudio[source_duration:duration]
    #
    #             newAudio.export('%s/%s'%(Extraction_PATH,file), format="wav")

if __name__=="__main__":
    speech_transform=ReadInput()