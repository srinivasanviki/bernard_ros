#!/usr/bin/env python

import speech_recognition as sr
import rospy
from hark_msgs.msg import HarkSource
import apiai
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numbers
from Queue import Queue
import threading
STORAGE_PATH= "/home/viki/Desktop/test_hark/audio"
CREATION_PATH="/home/viki/Desktop/bernado_ros/bernado_tools/audio"
Extraction_PATH="/home/viki/Desktop/bernado_ros/bernado_tools/extracted_path"
filter_timestamp=[]
already_greeted=False
buffer_queue = Queue()
class ReadInput():
    def __init__(self):
        rospy.init_node('api_ai', anonymous=True)
        self.seconds=0
        self.buffer={}
        self.speech_recognizer=sr.Recognizer()
        self.speech_recognizer.energy_threshold=3000
        audio_thread=threading.Thread(target=self.process_audio)
        audio_thread.start()
        self.pub=rospy.Publisher('joint_states', JointState, queue_size=10)
        self.sound_localization=rospy.Subscriber("HarkSource",HarkSource, self.read_localization_results)
        rospy.spin()

    def read_api_ai(self,speech_text):
        api_token=rospy.get_param("API_AI_TOKEN")
        self.ai = apiai.ApiAI(api_token)
        self.request = self.ai.text_request()
        self.request.query = speech_text
        response = self.request.getresponse()
        return response.read()

    def convert_wav_text(self,file_name):
        with sr.WavFile("%s/%s"%(CREATION_PATH,file_name)) as source:
            audio = self.speech_recognizer.record(source)
        return self.speech_recognizer.recognize_google(audio)

    def convert_to_mono(self):
        import os
        from subprocess import call
        dirs = os.listdir(Extraction_PATH)
        if len(dirs)>0:
            call(["sox", "%s/%s"%(Extraction_PATH,dirs[0]), "-c 2","%s/%s"%(CREATION_PATH,dirs[0])])

    def greeted_bernard(self):
        import os
        dirs = os.listdir(CREATION_PATH)
        try:
            for file_name in dirs:
                if "sep" in file_name:
                    text=self.convert_wav_text(file_name)
                    response=self.read_api_ai(text)
                    if "Bernard" in text:
                        return response,True
                return response,False
        except sr.UnknownValueError as e:
            e.message

    def clear_buffer(self):
        for timestamp in filter_timestamp:
            del self.buffer[timestamp]

    def publish_audio(self,response):
        from sound_play.libsoundplay import SoundClient
        self.soundhandle = SoundClient()
        rospy.sleep(1)
        self.soundhandle.say(response)

    def process_audio(self):
        import math
        try:
            while True:
                if len(self.buffer) > 0:
                    for timestamp,audio_buffer in self.buffer.items():
                            self.extract_audio(timestamp)
                            self.convert_to_mono()
                            response,greeted=self.greeted_bernard()
                            filter_timestamp.append(timestamp)
                            if greeted:
                                already_greeted=True
                                if len(audio_buffer)>0:
                                   self.publish_joint_states(math.radians(max(audio_buffer)))
                                   self.clear_buffer()
                            if already_greeted:
                                self.publish_audio(response)
        except Exception as e:
            e.message

    def read_localization_results(self,msg):
        timestamp=msg.header.stamp.secs
        audio_buffer=[]
        for src in msg.src:
            azimuth=src.azimuth
            audio_buffer.append(90-azimuth)
        self.buffer[timestamp]=audio_buffer


    def publish_joint_states(self,goal):
        import math
        rate = rospy.Rate(10)
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['head_leftright']
        joint_state.position = [math.radians(int(goal))]
        joint_state.velocity = []
        joint_state.effort = []
        self.pub.publish(joint_state)
        rate.sleep()


    def extract_audio(self,sec):
        from pydub import AudioSegment
        import os
        import datetime
        files = os.listdir(STORAGE_PATH)
        for file in files:
            if "sep" in file:
                name=os.path.splitext(file)[0].split("_")[1].split("-")
                date="%s %s"%(name[0],name[1])
                time=datetime.datetime.strptime(date, '%Y%m%d %H%M%S')
                time_source=datetime.datetime.fromtimestamp(sec)
                elapsed=time_source-time
                newAudio = AudioSegment.from_wav("%s/%s"%(STORAGE_PATH,file))
                duration=int(newAudio.duration_seconds)*1000
                source_duration=abs( elapsed.total_seconds())
                source_duration=int(source_duration)*1000
                newAudio = newAudio[source_duration:duration]
                newAudio.export('%s/%s'%(Extraction_PATH,file), format="wav")


if __name__=="__main__":
    speech_transform=ReadInput()