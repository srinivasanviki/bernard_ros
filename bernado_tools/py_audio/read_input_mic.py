#!/usr/bin/env python

import speech_recognition as sr
import rospy
from hark_msgs.msg import HarkSource
import apiai
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import numbers

STORAGE_PATH="/home/viki/Desktop/bernado_ros/bernado_tools/wavfiles"
CREATION_PATH="/home/viki/Desktop/bernado_ros/bernado_tools/py_audio"
Extraction_PATH="/home/viki/Desktop/bernado_ros/bernado_tools/extracted_path"


buffer=dict()

class ReadInput():
    def __init__(self):
        rospy.init_node('api_ai', anonymous=True)
        self.seconds=0
        self.speech_recognizer=sr.Recognizer()
        self.speech_recognizer.energy_threshold=3000
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

    def greeted_bernard(self):
        import os
        dirs = os.listdir(CREATION_PATH)
        try:
            for file_name in dirs:
                text=self.convert_wav_text(file_name)
                if "Bernard" in text:
                    return True
                return False

        except sr.UnknownValueError:
            print("Oops! Didn't catch that")

    def convert_files_to_mono(self):
        import os
        from subprocess import call
        dirs = os.listdir(Extraction_PATH)
        for file in dirs:
            call(["sox", "%s/%s"%(Extraction_PATH,file), "-c 2","%s/%s"%(CREATION_PATH,file)])

    def read_localization_results(self,msg):
        buffer[msg.header.stamp.secs]={}
        azimuth=msg.src[0].azimuth
        elevation=msg.src[0].elevation
        if azimuth > 0:
            azimuth=90-azimuth
        self.extract_audio(msg.header.stamp.secs)
        self.convert_files_to_mono()
        greeted=self.greeted_bernard()
        if greeted:
            self.publish_joint_states(azimuth)

    def publish_joint_states(self,goal):
        import math
        pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rate = rospy.Rate(10) # 10hz
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = ['head_leftright']
        joint_state.position = [math.radians(int(goal))]
        joint_state.velocity = []
        joint_state.effort = []
        pub.publish(joint_state)
        rate.sleep()


    def extract_audio(self,sec):
        from pydub import AudioSegment
        import os
        import datetime
        file = os.listdir(STORAGE_PATH)[0]
        name=os.path.splitext(file)[0].split("_")[1].split("-")
        date="%s %s"%(name[0],name[1])
        time=datetime.datetime.strptime(date, '%Y%m%d %H%M%S')
        time_source=datetime.datetime.fromtimestamp(sec)
        elapsed=time-time_source
        newAudio = AudioSegment.from_wav("%s/%s"%(STORAGE_PATH,file))
        duration=int(newAudio.duration_seconds*1000)
        source_duration=abs(int(elapsed.total_seconds())*1000)
        newAudio = newAudio[source_duration:duration]
        newAudio.export('%s/%s'%(Extraction_PATH,file), format="wav")



if __name__=="__main__":
    speech_transform=ReadInput()

