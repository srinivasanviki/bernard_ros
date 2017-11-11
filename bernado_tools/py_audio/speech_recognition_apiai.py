#!/usr/bin/env python

import rospy
import apiai

class RecognizeSpeech():

    def read_api_ai(self,speech_text):
        api_token=rospy.get_param("API_AI_TOKEN")
        self.ai = apiai.ApiAI(api_token)
        self.request = self.ai.text_request()
        self.request.query = speech_text
        response = self.request.getresponse()
        return response.read()

    def convert_text_speech(self,text):
        from sound_play.libsoundplay import SoundClient
        soundhandle = SoundClient()
        rospy.sleep(1)
        soundhandle.say(text)
        rospy.sleep(1)

