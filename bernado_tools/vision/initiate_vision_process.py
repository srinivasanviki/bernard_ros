#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
# from sensor_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os
import face_recognition
from speech_recognition_apiai import RecognizeSpeech
# from age_gender_check import agegender
from std_msgs.msg import String

# from ExpressionDetails import ExpressionDetection

FACE_PATH='/home/viki/Desktop/bernado_ros/bernado_tools/vision/faces'
PATH='/home/viki/Desktop/bernado_ros/bernado_tools'

class vision():
    def __init__(self):
        rospy.init_node('vision', anonymous=True)
        self.rate = rospy.Rate(30)
        self.bridge = CvBridge()
        self.known_encodings={}
        self.name=None
        self.already_greeted=False
        self.count=0
        self.speech_rec=RecognizeSpeech()
        cascPath = "%s/%s/haarcascade_frontalface_default.xml"%(PATH,"haar_cascade")
        self.faceCascade = cv2.CascadeClassifier(cascPath)
        self.pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)
        self.sub=rospy.Subscriber('/initiate_vision/vision',String, self.process_vision)
        self.start_processing=False
        self.processed_msg=None
        self.matched_faces={}
        rospy.spin()

    def process_vision(self,msg):
        msg=msg.data
        if msg=="Start Recognition":
            # self.sub=rospy.Subscriber('/kinect2/qhd/image_color_rect',Image, self.read_image_snapshot)
            self.read_image_snapshot()
        else:
            print "Inside vision again %s"%(msg)
            self.processed_msg=msg
            cv2.imwrite("%s/face_%s.jpg"%(FACE_PATH,self.processed_msg),self.sub_face)

    def read_image_snapshot(self ):
        try:
            print("reading image")
            cam = cv2.VideoCapture(0)

            while(True):
                s, cv_image = cam.read()
                # cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                # cv2.imshow("Image window",gray)
                faces = (self.faceCascade.detectMultiScale(
                    gray,
                    scaleFactor=1.1,
                    minNeighbors=5,
                    minSize=(50, 50),
                    flags = cv2.CASCADE_SCALE_IMAGE)
                )
                # print faces
                # Draw a rectangle around the
                if len(faces)>0:
                    for (x, y, w, h) in faces:
                        cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                        self.sub_face = cv_image[y:y+h, x:x+w]
                        has_matched,name=self.match_face(self.sub_face)
                        if has_matched:
                            name=name.split(".jpg")[0]
                            # age_parameter=agegender("%s/%s_%s"%(FACE_PATH,"face",name))
                            # expression_parameter=ExpressionDetection("%s/%s_%s"%(FACE_PATH,"face",name))
                            # gender=age_parameter[1]
                            # expression=expression_parameter
                            # if self.processed_msg=="process_gender":
                            #     self.speech_rec.convert_text_speech("Your are %s"%(gender))
                            if not self.matched_faces.get(name):
                               self.speech_rec.convert_text_speech("Hello %s"%(name))
                            self.matched_faces[name]="True"
                        else:
                            self.speech_rec.convert_text_speech("Hello May i know ur name")
                self.rate.sleep()
                cv2.waitKey(0)
        except CvBridgeError, e:
                print e


    def check_image(self):
        dirs = os.listdir(FACE_PATH)
        for file in dirs:
            face_path="%s/%s"%(FACE_PATH,file)
            known_images=face_recognition.load_image_file(face_path)
            face_encoding=face_recognition.face_encodings(known_images)[0]
            self.known_encodings[file.split("_")[1]]=face_encoding

    def match_face(self,img):
        self.check_image()
        face_locations = face_recognition.face_locations(img)
        face_encodings = face_recognition.face_encodings(img, face_locations)
        names=self.known_encodings.keys()
        values=self.known_encodings.values()
        for encoding in face_encodings:
            face_distances = face_recognition.face_distance(values, encoding)
            if len(face_distances) >0:
                for i, face_distance in enumerate(face_distances):
                    if face_distance <0.5:
                         return True ,names[i]
        return False,None



if __name__ == '__main__':
    import threading
    try:
       vision()
    except rospy.ROSInterruptException:
        pass
