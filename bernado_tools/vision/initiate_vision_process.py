#!/usr/bin/env python
import rospy
import rospkg
import roslib
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from cv_bridge import CvBridge, CvBridgeError
import os
import face_recognition
# from age_gender_check import agegender
# from ExpressionDetails import ExpressionDetection



FACE_PATH='/home/viki/Desktop/bernado_ros/bernado_tools/vision/faces'

class vision():
    def __init__(self):
        rospy.init_node('vision')
        self.rate = rospy.Rate(30)
        self.bridge = CvBridge()

        self.pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)
        # self.sub=rospy.subscriber('/initiate_vision',String )
        self.sub=rospy.Subscriber('/camera/image_raw',Image, self.read_image_snapshot)
        cascPath = "/home/viki/Webcam-Face-Detect/haarcascade_frontalface_default.xml"
        self.faceCascade = cv2.CascadeClassifier(cascPath)
        self.known_encodings={}
        rospy.spin()

    def read_image_snapshot(self,msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')


            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            cv2.imshow("Image window",gray)

            faces = (self.faceCascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30),
                flags = cv2.cv.CV_HAAR_SCALE_IMAGE)
            )
            # print faces
            # Draw a rectangle around the
            for (x, y, w, h) in faces:
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                sub_face = cv_image[y:y+h, x:x+w]
                has_matched,name=self.match_face(sub_face)
                print has_matched,name
                if has_matched:
                    print ("Matched")
                else:
                    cv2.imwrite("%s/%s_%s.jpg"%(FACE_PATH,"face","vignesh"),cv_image)

            # cv2.imshow('image', cv_image)
            # self.pub.publish(self.bridge.cv2_to_imgmsg(cv_image,'bgr8'))
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
        print("Into match face")
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
