import cv2
import sys
import Image
import cv
from match_face import facerecognition

class crop_image():
    def __init__(self):
        self.cascPath="/home/viki/Webcam-Face-Detect/haarcascade_frontalface_default.xml"
        self.face_rec=facerecognition()

    def capture_check(self):

        faceCascade = cv2.CascadeClassifier(self.cascPath)

        video_capture = cv2.VideoCapture(0)

        while True:
            ret, frame = video_capture.read()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            faces = faceCascade.detectMultiScale(
                gray,
                scaleFactor=1.1,
                minNeighbors=5,
                minSize=(30, 30),
                flags=cv2.cv.CV_HAAR_SCALE_IMAGE
            )
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                sub_face = frame[y:y+h, x:x+w]
                check_face=self.face_rec.test_image_encoding(sub_face)
                if check_face == False:

                    FaceFileName = "/home/viki/Desktop/bernado_ros/vision/faces/face_" + str(y) + ".jpg"
                    cv2.imwrite(FaceFileName, sub_face)
                break
            cv2.imshow('Video', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        video_capture.release()
        cv2.destroyAllWindows()


crop_image().capture_check()