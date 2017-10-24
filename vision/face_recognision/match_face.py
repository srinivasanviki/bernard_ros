
import os
import face_recognition

FACE_PATH='/home/viki/Desktop/bernado_ros/vision/faces'

class facerecognition():
      def __init__(self):
          self.known_encodings=[]
      def load_image_encodings(self):
          dirs = os.listdir(FACE_PATH)
          for file in dirs:
              face_path="%s/%s"%(FACE_PATH,file)
              known_images=face_recognition.load_image_file(face_path)
              face_encoding=face_recognition.face_encodings(known_images)[0]
              self.known_encodings.append(face_encoding)

      def test_image_encoding(self,frame):
          self.load_image_encodings()
          face_locations = face_recognition.face_locations(frame)
          face_encodings = face_recognition.face_encodings(frame, face_locations)

          for encoding in face_encodings:
              face_distances = face_recognition.face_distance(self.known_encodings, encoding)

              if len(face_distances) >0:
                  for i, face_distance in enumerate(face_distances):
                      if face_distance <0.5:
                          return True
                      else:
                          return False
              return False
