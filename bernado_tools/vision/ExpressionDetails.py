import cv2
from keras.models import load_model
import numpy as np

from datasets import get_labels
from inference import detect_faces, apply_offsets,load_detection_model

from preprocessor import preprocess_input


#[0:'angry',1:'disgust',2:'fear',3:'happy',4:'sad',5:'surprise',6:'neutral']

# parameters for loading data and images
def ExpressionDetection (image):
    detection_model_path = 'src/get_image/doc/haarcascade_frontalface_default.xml'


    emotion_model_path = 'src/get_image/models/fer2013_mini_XCEPTION.102-0.66.hdf5'
    emotion_labels = get_labels('fer2013')

    # loading models
    face_detection = load_detection_model(detection_model_path)

    emotion_classifier = load_model(emotion_model_path, compile=False)

    # getting input model shapes for inference
    emotion_target_size = emotion_classifier.input_shape[1:3]
    emotion_offsets = (20, 40)
    #read image


    # print 'dddddddd',gray_image
    #gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    faces = detect_faces(face_detection, image)
    # print 'qqqqqqqq',faces

    for face_coordinates in faces:

        x1, x2, y1, y2 = apply_offsets(face_coordinates, emotion_offsets)
        gray_face = image[y1:y2, x1:x2]
        try:
            gray_face = cv2.resize(gray_face, (emotion_target_size))
        except:
            continue

        gray_face = preprocess_input(gray_face, True)
        # print 'eeeeee',gray_face
        gray_face = np.expand_dims(gray_face, 0)
        gray_face = np.expand_dims(gray_face, -1)
        emotion_prediction = emotion_classifier.predict(gray_face)
        emotion_probability = np.max(emotion_prediction)
        emotion_label_arg = np.argmax(emotion_prediction)
        emotion_text = emotion_labels[emotion_label_arg]
        # print 'the emotion probability is:',emotion_probability
        # print 'the emotion is:',emotion_label_arg

        # print 'the predicted emotion is:',emotion_text
        return emotion_text








