import os
import numpy as np
import matplotlib.pyplot as plt
import sys
import cv2

#sys.path.append('/home/caffe/python')

# caffe_root = '/home/caffe/'
# import sys
# sys.path.append(caffe_root + 'python')
import caffe

PATH='/home/viki/Desktop/bernado_ros/bernado_tools'

def agegender(example_image):
    plt.rcParams['figure.figsize'] = (10, 10)
    plt.rcParams['image.interpolation'] = 'nearest'
    plt.rcParams['image.cmap'] = 'gray'

    mean_filename='%s/vision/models/mean.binaryproto'%(PATH)
    proto_data = open(mean_filename, "rb").read()
    a = caffe.io.caffe_pb2.BlobProto.FromString(proto_data)
    mean  = caffe.io.blobproto_to_array(a)[0]

    age_net_pretrained='%s/vision/models/age_net.caffemodel'%(PATH)
    age_net_model_file='%s/vision/doc/agedeploy.prototxt'%(PATH)
    age_net = caffe.Classifier(age_net_model_file, age_net_pretrained,
                               mean=mean,
                               channel_swap=(2,1,0),
                               raw_scale=255,
                               image_dims=(256, 256))


    gender_net_pretrained='%s/vision/models/gender_net.caffemodel'%(PATH)
    gender_net_model_file='%s/vision/doc/genderdeploy.prototxt'%(PATH)
    gender_net = caffe.Classifier(gender_net_model_file, gender_net_pretrained,
                                  mean=mean,
                                  channel_swap=(2,1,0),
                                  raw_scale=255,
                                  image_dims=(256, 256))

    age_list=['(0, 2)','(4, 6)','(8, 12)','(15, 20)','(25, 32)','(38, 43)','(48, 53)','(60, 100)']
    gender_list=['Male','Female']


    input_image = caffe.io.load_image(example_image)
    _ = plt.imshow(input_image)
    image1 = cv2.imread(example_image)
    image = cv2.resize(image1,(48,48))
    image1 = cv2.resize(image1,(1000,700))
    pixels_dict = {}
    pixels_list = []
    for i in range(48):
        # print image[i,i]
        pixels_list.append(image[i,i])
    pixels_dict['pixels']= pixels_list
    # print 'pixels are:',pixels_dict['pixels']

    prediction = age_net.predict([input_image])

    # print 'predicted age:', age_list[prediction[0].argmax()]

    prediction = gender_net.predict([input_image])

    # print 'predicted gender:', gender_list[prediction[0].argmax()]

    return age_list[prediction[0].argmax()],gender_list[prediction[0].argmax()]
    # faceCascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    # gray = cv2.cvtColor(image1,cv2.COLOR_BGR2GRAY)
    # faces = faceCascade.detectMultiScale(
    #     gray,1.05,2,cv2.CASCADE_SCALE_IMAGE,(100,100)
    # )
    # for (x,y,w,h) in faces:
    #     cv2.rectangle(image1,(x,y),(x+w,y+h),(0,255,0),2)
    #
    # cv2.putText(image1,age_list[prediction[0].argmax()],(50,600),cv2.FONT_HERSHEY_SIMPLEX,2,(0,0,0),2,cv2.LINE_AA)
    # cv2.putText(image1,gender_list[prediction[0].argmax()],(100,500),cv2.FONT_HERSHEY_SIMPLEX,2,(0,0,0),2,cv2.LINE_AA)
    # cv2.putText(image1,expressionResult,(100,400),cv2.FONT_HERSHEY_SIMPLEX,2,(0,0,0),2,cv2.LINE_AA)
    #
    #
    # cv2.imshow("Result",image1)
    # cv2.waitKey(0)

    def showimage(im):
        if im.ndim == 3:
            im = im[:, :, ::-1]
        plt.set_cmap('jet')
        plt.imshow(im, vmin=0, vmax=0.3)


    def vis_square(data, padsize=1, padval=0):
        data -= data.min()
        data /= data.max()

        # force the number of filters to be square
        n = int(np.ceil(np.sqrt(data.shape[0])))
        padding = ((0, n ** 2 - data.shape[0]), (0, padsize), (0, padsize)) + ((0, 0),) * (data.ndim - 3)
        data = np.pad(data, padding, mode='constant', constant_values=(padval, padval))

        # tile the filters into an image
        data = data.reshape((n, n) + data.shape[1:]).transpose((0, 2, 1, 3) + tuple(range(4, data.ndim + 1)))
        data = data.reshape((n * data.shape[1], n * data.shape[3]) + data.shape[4:])

        showimage(data)

    _ = plt.imshow(input_image)

    filters = age_net.params['conv1'][0].data[:49]
    vis_square(filters.transpose(0, 2, 3, 1))

    feat = age_net.blobs['conv1'].data[0, :49]
    vis_square(feat, padval=1)
