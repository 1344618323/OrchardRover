#!/usr/bin/env python
# coding:utf-8
import rospy
import cv2
import os
import sys
import numpy as np
import math
import os.path as osp
import trunk_detection_initpath
import caffe
from fast_rcnn.test import im_detect
from fast_rcnn.nms_wrapper import nms
from fast_rcnn.config import cfg
from utils.timer import Timer
import matplotlib.pyplot as plt
from or_msgs.msg import TrunkObsMsg

from sensor_msgs.msg import Image
import cv_bridge
import copy


CLASSES = ('__background__',
           'tree', 'pine_tree')

CAMERAFdSx = 752.9652
CAMERAux = 308.3392

# CAMERAFdSx= 1435.2
# CAMERAux= 595.4


def vis_detections(im, class_name, dets, thresh=0.9):
    """Draw detected bounding boxes."""
    inds = np.where(dets[:, -1] >= thresh)[0]
    leftbox = 1000
    k = 0
    if len(inds) == 0:
        return
    bbox = []
    for i in inds:
        bbox.append(dets[i, :4])
        score = dets[i, -1]
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(im, '{}, {:.3f}'.format(class_name, score),
                    (int(bbox[i][0]), int(bbox[i][3])), font, 1, (0, 255, 0), 2)
        cv2.rectangle(im, (int(bbox[i][0]), int(bbox[i][3])), (int(
            bbox[i][2]), int(bbox[i][1])), (0, 255, 0), 5)
    for i in range(len(bbox)):
        if (bbox[i][0] < leftbox):
            leftbox = bbox[i][0]
            k = i
    print("bbox[k]:", bbox[k])
    if(bbox[k][0] > 1500 and class_name == 'pine_tree'):
        cv2.rectangle(im, (int(bbox[k][0]), int(bbox[k][3])), (int(
            bbox[k][2]), int(bbox[k][1])), (0, 0, 255), 5)
    cv2.imshow("im", im)


def detecte_and_draw(net, im, str):
    """Detect object classes in an image using pre-computed object proposals."""

    # Detect all object classes and regress object bounds
    # timer = Timer()
    # timer.tic()
    scores, boxes = im_detect(net, im)
    # timer.toc()
    # print ('Detection took {:.3f}s for '
    #        '{:d} object proposals').format(timer.total_time, boxes.shape[0])

    # Visualize detections for each class
    CONF_THRESH = 0.8
    NMS_THRESH = 0.3
    clsdict = dict()
    for cls_ind, cls in enumerate(CLASSES[1:]):
        cls_ind += 1  # because we skipped background
        cls_boxes = boxes[:, 4*cls_ind:4*(cls_ind + 1)]
        cls_scores = scores[:, cls_ind]
        dets = np.hstack((cls_boxes,
                          cls_scores[:, np.newaxis])).astype(np.float32)
        keep = nms(dets, NMS_THRESH)  # 目测是把对同一个目标识别处理的矩形做融合
        dets = dets[keep, :]  # 最终剩下的矩形
        # vis_detections(im, cls, dets, thresh=CONF_THRESH)
        clsdict[cls] = dets

    # 画矩形,只要是树干就行
    """Draw detected bounding boxes."""
    pubbox = []
    for cls in clsdict:
        inds = np.where(clsdict[cls][:, -1] >= CONF_THRESH)[0]
        for i in inds:
            pubbox.append(clsdict[cls][i, :4])
    for enumpubbox in pubbox:
        cv2.rectangle(im, (int(enumpubbox[0]), int(enumpubbox[3])), (int(
            enumpubbox[2]), int(enumpubbox[1])), (0, 255, 0), 5)
    cv2.imshow(str, im)
    return pubbox


cfg.TEST.HAS_RPN = True  # Use RPN for proposals


class DetectTrunk:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(
            'hk_image', Image, self.image_callback)
        self.procimg = []

    def image_callback(self, msg):
        self.procimg = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


class ObserveMsg:
    def __init__(self):
        self.cur_angle = [0, 0]
        self.obser_admin = [1, 1]
        self.observe_sub = rospy.Subscriber(
            'trunk_ultrasonic_obs', TrunkObsMsg, self.observe_callback, queue_size=100)

    def observe_callback(self, msg):
        for i in range(2):
            if msg.valids[i] >= 1:
                self.cur_angle[i] = msg.bearings[i]
                self.obser_admin[i] = 1


if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)

    # 加载 faster-RCNN 模型
    prototxt = '/home/cxn/myfile/py-faster-rcnn/models/pascal_voc/VGG16/faster_rcnn_alt_opt/faster_rcnn_test_cxn.pt'
    caffemodel = '/home/cxn/myfile/py-faster-rcnn/data/faster_rcnn_models/vgg16_faster_rcnn_iter_4000.caffemodel'
    if not osp.isfile(caffemodel):
        raise IOError(('{:s} not found.').format(caffemodel))
    caffe.set_mode_gpu()
    caffe.set_device(0)
    net = caffe.Net(prototxt, caffemodel, caffe.TEST)
    print '\n\nLoaded network {:s}'.format(caffemodel)

    ##################用于雷达####################

    # detect_trunk = DetectTrunk()
    # angle_pub = rospy.Publisher(
    #     'trunk_obs', TrunkObsMsg, queue_size=10)

    # while not rospy.is_shutdown():
    #     if detect_trunk.procimg == []:
    #         continue
    #     out = copy.deepcopy(detect_trunk.procimg)
    #     # cv2.imshow('frame',out)
    #     boxes = detecte_and_draw(net, out)
    #     # if boxes != []:
    #     #     angle = []
    #     #     for box in boxes:
    #     #         angle.append(math.atan2(
    #     #             (intrinsicMatrix[0][2]-box[2]), intrinsicMatrix[0][0])*180/math.pi)
    #     #         angle.append(math.atan2(
    #     #             (intrinsicMatrix[0][2]-box[0]), intrinsicMatrix[0][0])*180/math.pi)
    #     #         boxwidth = box[2]-box[0]
    #     #         angle.append(intrinsicMatrix[0][0]*0.2/boxwidth)
    #     #     angle_pub.publish(TrunkObsMsg(None, angle))
    #     cv2.waitKey(5)
    # cv2.destroyAllWindows()
    # rospy.spin()

    ###########读yaml文件##########
    # fs=cv2.FileStorage('/home/cxn/myfile/orchardrover_ws/src/OrchardRover/or_detection/camparams/LongFCamCalib.yaml',cv2.FileStorage_READ)
    # intrinsicMatrix=fs.getNode('intrinsicMatrix').mat()
    # distCoeffs=fs.getNode('distCoeffs').mat()
    # fs.release()
    # print intrinsicMatrix
    # print distCoeffs
    #    cap = cv2.VideoCapture(1)
    #  if cap.isOpened():
    #   cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
    #      cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
    #     ret, im = cap.read()
    # if ret==True:
    #     # out=np.zeros_like(im)
    #     # cv2.undistort(im,intrinsicMatrix,distCoeffs,out)
    #     out=im
    #     # cv2.imshow('frame',out)

    ###################用于超声波####################
    angle_pub = rospy.Publisher(
        'ultrasonic_cam', TrunkObsMsg, queue_size=100)

    observe_msg = ObserveMsg()

    # cap = cv2.VideoCapture(
    #     '/home/cxn/myfile/py-faster-rcnn/data/demo/beisu3.mp4')
    # cap2 = cv2.VideoCapture(
    #     '/home/cxn/myfile/py-faster-rcnn/data/demo/beisu3.mp4')
    cap = cv2.VideoCapture(0)
    cap2 = cv2.VideoCapture(1)
    while cap.isOpened() and \
            cap2.isOpened() and\
            not rospy.is_shutdown():

        valids = [0, 0]
        bearings = [0, 0]

        ret, im = cap.read()
        if ret == True and observe_msg.obser_admin[0] == 1:
            out = im
            out = cv2.resize(im, (640, 480))
            boxes = detecte_and_draw(net, out, "im1")
            if boxes != []:
                minabsangle = 180
                minangle = 0
                for box in boxes:
                    angle = 0.0935*(box[2]+box[0])/2-31.7933
                    if (math.fabs(angle) < minabsangle) and\
                        (observe_msg.cur_angle[0]+angle < 90) and\
                            (observe_msg.cur_angle[0]+angle > -90):
                        minangle = angle
                        minabsangle = math.fabs(angle)
                if(minabsangle != 180):
                    valids[0] = 2
                    bearings[0] = -minangle
                    observe_msg.obser_admin[0] = 0

        ret, im = cap2.read()
        if ret == True and observe_msg.obser_admin[1] == 1:
            out = im
            out = cv2.resize(im, (640, 480))
            boxes = detecte_and_draw(net, out, "im2")
            if boxes != []:

                minabsangle = 180
                minangle = 0
                for box in boxes:
                    # angle = 0.0935*(box[2]+box[0])/2-34.0377
                    angle = math.atan2((box[2]+box[0])/2-314, 357)*180/math.pi
                    if math.fabs(angle) < minabsangle and\
                        (observe_msg.cur_angle[1]+angle < 90) and\
                            (observe_msg.cur_angle[1]+angle > -90):
                        minangle = angle
                        minabsangle = math.fabs(angle)
                if(minabsangle != 180):
                    valids[1] = 2
                    bearings[1] = -minangle
                    observe_msg.obser_admin[1] = 0
                    angle_pub.publish(TrunkObsMsg( None, valids, bearings, None))

        cv2.waitKey(1)
    cap.release()
    cv2.destroyAllWindows()

    # im_names = ['/home/cxn/myfile/py-faster-rcnn/data/demo/IMG_20190521_171646.jpg',
    #     '/home/cxn/myfile/py-faster-rcnn/data/demo/IMG_20190521_181206.jpg']
    # for im_name in im_names:
    #     print '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
    #     print 'Demo for {}'.format(im_name)
    #     # Load the demo image
    #     im = cv2.imread(im_name)
    #     boxes=detecte_and_draw(net, im)
    #     angle=[]
    #     for box in boxes:
    #         angle.append(math.atan2((CAMERAux-box[2]),CAMERAFdSx))
    #         angle.append(math.atan2((CAMERAux-box[0]),CAMERAFdSx))
    #     angle_pub.publish(TrunkObsMsg(None,angle))

    # cv2.waitKey(0);
    rospy.spin()
