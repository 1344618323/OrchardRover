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
from or_msgs.msg import TrunkAngleMsg

CLASSES = ('__background__',
           'tree', 'pine_tree')

CAMERAFdSx= 752.9652
CAMERAux= 308.3392

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


def detecte_and_draw(net, im):
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
    cv2.imshow("im", im)
    return pubbox


if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    angle_pub = rospy.Publisher('trunkAngle', TrunkAngleMsg, queue_size=10)

    #faster-RCNN
    cfg.TEST.HAS_RPN = True  # Use RPN for proposals
    prototxt = '/home/cxn/myfile/py-faster-rcnn/models/pascal_voc/VGG16/faster_rcnn_alt_opt/faster_rcnn_test_cxn.pt'
    caffemodel = '/home/cxn/myfile/py-faster-rcnn/data/faster_rcnn_models/vgg16_faster_rcnn_iter_4000.caffemodel'
    if not osp.isfile(caffemodel):
        raise IOError(('{:s} not found.').format(caffemodel))
    caffe.set_mode_gpu()
    caffe.set_device(0)
    net = caffe.Net(prototxt, caffemodel, caffe.TEST)
    print '\n\nLoaded network {:s}'.format(caffemodel)

    # cap = cv2.VideoCapture(
    #     '/home/cxn/myfile/py-faster-rcnn/data/demo/beisu3.mp4')
    cap = cv2.VideoCapture(1)
    # if cap.isOpened():
    #     cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
    #     cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)

    fs=cv2.FileStorage('/home/cxn/myfile/orchardrover_ws/src/OrchardRover/or_detection/camparams/LongFCamCalib.yaml',cv2.FileStorage_READ)
    intrinsicMatrix=fs.getNode('intrinsicMatrix').mat()
    distCoeffs=fs.getNode('distCoeffs').mat()
    fs.release()

    print intrinsicMatrix
    print distCoeffs

    while cap.isOpened() and not rospy.is_shutdown():
        ret, im = cap.read()
        out=np.zeros_like(im)
        cv2.undistort(im,intrinsicMatrix,distCoeffs,out)
        # cv2.imshow('frame',out)

        # boxes=detecte_and_draw(net, im)
        boxes=detecte_and_draw(net, out)
        if boxes !=[]:
            angle=[]
            for box in boxes:
                angle.append(math.atan2((intrinsicMatrix[0][2]-box[2]),intrinsicMatrix[0][0])*180/math.pi)
                angle.append(math.atan2((intrinsicMatrix[0][2]-box[0]),intrinsicMatrix[0][0])*180/math.pi)
            angle_pub.publish(TrunkAngleMsg(None,angle))
        cv2.waitKey(5)
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
    #     angle_pub.publish(TrunkAngleMsg(None,angle))

    # cv2.waitKey(0);
    # rospy.spin()
