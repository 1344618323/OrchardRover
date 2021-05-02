#!/usr/bin/env python
# coding:utf-8

# 要让py文件可执行，要在终端输入：chmod +x trunk_detection_laser_cam.py

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
import copy

import rospy
import message_filters
from sensor_msgs.msg import Image, LaserScan
import geometry_msgs.msg
import cv_bridge
import std_msgs.msg

from or_msgs.msg import TrunkObsMsgXY


CLASSES = ('__background__',
           'tree', 'pine_tree')

# 用网络找树干


def DetectTrunkByNet(net, im):
    """Detect object classes in an image using pre-computed object proposals."""

    # Detect all object classes and regress object bounds
    # timer = Timer()
    # timer.tic()
    scores, boxes = im_detect(net, im)
    # timer.toc()
    # print ('Detection took {:.3f}s for '
    #        '{:d} object proposals').format(timer.total_time, boxes.shape[0])

    # Visualize detections for each class
    CONF_THRESH = 0.5
    NMS_THRESH = 0.1
    clsdict = dict()
    for cls_ind, cls in enumerate(CLASSES[2:]):
        cls_ind += 1  # because we skipped background
        cls_boxes = boxes[:, 4*cls_ind:4*(cls_ind + 1)]
        cls_scores = scores[:, cls_ind]
        dets = np.hstack((cls_boxes,
                          cls_scores[:, np.newaxis])).astype(np.float32)
        keep = nms(dets, NMS_THRESH)  # 目测是把对同一个目标识别处理的矩形做融合
        dets = dets[keep, :]  # 最终剩下的矩形
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
    return pubbox


cfg.TEST.HAS_RPN = True  # Use RPN for proposals


class TrunkDetector:
    def __init__(self, cam_only):
        self.get_img_ = []
        self.get_scan = LaserScan()

        self.cv_bridge_ = cv_bridge.CvBridge()

        self.image_sub_ = message_filters.Subscriber(
            '/usb_cam/image_raw', Image)

        if cam_only:
            self.image_sub_.registerCallback(self.image_callback)
        else:
            self.laser_sub_ = message_filters.Subscriber('/scan', LaserScan)
            self.ts_ = message_filters.ApproximateTimeSynchronizer(
                [self.image_sub_, self.laser_sub_], 1, 0.1, allow_headerless=True)
            self.ts_.registerCallback(self.image_laser_callback)

    def image_laser_callback(self, img_msg, laser_msg):
        self.get_img_ = self.cv_bridge_.imgmsg_to_cv2(
            img_msg, desired_encoding='bgr8')
        self.get_scan = laser_msg

    def image_callback(self, img_msg):
        self.get_img_ = self.cv_bridge_.imgmsg_to_cv2(
            img_msg, desired_encoding='bgr8')


class CameraLaser:
    def __init__(self):
        ###########读yaml文件：相机内参，相机雷达外参##########
        fs = cv2.FileStorage(
            '/home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_lasercamcal/config/calibra_config_pinhole.yaml', cv2.FileStorage_READ)
        self.fx = fs.getNode('projection_parameters').getNode('fx').real()
        self.fy = fs.getNode('projection_parameters').getNode('fy').real()
        self.cx = fs.getNode('projection_parameters').getNode('cx').real()
        self.cy = fs.getNode('projection_parameters').getNode('cy').real()
        self.k1 = fs.getNode('distortion_parameters').getNode('k1').real()
        self.k2 = fs.getNode('distortion_parameters').getNode('k2').real()
        self.p1 = fs.getNode('distortion_parameters').getNode('p1').real()
        self.p2 = fs.getNode('distortion_parameters').getNode('p2').real()
        self.image_width = fs.getNode('image_width').real()
        self.image_height = fs.getNode('image_height').real()
        # 关闭文件
        fs.release()
        # 内参矩阵
        self.internal_Mat = np.array(
            [[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]])

        fs = cv2.FileStorage(
            '/home/cxn/myfile/OR_ws/orchardrover_ws/src/OrchardRover/or_lasercamcal/log/result.yaml', cv2.FileStorage_READ)
        self.extrinsic_Tlc_ = fs.getNode('extrinsicTlc').mat()
        fs.release()
        # 计算 Tcam_laser
        self.Rcl = (self.extrinsic_Tlc_[0:3, 0:3]).T
        self.tcl = - \
            np.dot(self.Rcl, (self.extrinsic_Tlc_[0:3, 3]).reshape(3, 1))

    # 相机坐标系P3d -> 归一化平面坐标p_u（函数输入参数，维度3×1） -> 畸变d_u（函数输出参数，维度3×1） -> 归一化平面坐标畸变后 p_u+d_u
    def Distortion(self, p_u):
        d_u = np.zeros(3)
        mx2_u = p_u[0] * p_u[0]                         # x^2
        my2_u = p_u[1] * p_u[1]                         # y^2
        mxy_u = p_u[0] * p_u[1]                         # xy
        rho2_u = mx2_u + my2_u                          # r^2=x^2+y^2
        rad_dist_u = self.k1 * rho2_u + self.k2 * rho2_u * rho2_u  # k1*r^2 + K2* r^4
        d_u[0] = p_u[0] * rad_dist_u + 2.0 * self.p1 * \
            mxy_u + self.p2 * (rho2_u + 2.0 * mx2_u)
        d_u[1] = p_u[1] * rad_dist_u + 2.0 * self.p2 * \
            mxy_u + self.p1 * (rho2_u + 2.0 * my2_u)
        return d_u

    # 相机坐标系P3d（函数输入参数,维度3×N） -> 归一化平面坐标p_u -> 畸变d_u（函数输出参数） ->
    #   归一化平面坐标畸变后 p_u+d_u -> 像素坐标系坐标(函数输出参数，维度2×N)
    def SpaceToPlane(self, P3d):
        P2d = []
        # Project points to the normalised plane
        for i in range(P3d.shape[1]):
            # 3×N，遍历每一列
            P3d[:, i] = P3d[:, i]/P3d[2, i]
            # Apply distortion
            d_u = self.Distortion(P3d[:, i])
            p_d = (P3d[:, i] + d_u).reshape(3, 1)

            if P2d == []:
                P2d = p_d
            else:
                P2d = np.hstack((P2d, p_d))

        # Apply generalised projection matrix
        P2d = np.dot(self.internal_Mat, P2d)
        return P2d[0:2, :]

    # 激光雷达坐标系坐标 laserPoints (函数输入参数,维度3×N) ->  相机坐标系坐标 P3d -> 像素坐标系坐标(函数输出参数，维度2×N)
    def ProjectPoints(self, laserPoints):
        # laserPoints:3*N
        P3d = np.dot(self.Rcl, laserPoints)+self.tcl
        return self.SpaceToPlane(P3d)

    # 将一帧sensor_msgs.msg.LaserScan(函数输入参数)  转成 激光雷达坐标系坐标 points (函数输出参数,维度3×N)
    def TranScanToPoints(self, scan_in):
        points = []
        # -60~60度，-120~120,240~480

        if scan_in.ranges == []:
            return points

        for i in range(239, 480):
            r = scan_in.ranges[i]
            if r <= scan_in.range_min or r > float(35.0):
                continue
            p = np.zeros(3).reshape(3, 1)
            p[0] = math.cos(scan_in.angle_min + i * scan_in.angle_increment)*r
            p[1] = math.sin(scan_in.angle_min + i * scan_in.angle_increment)*r

            if points == []:
                points = p
            else:
                points = np.hstack((points, p))
        return points

    # 在 输入图片 process_img 上绘制 激光点 投影效果
    def ShowProject(self, process_img):
        points = self.TranScanToPoints(scan_in)
        if points == []:
            return
        img_points = self.ProjectPoints(points)
        for i in range(img_points.shape[1]):
            # 2×N，遍历每一列
            x = int(img_points[0, i])
            y = int(img_points[1, i])
            if x >= 0 and x < self.image_width and y >= 0 and y < self.image_height:
                cv2.circle(process_img, (x, y), 1, (0, 0, 255), -1)

    # 建立map关系 map(像素坐标系u分量)=激光雷达坐标系坐标
    # 若对同一个u有多个激光坐标，选近距离的
    def ProjectAndMap(self, scan_in):
        points_map = dict()
        points = self.TranScanToPoints(scan_in)
        if points == []:
            return points_map
        img_points = self.ProjectPoints(points)
        for i in range(img_points.shape[1]):
            # 2×N，遍历每一列
            new_value = points[0:2, i]
            key = round(img_points[0, i])
            if points_map.has_key(key):
                if np.linalg.norm(points_map[key]) > np.linalg.norm(new_value):
                    points_map[key] = [new_value, img_points[:, i]]
            else:
                points_map[key] = [new_value, img_points[:, i]]
        return points_map


# 用opencv在图像中写文字
def draw_text(img, point, text, drawType="custom"):
    '''
    :param img:
    :param point:
    :param text:
    :param drawType: custom or custom
    :return:
    '''
    fontScale = 0.6
    thickness = 5
    text_thickness = 2
    bg_color = (255, 0, 0)
    fontFace = cv2.FONT_HERSHEY_SIMPLEX
    # fontFace=cv2.FONT_HERSHEY_SIMPLEX
    if drawType == "custom":
        text_size, baseline = cv2.getTextSize(
            str(text), fontFace, fontScale, thickness)
        text_loc = (point[0], point[1] + text_size[1])
        # cv2.rectangle(img, (text_loc[0] - 2 // 2, text_loc[1] - 2 - baseline),
        #               (text_loc[0] + text_size[0], text_loc[1] + text_size[1]), bg_color, -1)
        # draw score value
        cv2.putText(img, str(text), (text_loc[0], text_loc[1] + baseline), fontFace, fontScale,
                    (0, 255, 0), text_thickness, 8)
    elif drawType == "simple":
        cv2.putText(img, '%d' % (text), point, fontFace, 0.5, (255, 0, 0))
    return img

# 输入limits（boundingBoxes）,找出每个box中心点u 对应的 激光坐标


def CalcTrunkDepth(limits, points_map, process_img=None, visualize=False):
    laser_points = []
    for limit in limits:
        vec = []
        for key in range(int(limit[0]), int(limit[2]+1)):
            if points_map.has_key(key):
                vec.append(points_map[key])
                if visualize:
                    cv2.circle(process_img, (int(points_map[key][1][0]), int(
                        points_map[key][1][1])), 3, (0, 0, 255), -1)
        if len(vec) > 2:
            mid = len(vec)/2
            p = geometry_msgs.msg.Point()
            p.x = vec[mid][0][0]
            p.y = vec[mid][0][1]
            p.z = 0.0
            laser_points.append(p)

            if visualize:
                cv2.circle(process_img, (int(vec[mid][1][0]), int(
                    vec[mid][1][1])), 3, (0, 255, 255), -1)
                text = '{:.1f},{:.1f}'.format(p.x, p.y)
                draw_text(process_img, (int(limit[0]), int(
                    limit[3])), text, drawType="custom")

    return laser_points


if __name__ == '__main__':
    rospy.init_node('trunk_detection_node', anonymous=True)

    visualize = rospy.get_param('or_detection/visualize', True)
    cam_only = rospy.get_param('or_detection/cam_only', False)

    # ↓加载 faster-RCNN 模型↓
    prototxt = '/home/cxn/myfile/OR_ws/py-faster-rcnn/models/pascal_voc/VGG16/faster_rcnn_alt_opt/faster_rcnn_test_cxn.pt'
    caffemodel = '/home/cxn/myfile/OR_ws/py-faster-rcnn/data/faster_rcnn_models/vgg16_faster_rcnn_iter_4000.caffemodel'
    if not osp.isfile(caffemodel):
        raise IOError(('{:s} not found.').format(caffemodel))
    caffe.set_mode_gpu()
    caffe.set_device(0)
    net = caffe.Net(prototxt, caffemodel, caffe.TEST)
    print '\n\nLoaded network {:s}'.format(caffemodel)
    # ↑加载 faster-RCNN 模型↑

    cam_laser = CameraLaser()

    obs_pub = rospy.Publisher(
        'trunk_obs', TrunkObsMsgXY, queue_size=5)

    trunk_detector = TrunkDetector(cam_only)

    while not rospy.is_shutdown():
        if cam_only:
            if trunk_detector.get_img_ == []:
                continue
            process_img = copy.deepcopy(trunk_detector.get_img_)
            boxes = DetectTrunkByNet(net, process_img)

        else:
            if trunk_detector.get_img_ == [] or trunk_detector.get_scan == []:
                continue
            process_img = copy.deepcopy(trunk_detector.get_img_)
            scan_in = copy.deepcopy(trunk_detector.get_scan)

            points_map = cam_laser.ProjectAndMap(scan_in)

            boxes = DetectTrunkByNet(net, process_img)

            laser_points = CalcTrunkDepth(
                boxes, points_map, process_img, visualize)

            if laser_points != []:
                hdr = std_msgs.msg.Header(
                    stamp=scan_in.header.stamp, frame_id='laser')
                obs_pub.publish(TrunkObsMsgXY(hdr, laser_points))

        if visualize:
            cv2.imshow("ProcessImg", process_img)
            cv2.waitKey(5)

    cv2.destroyAllWindows()

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
