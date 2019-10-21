#!/usr/bin/env python
import rospy
import cv2
import os
import sys
import numpy as np
import os.path as osp
import trunk_detection_initpath
import caffe
from fast_rcnn.test import im_detect
from fast_rcnn.nms_wrapper import nms
from fast_rcnn.config import cfg
from utils.timer import Timer
import matplotlib.pyplot as plt

CLASSES = ('__background__',
           'tree', 'pine_tree')

def vis_detections(im, class_name, dets, fig, ax, thresh=0.5):
    """Draw detected bounding boxes."""
    inds = np.where(dets[:, -1] >= thresh)[0]
    if len(inds) == 0:
        return

    im = im[:, :, (2, 1, 0)]

    ax.imshow(im, aspect='equal')
    for i in inds:
        bbox = dets[i, :4]
        score = dets[i, -1]

        ax.add_patch(
            plt.Rectangle((bbox[0], bbox[1]),
                          bbox[2] - bbox[0],
                          bbox[3] - bbox[1], fill=False,
                          edgecolor='red', linewidth=3.5)
        )
        ax.text(bbox[0], bbox[1] - 2,
                '{:s} {:.3f}'.format(class_name, score),
                bbox=dict(facecolor='blue', alpha=0.5),
                fontsize=14, color='white')

    ax.set_title(('{} detections with '
                  'p({} | box) >= {:.1f}').format(class_name, class_name,
                                                  thresh),
                 fontsize=14)
    plt.axis('off')
    plt.tight_layout()
    plt.draw()

def demo(net, image_name):
    """Detect object classes in an image using pre-computed object proposals."""

    # Load the demo image
    im = cv2.imread(image_name)

    # Detect all object classes and regress object bounds
    timer = Timer()
    timer.tic()
    scores, boxes = im_detect(net, im)
    timer.toc()
    print ('Detection took {:.3f}s for '
           '{:d} object proposals').format(timer.total_time, boxes.shape[0])

    # Visualize detections for each class
    CONF_THRESH = 0.8
    NMS_THRESH = 0.3
    fig, ax = plt.subplots(figsize=(12, 12))
    for cls_ind, cls in enumerate(CLASSES[1:]):
        cls_ind += 1  # because we skipped background
        cls_boxes = boxes[:, 4*cls_ind:4*(cls_ind + 1)]
        cls_scores = scores[:, cls_ind]
        dets = np.hstack((cls_boxes,
                          cls_scores[:, np.newaxis])).astype(np.float32)
        keep = nms(dets, NMS_THRESH)
        dets = dets[keep, :]
        print(cls)
        vis_detections(im, cls, dets, fig, ax, thresh=CONF_THRESH)

if __name__ == '__main__':
    cfg.TEST.HAS_RPN = True  # Use RPN for proposals
    prototxt = '/home/cxn/myfile/py-faster-rcnn/models/pascal_voc/VGG16/faster_rcnn_alt_opt/faster_rcnn_test_cxn.pt'
    caffemodel = '/home/cxn/myfile/py-faster-rcnn/data/faster_rcnn_models/vgg16_faster_rcnn_iter_4000.caffemodel'
    if not osp.isfile(caffemodel):
        raise IOError(('{:s} not found.').format(caffemodel))
    caffe.set_mode_gpu()
    caffe.set_device(0)
    net = caffe.Net(prototxt, caffemodel, caffe.TEST)
    print '\n\nLoaded network {:s}'.format(caffemodel)

    im_names = ['/home/cxn/myfile/py-faster-rcnn/data/demo/IMG_20190521_171646.jpg', 
        '/home/cxn/myfile/py-faster-rcnn/data/demo/IMG_20190521_181206.jpg']
    for im_name in im_names:
        print '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
        print 'Demo for {}'.format(im_name)
        demo(net, im_name)

    plt.show()


# if __name__ == '__main__':
#     rospy.init_node('talker', anonymous=True)
#     cap = cv2.VideoCapture(0)
#     while cap.isOpened() and not rospy.is_shutdown():  
#         # Capture frame-by-frame  
#         ret, frame = cap.read()  
#         cv2.imshow('frame',frame)  
#         cv2.waitKey(5)  
#     cap.release()
#     cv2.destroyAllWindows()
    
#     # pub = rospy.Publisher('chatter', String, queue_size=10)