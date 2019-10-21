import os.path as osp
import sys
def add_path(path):
    if path not in sys.path:
        sys.path.insert(0, path)


this_dir = osp.dirname(__file__)
# Add caffe to PYTHONPATH
add_path('/home/cxn/myfile/py-faster-rcnn/caffe-fast-rcnn/python')
# Add lib to PYTHONPATH
add_path('/home/cxn/myfile/py-faster-rcnn/lib')
