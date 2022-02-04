#!/usr/bin/env python3

# YOLOv3 🚀 by Ultralytics, GPL-3.0 license
"""
Run inference on images, videos, directories, streams, etc.

Usage:
    $ python path/to/detect.py --weights yolov3.pt --source 0  # webcam
                                                             img.jpg  # image
                                                             vid.mp4  # video
                                                             path/  # directory
                                                             path/*.jpg  # glob
                                                             'https://youtu.be/Zgi9g1ksQHc'  # YouTube
                                                             'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream
"""

import argparse
import os
import sys
from pathlib import Path

import cv2
import torch
import torch.backends.cudnn as cudnn
import torch.nn.functional as F

import rospy
from sensor_msgs.msg import Image 
from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped
import numpy as np
import cv2
from cv_bridge import CvBridge

SHOW = True
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.datasets import IMG_FORMATS, VID_FORMATS, LoadImages, LoadStreams
from utils.general import (LOGGER, check_file, check_img_size, check_imshow, check_requirements, colorstr,
                           increment_path, non_max_suppression, print_args, scale_coords, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, time_sync
from utils.augmentations import letterbox

curr_image = None

# # 2D for simplicity
# def translate_pose_to_rotation_and_translation(pose):
#     translation_matrix = np.array([pose.position.x, pose.position.y, pose.position.z])
#     rotation_matrix = np.cos()

class Inference():
    def __init__(self):
        self.weights='/home/uwfsae/driverless_ws/src/inference_yolo/src/best_300epoch_alldata.pt'  # model.pt path(s)
        self.imgsz=640  # inference size (pixels)
        self.conf_thres=0.25  # confidence threshold
        self.iou_thres=0.45  # NMS IOU threshold
        self.max_det=1000  # maximum detections per image
        self.device=''  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        self.classes=None  # filter by class: --class 0, or --class 0 2 3
        self.agnostic_nms=False  # class-agnostic NMS
        self.augment=False  # augmented inference
        self.half=False  # use FP16 half-precision inference
        self.dnn=False  # use OpenCV DNN for ONNX inference
        self.cone_pub = rospy.Publisher("/cone_poses_camera", PoseArray, queue_size=1)
        self.bounding_box_pub = rospy.Publisher("/bounding_box_image", Image, queue_size=1)

        # initialize cuda device
        cuda = torch.cuda.is_available()
        self.device = torch.device('cuda:0' if cuda else 'cpu')
        
        # Load model
        self.device = select_device(self.device)

        self.model = DetectMultiBackend(self.weights, device=self.device, dnn=self.dnn)
        self.stride, self.names, self.pt, self.jit, self.onnx = self.model.stride, self.model.names, self.model.pt, self.model.jit, self.model.onnx
        self.imgsz = check_img_size(self.imgsz, s=self.stride)  # check image size

        # Half
        self.half &= self.pt and self.device.type != 'cpu'  # half precision only supported by PyTorch on CUDA
        if self.pt:
            self.model.model.half() if self.half else self.model.model.float()

        if self.pt and self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device).type_as(next(self.model.model.parameters())))  # warmup
    

    @torch.no_grad()
    def infer(self, im0):
        cone_poses = PoseArray()
        cone_poses.poses = []

        # Padded resize
        img = letterbox(im0, self.imgsz, stride=self.stride, auto=self.pt and not self.jit)[0]

        # Convert
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)

        # prepare image
        im = torch.from_numpy(img).to(self.device)
        im = im.half() if self.half else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim

        # Inference
        pred = self.model(im, augment=self.augment, visualize=False)

        # NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)

        # Process predictions
        height = im.shape[1]
        width = im.shape[0]

        image_draw = im0.copy()
        for i, det in enumerate(pred):  # per image

            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                for detection in det:
                    x0, y0, x1, y1, prob, cone_class = detection
                    center = (int((x1-x0)/2 + x0), int((y1-y0)/2 + y0))
                    # estimate depth to cone
                    # scaling factor * (focal length * real cone height * image height)/(pixel_height * sensor height)
                    h_pixel = int(y1) - int(y0)
                    if h_pixel <= 0:
                        continue
                    scaling_factor = 2464 / height
                    z_camera = (scaling_factor * (2.6 * 325 * height)/(h_pixel * 4.93)) / 1000 # depth
                    y_camera = (center[1] - height/2) * z_camera/1153
                    x_camera = (center[0] - width/2) * z_camera/1153

                    
                    # Compose cone pose and add to array
                    cone_pose = Pose()
                    cone_pose.position.x = z_camera #cone_pos_camera_frame[0]
                    cone_pose.position.y = x_camera #cone_pos_camera_frame[1]
                    cone_pose.position.z = y_camera #cone_pos_camera_frame[2]
                    cone_pose.orientation.x = 0 #upright_quaternion[0]
                    cone_pose.orientation.y = 0 #upright_quaternion[1]
                    cone_pose.orientation.z = 0 #upright_quaternion[2]
                    cone_pose.orientation.w = 1 #upright_quaternion[3]
                    cone_poses.poses.append(cone_pose)

                    if SHOW:
                        cv2.rectangle(image_draw, (int(x0), int(y0)), (int(x1), int(y1)), (0,255,0), 2)
                        text = self.names[int(cone_class)] + ": " + str(round(float(prob), 2)) + ", d=" + str(round(float(z_camera), 2))
                        cv2.putText(image_draw, text, (int(x0) ,int(y0 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                    
        cone_poses.header.stamp = rospy.Time.now()
        cone_poses.header.frame_id="base_link"
        self.cone_pub.publish(cone_poses)

        if SHOW:
            bridge = CvBridge()
            self.bounding_box_pub.publish(bridge.cv2_to_imgmsg(image_draw, encoding="passthrough"))

def update_img(msg):
    global curr_image
    curr_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

def update_curr_pose(msg):
    global curr_pose
    curr_pose = msg

if __name__ == '__main__':
    rospy.init_node('pipeline_listener', anonymous=True)
    rospy.Subscriber("/stereo/left/image_raw", Image, update_img)
    rospy.Subscriber('/slam_out_pose', Pose, update_curr_pose)
    
    rate = rospy.Rate(20) # 20Hz
    inference = Inference()
    
    while not rospy.is_shutdown():
        inference.infer(curr_image)
        rate.sleep()

