#!/usr/bin/env python3

# DEPRECATED - This code is outdated!
import torch
import torch.nn as nn
import torchvision
import PIL
import random
import math

from yolo.models import Darknet
from yolo.utils.utils import calculate_padding
from yolo.utils.nms import nms

# from rektnet.keypoint_net import KeypointNet
from geometry_msgs.msg import PoseWithCovarianceStamped

from params import *

import rospy
from sensor_msgs.msg import Image 
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
from numpy.linalg import inv
import cv2
from cv_bridge import CvBridge


PUB_BOUNDING_BOX_IMAGES = False
cone_pub = rospy.Publisher("/cone_poses_camera", PoseArray, queue_size=1)
if PUB_BOUNDING_BOX_IMAGES:
    bounding_box_pub = rospy.Publisher("/bounding_box_image", Image, queue_size=1)

camera_matrix = np.array([[1153.800791, 0.000000, 612.963545],
                 [0.000000, 1152.845387, 355.852706],
                 [0.000000, 0.000000, 1.000000]])
upright_quaternion = [0, 0.70707272, 0, 0.70714084]
curr_rot_matrix = None

# Preprocess an input image.
def preprocess_image(image: PIL.Image.Image) -> torch.Tensor:
    w, h = image.size
    new_width, new_height = yolo_model.img_size()
    pad_h, pad_w, ratio = calculate_padding(h, w, new_height, new_width)
    image = torchvision.transforms.functional.pad(image, padding=(pad_w, pad_h, pad_w, pad_h), fill=(127, 127, 127), padding_mode="constant")
    image = torchvision.transforms.functional.resize(image, (new_height, new_width))
    image = torchvision.transforms.functional.to_tensor(image)
    image = image.unsqueeze(0)
    return image, (pad_h, pad_w, ratio)

# Gets the bounding boxes from a pre-processed image.
# Input:
#   model: A YOLO model.
#   image: A preprocessed image as a Tensor.
# Output: An 2d Tensor with size n*4, each line being [x0, y0, x1, y1].
# TODO: Do we need to recover the coordinates PRIOR TO resizing the image?
# Tentative answer: yes, for this gives the keypoint regression step images with higher resolution.
def get_bounding_boxes(model: nn.Module, image: torch.Tensor) -> torch.Tensor:
    with torch.no_grad():
        model.eval()
        image = image.to(device, non_blocking=True)  # TODO: move this logic elsewhere?
        output = model(image)
        for detections in output:
            detections = detections[detections[:, 4] > YOLO_CONF_THRES]
            box_corner = torch.zeros((detections.shape[0], 4), device=detections.device)
            xy = detections[:, 0:2]
            wh = detections[:, 2:4] / 2
            box_corner[:, 0:2] = xy - wh
            box_corner[:, 2:4] = xy + wh
            probabilities = detections[:, 4]
            nms_indices = nms(box_corner, probabilities, YOLO_NMS_THRES)
            main_box_corner = box_corner[nms_indices]

            if nms_indices.shape[0] == 0:
                continue
            return main_box_corner  # why return prematurely
        
        # If we're here, we we didn't find a main box
        return torch.empty((1,1))

# TODO: Is this cache-efficient?
# Recover the bounding box coordinates *of the actual image* from the
# coordinates *of the resized image*.
def recover_box_coordinates(bounding_boxes, pad_h, pad_w, ratio):
    bounding_boxes[:, 0] = bounding_boxes[:, 0] / ratio - pad_w
    bounding_boxes[:, 2] = bounding_boxes[:, 2] / ratio - pad_w
    bounding_boxes[:, 1] = bounding_boxes[:, 1] / ratio - pad_h
    bounding_boxes[:, 3] = bounding_boxes[:, 3] / ratio - pad_h
    return bounding_boxes

# Preprocess the subimage containing an individual cone.
# Input should be the subimage delineated by a bounding box outputted by YOLO.
def preprocess_subimage(image: PIL.Image.Image) -> torch.Tensor:
    image = torchvision.transforms.functional.resize(image, REKTNET_INPUT_SIZE)
    image = torchvision.transforms.functional.to_tensor(image)
    # image = image.unsqueeze(0)
    return image

# Get the coordinates of keypoints given the image and its bounding boxes. 
# Input #2 is a list of subimages returned by get_subimages.
# Returns a tensor of shape [num_images, 7, 2], corresponding to seven keypoints.
def get_keypoints(model: nn.Module, sub_images_list: list) -> torch.Tensor:
    num_images = len(sub_images_list)
    sub_images = torch.zeros((num_images, 3, *REKTNET_INPUT_SIZE))
    for i, sub_image in enumerate(sub_images_list):
        sub_images[i] = preprocess_subimage(sub_image)
    with torch.no_grad():
        out = model(sub_images)
    return out[1]

# Returns a list of subimages (containing individual cones) detected by YOLO.
# Input: The raw input image, could be any size.
# Output: A list of Image, each containing a cone.
def get_subimages(image: PIL.Image.Image) -> list:
    image_for_yolo, scaling_info = preprocess_image(image)
    bounding_boxes_skewed = get_bounding_boxes(yolo_model, image_for_yolo)
    if len(bounding_boxes_skewed) > 1:
        bounding_boxes = recover_box_coordinates(bounding_boxes_skewed, *scaling_info)
    else:
        return []
    subimages = []
    #draw = PIL.ImageDraw.Draw(image)
    for box in bounding_boxes:
        x0, y0, x1, y1 = tuple(box.tolist())
        # To obtain a slightly larger bounding box, we need to:
        x0, y0 = math.floor(x0), math.floor(y0)
        x1, y1 = math.ceil(x1), math.ceil(y1)
        # subimages.append([image.crop((x0, y0, x1, y1)), x0, y0, x1, y1])
        subimages.append([x0, y0, x1, y1])
    #     draw.rectangle([(x0, y0), (x1, y1)], outline="red")  # very nice result!
    # image.show()
    return subimages

# Taken from https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

# Callback function for receiving an image to run image through model to predict bounding boxes
def callback(frame):
    global cone_pub, bounding_box_pub, camera_matrix, upright_quaternion, curr_pose

    # Step 1: Go through YOLO.
    image_data = np.frombuffer(frame.data, dtype=np.uint8).reshape(frame.height, frame.width, -1)
    pil_image = PIL.Image.fromarray(cv2.cvtColor(image_data, cv2.COLOR_BGR2RGB))
    cones = get_subimages(pil_image)

    cone_poses = PoseArray()
    cone_poses.poses = []
    
    if len(cones) > 0:
        print("{} cones detected.".format(len(cones)))
        for id, cone in enumerate(cones):
            x0, y0, x1, y1 = cone
        
            # estimate depth to cone
            #  scaling factor * (focal length * real cone height * image height)/(pixel_height * sensor height)
            h_pixel = y1- y0
            scaling_factor = 2464 / frame.height
            z_camera = (scaling_factor * (2.6 * 325 * frame.height)/(h_pixel * 4.93)) / 1000 # depth
            y_camera = (y1 - frame.height/2) * z_camera/1153
            x_camera = (x1 - frame.width/2) * z_camera/1153
            inv_rot_matrix = inv(curr_rot_matrix)
            # cone_pos_camera_frame = np.matmul(inv_rot_matrix,np.array([[x_world], [y_world],  [z_world]]))
            
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
            
            # label bounding box
            if PUB_BOUNDING_BOX_IMAGES:
                cv2.rectangle(image_data, (x0,y0), (x1, y1), (0,0, 255), 2)
                # label = "{}: ({}, {}, {})".format(id, x_world, y_world, z_world)
                cv2.putText(image_data, id, (x0, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 2, 2)
        # keypoints = get_keypoints(keypoint_model, cones)
    else:
        print("No cones detected")
    
    cone_poses.header.stamp = rospy.Time.now()
    cone_poses.header.frame_id="left_camera"
    cone_pub.publish(cone_poses)

    if PUB_BOUNDING_BOX_IMAGES:
        bridge = CvBridge()
        bounding_box_pub.publish(bridge.cv2_to_imgmsg(image_data, encoding="passthrough"))


# Callback function to update the current rotation matrix global variable
def pose_cb(msg):
    global curr_rot_matrix
    q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    curr_rot_matrix = quaternion_rotation_matrix(q)


if __name__ == '__main__':
    # initialize cuda device
    cuda = torch.cuda.is_available()
    device = torch.device('cuda:0' if cuda else 'cpu')
    print("\n Device: ", device)
    random.seed(0)
    torch.manual_seed(0)
    if cuda:
        torch.cuda.manual_seed(0)
        torch.cuda.manual_seed_all(0)
        torch.backends.cudnn.benchmark = True
        torch.cuda.empty_cache()
    
    # Initialize models, whose paramters can be accessed elsewhere
    yolo_model = Darknet(config_path=YOLO_CONFIG_PATH,
                         xy_loss=YOLO_XY_LOSS,
                         wh_loss=YOLO_WH_LOSS,
                         no_object_loss=YOLO_NO_OBJECT_LOSS,
                         object_loss=YOLO_OBJECT_LOSS,
                         vanilla_anchor=YOLO_VANILLA_ANCHOR)
    yolo_model.load_weights(YOLO_WEIGHT_PATH, yolo_model.get_start_weight_dim())
    yolo_model.to(device, non_blocking=True)

    # keypoint_model = KeypointNet()
    # keypoint_model.load_state_dict(torch.load(REKTNET_WEIGHT_PATH).get('model'))

    rospy.init_node('pipeline_listener', anonymous=True)
    rospy.Subscriber("/stereo/left/image_raw", Image, callback)
    rospy.Subscriber("pose", PoseWithCovarianceStamped, pose_cb)
    rospy.spin()
    cv2.destroyAllWindows()
