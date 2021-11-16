# Parameters
# TODO: Use yml instead?
import os

YOLO_CONFIG_PATH = '/home/uwfsae/driverless_ws/src/perception/src/model_cfg/yolo_baseline.cfg'
YOLO_WEIGHT_PATH = '/home/uwfsae/driverless_ws/src/perception/src/weights/pretrained_yolo.weights'
YOLO_CONF_THRES = 0.8
YOLO_NMS_THRES = 0.25
YOLO_VANILLA_ANCHOR = False
YOLO_XY_LOSS = 2
YOLO_WH_LOSS = 1.6
YOLO_NO_OBJECT_LOSS = 25
YOLO_OBJECT_LOSS = 0.1

REKTNET_INPUT_SIZE = (80, 80)
REKTNET_WEIGHT_PATH = '/home/uwfsae/driverless_ws/src/perception/src/weights/pretrained_kpt.pt'
