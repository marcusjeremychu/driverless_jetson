# UW Formula Motorsports - Driverless (Jetson Software)

## Introduction
This is a repository for the driverless effort for UW Formula Motorsports. This code lives on the NVIDIA Jetson, where it primarily handle vision related tasks. More specfically, the Jetson interfaces with the camera directly and also runs a YOLOv3-tiny model for cone detection.

## Hardware
* NVIDIA Jetson Xavier NX 
* Waveshare IMX219-83 Stereo Camera

## Package Descriptions
### ackermann_msgs
This package contains the ackermann_msgs from http://wiki.ros.org/ackermann_msgs for us to eventually publish drive commands. Ideally, these commands would essentially be the transistionary point between the ROS network and the lower-level microcontrollers. Note there is nothing to launch here.

### autonomous_system
This package functions are our finite state machine at the ROS level. The FSM Node listens for the can_node to publish the states of the vehicle (see external_input.msg) which is read from the ECU via CAN. These messages then determine the state of the vehicle in ROS and should eventually enable/disable systems in the vehicle. 

### can_node
This node listens and sends information on the CAN bus. This should be on the BeagleBone Black (BBB), but currently lives on the Jetson until the BBB is up and running. Interacts with autonomous_system.

### inference_yolo
This is a modified repo (https://github.com/ultralytics/yolov3/) which essentially wraps their inference code with ROS. The network was trained with the same repo, and so inference was also based on the same repo. The code has essentially been distilled into detect.py for our needs. Launch with roslaunch inference_yolo inference.launch. Note that this node needs a camera to be supplying images.

### perception
This runs the code that interfaces directly with the camera (https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera). Note that this is a temporary camera until we can afford better ones. In short, this node publishes images and camera info messages. It is also capable of publishing IMU messages but it doesn't work that well at the moment. The camera calibration parameters were produced with http://wiki.ros.org/camera_calibration but also can be recreated with OpenCV's calibration code if preferred. Note that stereo processing works extremely poorly on the Jetson as the code is CPU oriented.

### spi_node
This node listens and sends information via SPI. This was a temporary solution until we got CAN working, but can be repurposed for other SPI needs. 