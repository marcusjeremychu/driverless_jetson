execute_process(COMMAND "/home/uwfsae/driverless_ws/build/trt_yolo_ros/trt_yolo/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/uwfsae/driverless_ws/build/trt_yolo_ros/trt_yolo/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
