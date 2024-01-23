#!/bin/bash

docker container run -it --rm \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    lincolnxlw/my-depthai-ros \
    bash -c "roscore & cd /catkin_ws/devel/lib/minimum_image_publisher && pipeline_graph run './image_publisher_ros_node'"
