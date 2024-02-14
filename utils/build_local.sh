#!/bin/bash

if [ ! -d "tmp/depthai-ros" ]; then
    echo "Cloning depthai-ros"
    git clone -b noetic https://github.com/luxonis/depthai-ros.git tmp/depthai-ros/
fi

docker build \
    -f ./docker/Dockerfile \
    --memory=8g \
    --platform linux/amd64 \
    --build-arg USE_RVIZ=1 \
    --build-arg DEPTHAI_ROS_PATH="./tmp/depthai-ros" \
    -t lincolnxlw/my-depthai-ros \
    .
