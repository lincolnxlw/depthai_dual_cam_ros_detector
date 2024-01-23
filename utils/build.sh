#!/bin/bash

if [ ! -d "tmp/depthai-ros" ]; then
    echo "Cloning depthai-ros"
    git clone -b noetic https://github.com/luxonis/depthai-ros.git tmp/depthai-ros/
fi

docker buildx build --push \
    -f ./docker/Dockerfile \
    --memory=8g \
    --target my-depthai-ros \
    --platform linux/amd64,linux/arm64/v8 \
    --build-arg USE_RVIZ=1 \
    --build-arg DEPTHAI_ROS_PATH="./tmp/depthai-ros" \
    -t lincolnxlw/my-depthai-ros \
    .

docker pull lincolnxlw/my-depthai-ros