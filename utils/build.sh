#!/bin/bash

VERSION=""

while getopts "v:" opt; do
  case $opt in
    v)
      VERSION="$OPTARG"
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done

if [ -z "$VERSION" ]; then
    echo "Please provide target docker image version with -v"
    exit 1
fi

echo "Target docker image version: "$VERSION

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
    -t lincolnxlw/my-depthai-ros:latest \
    -t lincolnxlw/my-depthai-ros:$VERSION \
    .

docker pull lincolnxlw/my-depthai-ros:latest