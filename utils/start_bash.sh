#!/bin/bash

docker container run -it --rm \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e DEPTHAI_LEVEL=debug \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    lincolnxlw/my-depthai-ros \
    bash
