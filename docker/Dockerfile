ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}-ros-base as depthai-ros

ARG DEPTHAI_ROS_PATH
ARG USE_RVIZ
ARG BUILD_SEQUENTIAL=0
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get -y install --no-install-recommends software-properties-common git libusb-1.0-0-dev wget zsh python3-catkin-tools

ENV DEBIAN_FRONTEND=dialog
RUN sh -c "$(wget https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)"

ENV WS=/ws
RUN mkdir -p $WS/src
COPY ${DEPTHAI_ROS_PATH} .$WS/src/depthai-ros
RUN cd .$WS/ \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -y

RUN apt purge -y ros-noetic-depthai \
    && mkdir -p /root/src && cd /root/src \
    && git clone -b develop https://github.com/luxonis/depthai-core.git \
    && cd depthai-core && git checkout eb51d419f0616c90b17e2a13c09ee7f59764f8ea \
    && git submodule update --init --recursive

RUN rm -rf /root/.hunter \
    && cd /root/src/depthai-core \
    && cmake -S . -B build -D'CMAKE_INSTALL_PREFIX=/usr/local' -D'BUILD_SHARED_LIBS=ON' \
    && cmake --build build --target install

RUN rm -rf .$WS/src/depthai-ros/depthai_examples
RUN if [ "$BUILD_SEQUENTIAL" = "1" ] ; then cd .$WS/ && . /opt/ros/noetic/setup.sh && catkin build -j1 -l1; else cd .$WS/ && . /opt/ros/noetic/setup.sh && catkin build; fi 
RUN if [ "$USE_RVIZ" = "1" ] ; then echo "RVIZ ENABLED" && sudo apt install -y ros-noetic-rviz ros-noetic-rviz-imu-plugin ; else echo "RVIZ NOT ENABLED"; fi
RUN echo "if [ -f ${WS}/devel/setup.zsh ]; then source ${WS}/devel/setup.zsh; fi" >> $HOME/.zshrc
RUN echo "if [ -f ${WS}/devel/setup.bash ]; then source ${WS}/devel/setup.bash; fi" >> $HOME/.bashrc
ENTRYPOINT [ "/ws/src/depthai-ros/entrypoint.sh" ]
CMD ["zsh"]

#FROM depthai-ros as my-depthai-ros
#
#SHELL ["/bin/bash", "-c"]
#
#COPY ./src/minimum_image_publisher /catkin_ws/src/minimum_image_publisher
#
#RUN source "/opt/ros/noetic/setup.bash" \
#    && source "/ws/devel/setup.bash" \
#    && cd /catkin_ws \
#    && catkin build
#
## install pipeline graph
#RUN apt update \
#    && apt install -y python3-pip \
#    && pip install git+https://github.com/luxonis/depthai_pipeline_graph.git \
#    && pip install --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-snapshot-local/ depthai
#
#COPY ./docker/entrypoint.sh /
#RUN echo "if [ -f /catkin_ws/devel/setup.bash ]; then source /catkin_ws/devel/setup.bash; fi" >> $HOME/.bashrc
#ENTRYPOINT ["/entrypoint.sh"]