FROM ghcr.io/tiiuae/fog-ros-baseimage-builder:v2.0.0 AS builder

# TODO: not sure how many of these deps are actually needed for building. at least this:
# libusb-1.0-0-dev
# gstreamer-1.0 (<-- but not sure which package brings this)
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    libusb-1.0-0-dev \
    libgstreamer1.0-0 \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    libgstreamer-plugins-good1.0-dev \
    libgstrtspserver-1.0-dev \
    libgstreamer-plugins-base1.0-0 \
    libgstreamer-plugins-good1.0-0 \
    nlohmann-json3-dev \
    gstreamer1.0-x \
    libopencv-dev \
    && rm -rf /var/lib/apt/lists/*

RUN curl https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob \
    -o /tmp/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob

COPY . /main_ws/src/
# this:
# 1) builds the application
# 2) packages the application as .deb in /main_ws/
RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:v2.0.0

RUN mkdir /depthai_configs
COPY --from=builder /main_ws/src/params /depthai_configs/.
COPY --from=builder /tmp/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob /depthai_configs/.

VOLUME /depthai_configs
ENV DEPTHAI_PARAM_FILE /depthai_configs/parameters.yaml

ENTRYPOINT [ "/entrypoint.sh" ]

COPY entrypoint.sh /entrypoint.sh

COPY --from=builder /main_ws/ros-*-depthai-ctrl_*_amd64.deb /depthai.deb
# need update because ROS people have a habit of removing old packages pretty fast
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    libusb-1.0-0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gir1.2-gst-rtsp-server-1.0 \
    gstreamer1.0-libav \
    gstreamer1.0-rtsp \
    libopencv-dev \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-image-transport \
    ros-$ROS_DISTRO-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN ln -s /usr/bin/true /usr/bin/udevadm \
    && dpkg -i /depthai.deb && rm /depthai.deb \
    && rm -f /usr/bin/udevadm
