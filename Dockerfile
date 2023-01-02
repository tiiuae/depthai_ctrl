FROM ghcr.io/tiiuae/fog-ros-baseimage-builder:v1.0.0 AS builder

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
    && rm -rf /var/lib/apt/lists/*

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb in /main_ws/
RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:v2.0.0

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
    && rm -rf /var/lib/apt/lists/*

RUN ln -s /usr/bin/true /usr/bin/udevadm \
    && dpkg -i /depthai.deb && rm /depthai.deb \
    && rm -f /usr/bin/udevadm
