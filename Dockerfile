FROM --platform=${BUILDPLATFORM:-linux/amd64} ghcr.io/tiiuae/fog-ros-sdk:v3.0.1-${TARGETARCH:-amd64} AS builder

ARG TARGETARCH

RUN apt update \
    && apt install -y --no-install-recommends \
        curl \
    && rm -rf /var/lib/apt/lists/*

RUN curl https://artifacts.luxonis.com/artifactory/luxonis-depthai-data-local/network/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob \
    -o /tmp/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob

COPY . /main_ws/src/

# There is a file at /sdk_install/sysroots/core2-64-oe-linux/usr/include/depthai-shared/common/optional.hpp which has the 
# which has the line '#include "tl/optional.hpp"', change it to this with sed: '#include "depthai-shared/3rdparty/tl/optional.hpp"'
RUN YOCTO_TARGET_ARCH=$(/packaging/arch_translation.sh ${TARGETARCH:-amd64}) && \
    sed -i 's/#include "tl\/optional.hpp"/#include "depthai-shared\/3rdparty\/tl\/optional.hpp"/g' /sdk_install/sysroots/${YOCTO_TARGET_ARCH}-oe-linux/usr/include/depthai-shared/common/optional.hpp && \
    sed -i 's/#include "tl\/optional.hpp"/#include "depthai-shared\/3rdparty\/tl\/optional.hpp"/g' /sdk_install/sysroots/${YOCTO_TARGET_ARCH}-oe-linux/usr/include/depthai/pipeline/Node.hpp

# CV Bridge cmake file has host contamination, which leads to failure in the build.
# This hardcoded path will be different for each of the build. So, the sed command should find the place using OpenCV_CONFIG_PATH and OpenCV_INSTALL_PATH tags
RUN YOCTO_TARGET_ARCH=$(/packaging/arch_translation.sh ${TARGETARCH:-amd64}) && \
    sed -i 's/set(OpenCV_CONFIG_PATH .*)/set(OpenCV_CONFIG_PATH \/sdk_install\/sysroots\/${YOCTO_TARGET_ARCH}-oe-linux\/usr\/lib\/cmake\/opencv4)/g' /sdk_install/sysroots/${YOCTO_TARGET_ARCH}-oe-linux/usr/share/cv_bridge/cmake/cv_bridge-extras.cmake && \
    sed -i 's/set(OpenCV_INSTALL_PATH .*)/set(OpenCV_INSTALL_PATH \/sdk_install\/sysroots\/${YOCTO_TARGET_ARCH}-oe-linux\/usr)/g' /sdk_install/sysroots/${YOCTO_TARGET_ARCH}-oe-linux/usr/share/cv_bridge/cmake/cv_bridge-extras.cmake

WORKDIR /main_ws

RUN /packaging/build_colcon_sdk.sh ${TARGETARCH:-amd64}

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:v3.0.1

RUN apt update \
    && apt install -y --no-install-recommends \
        libusb-1.0-0 \
        eudev-staticdev \
        nlohmann-json-dev \
        yaml-cpp-vendor \
        cv-bridge \
        vision-msgs \
        camera-info-manager \
        camera-calibration-parsers \
        xacro \
        image-transport \
        libyaml-cpp0.6 \
        libyaml-0-staticdev \
        libyaml-vendor \
        libopencv-ts \
        opencv-staticdev \
        libdepthai-core20 \
        binutils-dev \
        backward-ros \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir /depthai_configs
COPY --from=builder /main_ws/src/params /depthai_configs/.
COPY --from=builder /tmp/yolo-v4-tiny-tf_openvino_2021.4_6shave.blob /depthai_configs/.

VOLUME /depthai_configs
ENV DEPTHAI_PARAM_FILE /depthai_configs/parameters.yaml

ENTRYPOINT [ "/entrypoint.sh" ]

COPY entrypoint.sh /entrypoint.sh

COPY --from=builder $INSTALL_DIR $INSTALL_DIR

COPY --from=builder $WORKSPACE_DIR/build $WORKSPACE_DIR/build
COPY --from=builder $WORKSPACE_DIR/src $WORKSPACE_DIR/src
