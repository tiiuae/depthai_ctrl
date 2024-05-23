#include <cv_bridge/cv_bridge.hpp>
#include <fuzzer/FuzzedDataProvider.h>

#include "depthai-shared/common/CameraInfo.hpp"
#include <depthai/depthai.hpp>
#include <depthai_ctrl/ImageConverter.hpp>
#include <depthai_ctrl/ImgDetectionConverter.hpp>
#include <depthai_ctrl/ImuConverter.hpp>
#include <deque>
#include <memory>
#include <ratio>
#include <tuple>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size) {
  FuzzedDataProvider dataProvider(Data, Size);

  std::string frameName = dataProvider.ConsumeRandomLengthString();

  auto inData = std::make_shared<dai::IMUData>();
  // NOTE: Starting from 0 leads to an OOB access
  size_t numPackets = dataProvider.ConsumeIntegralInRange<size_t>(1, 10);

  // Generate random packets and add them to inData->packets
  for (size_t i = 0; i < numPackets; ++i) {
    dai::IMUPacket packet;

    // Populate the fields of IMUPacket
    packet.acceleroMeter.x = dataProvider.ConsumeFloatingPoint<float>();
    packet.acceleroMeter.y = dataProvider.ConsumeFloatingPoint<float>();
    packet.acceleroMeter.z = dataProvider.ConsumeFloatingPoint<float>();

    packet.gyroscope.x = dataProvider.ConsumeFloatingPoint<float>();
    packet.gyroscope.y = dataProvider.ConsumeFloatingPoint<float>();
    packet.gyroscope.z = dataProvider.ConsumeFloatingPoint<float>();

    packet.magneticField.x = dataProvider.ConsumeFloatingPoint<float>();
    packet.magneticField.y = dataProvider.ConsumeFloatingPoint<float>();
    packet.magneticField.z = dataProvider.ConsumeFloatingPoint<float>();

    packet.rotationVector.i = dataProvider.ConsumeFloatingPoint<float>();
    packet.rotationVector.j = dataProvider.ConsumeFloatingPoint<float>();
    packet.rotationVector.k = dataProvider.ConsumeFloatingPoint<float>();
    packet.rotationVector.real = dataProvider.ConsumeFloatingPoint<float>();
    packet.rotationVector.rotationVectorAccuracy =
        dataProvider.ConsumeFloatingPoint<float>();

    inData->packets.push_back(packet);
  }

  dai::ros::ImuConverter imuConverter(frameName);
  dai::ros::ImuMsgs::Imu outImuMsg;
  imuConverter.toRosMsg(inData, outImuMsg);
  auto rosMsgPtr = imuConverter.toRosMsgPtr(inData);

  return 0;
}
