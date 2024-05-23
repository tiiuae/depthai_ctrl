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
  int width = dataProvider.ConsumeIntegral<int>();
  int height = dataProvider.ConsumeIntegral<int>();
  bool normalized = dataProvider.ConsumeBool();

  dai::ros::ImgDetectionConverter detectionConverter(frameName, width, height,
                                                     normalized);

  auto inNetData = std::make_shared<dai::ImgDetections>();

  // Determine the number of detections to generate
  size_t numDetections = dataProvider.ConsumeIntegralInRange<size_t>(0, 10);

  // Generate random detections and add them to inNetData->detections
  for (size_t i = 0; i < numDetections; ++i) {
    dai::ImgDetection detection;

    detection.label = dataProvider.ConsumeIntegral<int>();
    detection.confidence = dataProvider.ConsumeFloatingPoint<float>();
    detection.xmin = dataProvider.ConsumeFloatingPoint<float>();
    detection.ymin = dataProvider.ConsumeFloatingPoint<float>();
    detection.xmax = dataProvider.ConsumeFloatingPoint<float>();
    detection.ymax = dataProvider.ConsumeFloatingPoint<float>();

    inNetData->detections.push_back(detection);
  }

  auto timestamp =
      std::chrono::steady_clock::time_point(std::chrono::steady_clock::duration(
          dataProvider.ConsumeIntegral<int64_t>()));
  inNetData->setTimestamp(timestamp);

  auto timestampDevice =
      std::chrono::steady_clock::time_point(std::chrono::steady_clock::duration(
          dataProvider.ConsumeIntegral<int64_t>()));
  inNetData->setTimestampDevice(timestampDevice);

  inNetData->setSequenceNum(dataProvider.ConsumeIntegral<int64_t>());

  std::deque<dai::ros::VisionMsgs::Detection2DArray> opDetectionMsgs;

  detectionConverter.toRosMsg(inNetData, opDetectionMsgs);
  auto rosMsgPtr = detectionConverter.toRosMsgPtr(inNetData);

  return 0;
}
