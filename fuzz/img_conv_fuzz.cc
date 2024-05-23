#include <cstdint>
#include <cv_bridge/cv_bridge.hpp>
#include <fuzzer/FuzzedDataProvider.h>

#include "depthai-shared/common/CameraInfo.hpp"
#include <depthai/depthai.hpp>
#include <depthai_ctrl/ImageConverter.hpp>
#include <depthai_ctrl/ImgDetectionConverter.hpp>
#include <depthai_ctrl/ImuConverter.hpp>
#include <deque>
#include <iostream>
#include <memory>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/core/hal/interface.h>
#include <ratio>
#include <tuple>
#include <vector>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *Data, size_t Size) {
  FuzzedDataProvider dataProvider(Data, Size);

  auto inData = std::make_shared<dai::ImgFrame>();

  unsigned int width = dataProvider.ConsumeIntegral<int64_t>();
  unsigned int height = dataProvider.ConsumeIntegral<int64_t>();
  unsigned int channelSize = dataProvider.ConsumeIntegral<int64_t>();
  size_t imageSize = width * height * channelSize;
  std::vector<uint8_t> frameData =
      dataProvider.ConsumeBytes<uint8_t>(imageSize);

  // Set the generated data to the ImgFrame object
  inData->setWidth(width);
  inData->setHeight(height);
  inData->setData(frameData);

  // NOTE: Check if inData is populated
  // Otherwise we're triggering an assert:
  //     what():  OpenCV(4.6.0) ./modules/imgcodecs/src/loadsave.cpp:816: error:
  //     (-215:Assertion failed) !buf.empty() in function 'imdecode_'
  if (inData->getData().empty()) {
    return 1;
  }

  // NOTE: Another sanity check for:
  //     what():  OpenCV(4.6.0)
  //     /usr/include/opencv4/opencv2/core/utility.hpp:627: error:
  //     (-215:Assertion failed) !empty() in function 'forEach_impl'
  if (width <= 0 || height <= 0) {
    return 1;
  }

  std::deque<dai::ros::ImageMsgs::Image> outImageMsgs;
  dai::RawImgFrame::Type type;
  switch (dataProvider.ConsumeIntegralInRange(0, 2)) {
  case 0:
    type = dai::RawImgFrame::Type::RAW8;
    inData->setType(type);
    break;
  case 1:
    type = dai::RawImgFrame::Type::GRAY8;
    inData->setType(type);
    break;
  case 2:
    type = dai::RawImgFrame::Type::BGR888i;
    inData->setType(type);
    break;
  default:
    return 0;
  }
  sensor_msgs::msg::CameraInfo info;
  std::string frameName = dataProvider.ConsumeRandomLengthString();
  bool interleaved = dataProvider.ConsumeBool();

  dai::ros::ImageConverter imageConverter(frameName, interleaved);

  try {
    imageConverter.toRosMsgFromBitStream(inData, outImageMsgs, type, info);
  } catch (const cv::Exception &e) {
    return -1;
  }

  // Check if the output image messages are valid
  for (const auto &msg : outImageMsgs) {
    if (msg.width <= 0 || msg.height <= 0 || msg.encoding.empty()) {
      // Invalid output image message
      return -1;
    }
  }

  return 0;
}
