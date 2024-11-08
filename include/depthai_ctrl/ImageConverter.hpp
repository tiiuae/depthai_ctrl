#pragma once

#include <cv_bridge/cv_bridge.hpp>

#include <depthai-shared/common/CameraBoardSocket.hpp>
#include <depthai-shared/common/Point2f.hpp>
#include <depthai/depthai.hpp>
#include <depthai_ctrl/depthaiUtility.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <tuple>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

namespace dai
{

namespace ros
{

namespace StdMsgs = std_msgs::msg;
namespace ImageMsgs = sensor_msgs::msg;
using ImagePtr = ImageMsgs::Image::SharedPtr;

using TimePoint = std::chrono::time_point<std::chrono::steady_clock,
    std::chrono::steady_clock::duration>;
ImageMsgs::CameraInfo calibrationToCameraInfo(
  dai::CalibrationHandler calibHandler,
  dai::CameraBoardSocket cameraId,
  int width = -1,
  int height = -1,
  Point2f topLeftPixelId = Point2f(),
  Point2f bottomRightPixelId = Point2f());
class ImageConverter
{
public:
  // ImageConverter() = default;
  ImageConverter(const std::string frameName, bool interleaved);
  ~ImageConverter();
  ImageConverter(bool interleaved);
  void toRosMsgFromBitStream(
    std::shared_ptr<dai::ImgFrame> inData,
    std::deque<ImageMsgs::Image> & outImageMsgs,
    dai::RawImgFrame::Type type,
    const sensor_msgs::msg::CameraInfo & info);

  void toRosMsg(std::shared_ptr<dai::ImgFrame> inData, std::deque<ImageMsgs::Image> & outImageMsgs);
  ImagePtr toRosMsgPtr(std::shared_ptr<dai::ImgFrame> inData);

  void toDaiMsg(const ImageMsgs::Image & inMsg, dai::ImgFrame & outData);

  /** TODO(sachin): Add support for ros msg to cv mat since we have some
   *  encodings which cv supports but ros doesn't
   **/
  cv::Mat rosMsgtoCvMat(ImageMsgs::Image & inMsg);

  ImageMsgs::CameraInfo calibrationToCameraInfo(
    dai::CalibrationHandler calibHandler,
    dai::CameraBoardSocket cameraId,
    int width = -1,
    int height = -1,
    Point2f topLeftPixelId = Point2f(),
    Point2f bottomRightPixelId = Point2f());

private:
  static std::unordered_map<dai::RawImgFrame::Type, std::string> encodingEnumMap;
  static std::unordered_map<dai::RawImgFrame::Type, std::string> planarEncodingEnumMap;

  // dai::RawImgFrame::Type _srcType;
  const std::string _frameName = "";
  bool _daiInterleaved;
  // bool c
  void planarToInterleaved(
    const std::vector<uint8_t> & srcData, std::vector<uint8_t> & destData,
    int w, int h, int numPlanes, int bpp);
  void interleavedToPlanar(
    const std::vector<uint8_t> & srcData, std::vector<uint8_t> & destData,
    int w, int h, int numPlanes, int bpp);
  std::chrono::time_point<std::chrono::steady_clock> _steadyBaseTime;

  rclcpp::Time _rosBaseTime;
};

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
