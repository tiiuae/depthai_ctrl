#ifndef FOG_SW_DEPTHAI_CAMERA_H
#define FOG_SW_DEPTHAI_CAMERA_H
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <depthai/device/Device.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/node/ColorCamera.hpp>
#include <depthai/pipeline/node/MonoCamera.hpp>
#include <depthai/pipeline/node/VideoEncoder.hpp>
#include <depthai/pipeline/node/XLinkIn.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>
#include <depthai/utility/Initialization.hpp>
//restore compiler switches
#pragma GCC diagnostic pop
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>

namespace depthai_ctrl
{

/// ROS2 Node which works as DepthAI Camera driver
/// Reads camera frames directly fromn the camera and publishes corresponding ROS2 messages
/// It also publishes color camera video stream
class DepthAICamera : public rclcpp::Node
{
public:
  using ImageMsg = sensor_msgs::msg::Image;
  using CompressedImageMsg = sensor_msgs::msg::CompressedImage;

  DepthAICamera()
  : Node("depthai_camera"),
    _videoWidth(1280),
    _videoHeight(720),
    _videoFps(25),
    _videoBitrate(3000000),
    _videoH265(false),
    _useMonoCams(false),
    _useRawColorCam(false),
    _useUSB3(false),
    _thread_running(false),
    _left_camera_frame("left_camera_frame"),
    _right_camera_frame("right_camera_frame"),
    _color_camera_frame("color_camera_frame"),
    _leftCamCallback(0),
    _rightCamCallback(0),
    _colorCamCallback(0),
    _videoEncoderCallback(0)
  {
    Initialize();
    TryRestarting();
  }

  DepthAICamera(const rclcpp::NodeOptions & options)
  : Node("depthai_camera", options),
    _videoWidth(1280),
    _videoHeight(720),
    _videoFps(25),
    _videoBitrate(3000000),
    _videoH265(false),
    _useMonoCams(false),
    _useRawColorCam(false),
    _useUSB3(false),
    _thread_running(false),
    _left_camera_frame("left_camera_frame"),
    _right_camera_frame("right_camera_frame"),
    _color_camera_frame("color_camera_frame"),
    _leftCamCallback(0),
    _rightCamCallback(0),
    _colorCamCallback(0),
    _videoEncoderCallback(0)
  {

    Initialize();
    TryRestarting();
  }

  ~DepthAICamera()
  {
    Stop();
  }

  bool IsNodeRunning() {return bool(_device) && !_device->isClosed() && _thread_running;}

  void Stop()
  {
    // TODO, maybe remove callbacks?
    if (bool(_device)) {
      _device->close();
    }
  }

  void TryRestarting();

private:
  void ProcessingThread();

  void onLeftCamCallback(const std::shared_ptr<dai::ADatatype> data);
  void onRightCallback(const std::shared_ptr<dai::ADatatype> data);
  void onColorCamCallback(const std::shared_ptr<dai::ADatatype> data);
  void onVideoEncoderCallback(const std::shared_ptr<dai::ADatatype> data);
  std::shared_ptr<ImageMsg> ConvertImage(std::shared_ptr<dai::ImgFrame>, const std::string &);
  void Initialize();
  void VideoStreamCommand(std_msgs::msg::String::SharedPtr);

  std::shared_ptr<dai::Device> _device;
  std::shared_ptr<dai::Pipeline> _pipeline;
  std::shared_ptr<dai::DataOutputQueue> _videoQueue;
  std::shared_ptr<dai::DataOutputQueue> _leftQueue;
  std::shared_ptr<dai::DataOutputQueue> _rightQueue;
  std::shared_ptr<dai::DataOutputQueue> _colorQueue;
  std::shared_ptr<dai::DataInputQueue> _colorCamInputQueue;

  int _videoWidth;
  int _videoHeight;
  int _videoFps;
  int _videoBitrate;
  bool _videoH265;
  bool _useMonoCams;
  bool _useRawColorCam;
  bool _useUSB3;
  rclcpp::Time _lastFrameTime;
  long int _lastFrameTimePoint;

  std::shared_ptr<rclcpp::Publisher<ImageMsg>> _left_publisher;
  std::shared_ptr<rclcpp::Publisher<ImageMsg>> _right_publisher;
  std::shared_ptr<rclcpp::Publisher<ImageMsg>> _color_publisher;
  std::shared_ptr<rclcpp::Publisher<CompressedImageMsg>> _video_publisher;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _stream_command_subscriber;

  std::atomic<bool> _thread_running;
  std::string _left_camera_frame, _right_camera_frame, _color_camera_frame;
  dai::DataOutputQueue::CallbackId _leftCamCallback, _rightCamCallback, _colorCamCallback,
    _videoEncoderCallback;

  std::unordered_map<dai::RawImgFrame::Type, std::string> encodingEnumMap = {
    {dai::RawImgFrame::Type::YUV422i, "yuv422"},
    {dai::RawImgFrame::Type::RGBA8888, "rgba8"},
    {dai::RawImgFrame::Type::RGB888i, "rgb8"},
    {dai::RawImgFrame::Type::BGR888i, "bgr8"},
    {dai::RawImgFrame::Type::GRAY8, "mono8"},
    {dai::RawImgFrame::Type::RAW8, "8UC1"},
    {dai::RawImgFrame::Type::RAW16, "16UC1"}};

};

}  // namespace depthai_ctrl

#endif  // FOG_SW_DEPTHAI_CAMERA_H
