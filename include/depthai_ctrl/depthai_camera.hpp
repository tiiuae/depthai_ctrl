/*******************************************************************************
* Copyright 2021 Unikie Oy, Technology Innovation Institute
* All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors(Unikie Oy): Mehmet Killioglu, Manuel Segarra-Abad, Sergey */

#ifndef FOG_SW_DEPTHAI_CAMERA_H
#define FOG_SW_DEPTHAI_CAMERA_H
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include <depthai/device/Device.hpp>
#include <depthai/depthai.hpp>
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
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cv_bridge/cv_bridge.h>


#include <camera_info_manager/camera_info_manager.hpp>

#include <depthai_ctrl/ImageConverter.hpp>
#include <depthai_ctrl/DisparityConverter.hpp>
#include <depthai_ctrl/ImgDetectionConverter.hpp>


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
  using Ptr = std::shared_ptr<depthai_ctrl::DepthAICamera>;
  DepthAICamera()
  : Node("camera_node"),
    _videoWidth(1280),
    _videoHeight(720),
    _videoFps(25),
    _videoBitrate(3000000),
    _videoLensPosition(110),
    _videoH265(false),
    _useMonoCams(false),
    _useRawColorCam(false),
    _useDepth(false),
    _useVideoFromColorCam(true),
    _useAutoFocus(false),
    _useUSB3(false),
    _firstFrameReceived(false),
    _useNeuralNetwork(false),
    _syncNN(true),
    _thread_running(false),
    _left_camera_frame("left_camera_frame"),
    _right_camera_frame("right_camera_frame"),
    _color_camera_frame("color_camera_frame"),
    _nn_directory("tiny-yolo-v4_openvino_2021.2_6shave.blob"),
    _cameraName("oak"),
    _leftCamCallback(0),
    _rightCamCallback(0),
    _colorCamCallback(0),
    _depthCallback(0),
    _videoEncoderCallback(0),
    _passthroughCallback(0)
  {
    Initialize();
    TryRestarting();
  }

  DepthAICamera(const rclcpp::NodeOptions & options)
  : Node("camera_node", options),
    _videoWidth(1280),
    _videoHeight(720),
    _videoFps(25),
    _videoBitrate(3000000),
    _videoLensPosition(110),
    _videoH265(false),
    _useMonoCams(false),
    _useRawColorCam(false),
    _useDepth(false),
    _useVideoFromColorCam(true),
    _useAutoFocus(false),
    _useUSB3(false),
    _firstFrameReceived(false),
    _useNeuralNetwork(false),
    _syncNN(true),
    _thread_running(false),
    _left_camera_frame("left_camera_frame"),
    _right_camera_frame("right_camera_frame"),
    _color_camera_frame("color_camera_frame"),
    _nn_directory("tiny-yolo-v4_openvino_2021.2_6shave.blob"),
    _cameraName("oak"),
    _leftCamCallback(0),
    _rightCamCallback(0),
    _colorCamCallback(0),
    _depthCallback(0),
    _videoEncoderCallback(0),
    _passthroughCallback(0)
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
    if (_thread_running && bool(_device)) {
      _thread_running = false;
      _device.reset();
    }
  }

  void TryRestarting();

private:
  void ProcessingThread();
  void AutoFocusTimer();
  void changeLensPosition(int lens_position);
  void changeFocusMode(bool use_auto_focus);
  void onLeftCamCallback(const std::shared_ptr<dai::ADatatype> data);
  void onRightCallback(const std::shared_ptr<dai::ADatatype> data);
  void onColorCamCallback(const std::shared_ptr<dai::ADatatype> data);
  void onDepthCallback(const std::shared_ptr<dai::ADatatype> data);
  void onPassthroughCallback(const std::shared_ptr<dai::ADatatype> data);
  void onVideoEncoderCallback(const std::shared_ptr<dai::ADatatype> data);
  void onNeuralNetworkCallback(const std::shared_ptr<dai::ADatatype> data);
  std::shared_ptr<ImageMsg> ConvertImage(std::shared_ptr<dai::ImgFrame>, const std::string &);
  void Initialize();
  void VideoStreamCommand(std_msgs::msg::String::SharedPtr);

  std::shared_ptr<dai::Device> _device;
  std::shared_ptr<dai::Pipeline> _pipeline;
  std::shared_ptr<dai::DataOutputQueue> _videoQueue;
  std::shared_ptr<dai::DataOutputQueue> _leftQueue;
  std::shared_ptr<dai::DataOutputQueue> _rightQueue;
  std::shared_ptr<dai::DataOutputQueue> _colorQueue;
  std::shared_ptr<dai::DataOutputQueue> _depthQueue;
  std::shared_ptr<dai::DataOutputQueue> _passthroughQueue;
  std::shared_ptr<dai::DataInputQueue> _colorCamInputQueue;
  std::shared_ptr<dai::DataOutputQueue> _neuralNetworkOutputQueue;

  int _videoWidth;
  int _videoHeight;
  int _videoFps;
  int _videoBitrate;
  int _videoLensPosition;
  bool _videoH265;
  bool _useMonoCams;
  bool _useRawColorCam;
  bool _useDepth;
  bool _useVideoFromColorCam;
  bool _useAutoFocus;
  bool _useUSB3;
  bool _firstFrameReceived;
  bool _useNeuralNetwork;
  bool _syncNN;
  rclcpp::Time _lastFrameTime;

  std::shared_ptr<rclcpp::Publisher<ImageMsg>> _left_publisher;
  std::shared_ptr<rclcpp::Publisher<ImageMsg>> _right_publisher;
  std::shared_ptr<rclcpp::Publisher<ImageMsg>> _color_publisher;
  std::shared_ptr<rclcpp::Publisher<ImageMsg>> _depth_publisher;
  std::shared_ptr<rclcpp::Publisher<ImageMsg>> _passthrough_publisher;
  std::shared_ptr<rclcpp::Publisher<CompressedImageMsg>> _video_publisher;

  rclcpp::TimerBase::SharedPtr _auto_focus_timer;
  std::shared_ptr<rclcpp::Publisher<vision_msgs::msg::Detection2DArray>> _detection_roi_publisher;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _stream_command_subscriber;

  std::atomic<bool> _thread_running;
  std::string _left_camera_frame, _right_camera_frame, _color_camera_frame;
  std::string _nn_directory, _cameraName;
  dai::DataOutputQueue::CallbackId _leftCamCallback, _rightCamCallback, _colorCamCallback,
    _depthCallback, _videoEncoderCallback, _neuralNetworkCallback, _passthroughCallback;
  dai::CalibrationHandler _calibrationHandler;
  std::shared_ptr<dai::rosBridge::DisparityConverter> _depth_disparity_converter;
  std::shared_ptr<dai::rosBridge::ImageConverter> _color_camera_converter;
  std::shared_ptr<dai::rosBridge::ImageConverter> _left_camera_converter;
  std::shared_ptr<dai::rosBridge::ImageConverter> _right_camera_converter;
  std::shared_ptr<dai::rosBridge::ImageConverter> _passthrough_converter;
  std::shared_ptr<dai::rosBridge::ImgDetectionConverter> _neural_network_converter;
  std::unordered_map<dai::RawImgFrame::Type, std::string> encodingEnumMap = {
    {dai::RawImgFrame::Type::YUV422i, "yuv422"},
    {dai::RawImgFrame::Type::RGBA8888, "rgba8"},
    {dai::RawImgFrame::Type::RGB888i, "rgb8"},
    {dai::RawImgFrame::Type::BGR888i, "bgr8"},
    {dai::RawImgFrame::Type::GRAY8, "mono8"},
    {dai::RawImgFrame::Type::RAW8, "8UC1"},
    {dai::RawImgFrame::Type::RAW16, "16UC1"}};

  std::unordered_map<dai::RawImgFrame::Type, std::string> planarEncodingEnumMap = {
    {dai::RawImgFrame::Type::BGR888p, "rgb8"},  // 3_1_bgr8 represents 3 planes/channels and 1 byte per pixel in BGR format
    {dai::RawImgFrame::Type::RGB888p, "rgb8"},
    {dai::RawImgFrame::Type::NV12, "rgb8"},
    {dai::RawImgFrame::Type::YUV420p, "rgb8"}};

  std::unordered_map<dai::UsbSpeed, std::string> usbSpeedEnumMap = {
    {dai::UsbSpeed::UNKNOWN, "Unknown"},
    {dai::UsbSpeed::LOW, "Low"},
    {dai::UsbSpeed::FULL, "Full"},
    {dai::UsbSpeed::HIGH, "High"},
    {dai::UsbSpeed::SUPER, "Super"},
    {dai::UsbSpeed::SUPER_PLUS, "SuperPlus"}};
};

}  // namespace depthai_ctrl

#endif  // FOG_SW_DEPTHAI_CAMERA_H
