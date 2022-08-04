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
#include <depthai/depthai.hpp>
//restore compiler switches
#pragma GCC diagnostic pop
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>

#include <prometheus/counter.h>
#include <prometheus/exposer.h>
#include <prometheus/registry.h>

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
    _videoLensPosition(110),
    _videoH265(false),
    _useMonoCams(false),
    _useRawColorCam(false),
    _useAutoFocus(false),
    _useUSB3(false),
    _firstFrameReceived(false),
    _metricsExposer{"127.0.0.1:8080"},
    _metricsRegistry(std::make_shared<prometheus::Registry>()),
    _thread_running(false),
    _left_camera_frame("left_camera_frame"),
    _right_camera_frame("right_camera_frame"),
    _color_camera_frame("color_camera_frame"),
    _leftCamCallback(0),
    _rightCamCallback(0),
    _colorCamCallback(0),
    _videoEncoderCallback(0)
  {
  	initMetrics();
    Initialize();
    TryRestarting();
  }

  DepthAICamera(const rclcpp::NodeOptions & options)
  : Node("depthai_camera", options),
    _videoWidth(1280),
    _videoHeight(720),
    _videoFps(25),
    _videoBitrate(3000000),
    _videoLensPosition(110),
    _videoH265(false),
    _useMonoCams(false),
    _useRawColorCam(false),
    _useAutoFocus(false),
    _useUSB3(false),
    _firstFrameReceived(false),
    _metricsExposer{"127.0.0.1:8080"},
    _metricsRegistry(std::make_shared<prometheus::Registry>()),
    _thread_running(false),
    _left_camera_frame("left_camera_frame"),
    _right_camera_frame("right_camera_frame"),
    _color_camera_frame("color_camera_frame"),
    _leftCamCallback(0),
    _rightCamCallback(0),
    _colorCamCallback(0),
    _videoEncoderCallback(0)
  {
  	initMetrics();
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
    if (_thread_running && bool(_device))
    {
      _thread_running = false;
      _device.reset();
    }
  }

  void TryRestarting();

private:
	void initMetrics()
	{
		// ask the exposer to scrape the registry on incoming HTTP requests
		_metricsExposer.RegisterCollectable(_metricsRegistry);

		_framesReceivedCounter = &(prometheus::BuildCounter()
					.Name("frames_received_count")
					.Help("Number of video frames received from DepthAI camera")
					.Register(*_metricsRegistry).Add({{"direction", "inbound"}}));
	}

  void ProcessingThread();
  void AutoFocusTimer();
  void changeLensPosition(int lens_position);
  void changeFocusMode(bool use_auto_focus);
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
  int _videoLensPosition;
  bool _videoH265;
  bool _useMonoCams;
  bool _useRawColorCam;
  bool _useAutoFocus;
  bool _useUSB3;
  bool _firstFrameReceived;
  prometheus::Exposer _metricsExposer;
  std::shared_ptr<prometheus::Registry> _metricsRegistry;
  prometheus::Counter* _framesReceivedCounter;
  rclcpp::Time _lastFrameTime;

  std::shared_ptr<rclcpp::Publisher<ImageMsg>> _left_publisher;
  std::shared_ptr<rclcpp::Publisher<ImageMsg>> _right_publisher;
  std::shared_ptr<rclcpp::Publisher<ImageMsg>> _color_publisher;
  std::shared_ptr<rclcpp::Publisher<CompressedImageMsg>> _video_publisher;

  rclcpp::TimerBase::SharedPtr     _auto_focus_timer;
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
