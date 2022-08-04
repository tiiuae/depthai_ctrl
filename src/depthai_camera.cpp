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

#include "depthai_camera.h"
#include "depthai_utils.h"
#include <nlohmann/json.hpp>

using namespace depthai_ctrl;

using std::placeholders::_1;
using std::placeholders::_2;
using Profile = dai::VideoEncoderProperties::Profile;

using std::chrono::duration_cast;
using std::chrono::nanoseconds;
using std::chrono::seconds;

void DepthAICamera::Initialize()
{
  RCLCPP_INFO(get_logger(), "[%s]: Initializing...", get_name());
  declare_parameter<std::string>("left_camera_topic", "camera/left/image_raw");
  declare_parameter<std::string>("right_camera_topic", "camera/right/image_raw");
  declare_parameter<std::string>("color_camera_topic", "camera/color/image_raw");
  declare_parameter<std::string>("video_stream_topic", "camera/color/video");
  declare_parameter<std::string>("stream_control_topic", "camera/videostreamcmd");

  const std::string left_camera_topic = get_parameter("left_camera_topic").as_string();
  const std::string right_camera_topic = get_parameter("right_camera_topic").as_string();
  const std::string color_camera_topic = get_parameter("color_camera_topic").as_string();
  const std::string video_stream_topic = get_parameter("video_stream_topic").as_string();
  const std::string stream_control_topic = get_parameter("stream_control_topic").as_string();

  _left_publisher = create_publisher<ImageMsg>(left_camera_topic, rclcpp::SensorDataQoS());
  _right_publisher = create_publisher<ImageMsg>(right_camera_topic, rclcpp::SensorDataQoS());
  _color_publisher = create_publisher<ImageMsg>(color_camera_topic, rclcpp::SensorDataQoS());
  _video_publisher = create_publisher<CompressedImageMsg>(
    video_stream_topic,
    rclcpp::SystemDefaultsQoS());
  _stream_command_subscriber = create_subscription<std_msgs::msg::String>(
    stream_control_topic, rclcpp::QoS(10).reliable(),
    std::bind(&DepthAICamera::VideoStreamCommand, this, _1));

  _auto_focus_timer =
    this->create_wall_timer(
    std::chrono::duration<double>(10.0),
    std::bind(&DepthAICamera::AutoFocusTimer, this));

  // Video Stream parameters
  declare_parameter<std::string>("encoding", "H264");
  declare_parameter<int>("width", 1280);
  declare_parameter<int>("height", 720);
  declare_parameter<int>("fps", 25);
  declare_parameter<int>("bitrate", 3000000);
  declare_parameter<int>("lens_position", 110);
  declare_parameter<bool>("use_mono_cams", false);
  declare_parameter<bool>("use_raw_color_cam", false);
  declare_parameter<bool>("use_auto_focus", false);
  declare_parameter<bool>("use_usb_three", false);

  _videoWidth = get_parameter("width").as_int();
  _videoHeight = get_parameter("height").as_int();
  _videoFps = get_parameter("fps").as_int();
  _videoBitrate = get_parameter("bitrate").as_int();
  _videoLensPosition = get_parameter("lens_position").as_int();
  _videoH265 = (get_parameter("encoding").as_string() == "H265");
  _useMonoCams = get_parameter("use_mono_cams").as_bool();
  _useRawColorCam = get_parameter("use_raw_color_cam").as_bool();
  _useAutoFocus = get_parameter("use_auto_focus").as_bool();
  // USB2 can only handle one H264 stream from camera. Adding raw camera or mono cameras will
  // cause dropped messages and unstable latencies between frames. When using USB3, we can
  // support multiple streams without any bandwidth issues.
  _useUSB3 = get_parameter("use_usb_three").as_bool();
  _lastFrameTime = rclcpp::Time(0);
}

void DepthAICamera::AutoFocusTimer()
{
  if (_firstFrameReceived) {
    RCLCPP_INFO(
      get_logger(), "[%s]: Change focus mode to %s",
           get_name(), _useAutoFocus ? "auto" : "manual");
    if (!_useAutoFocus){
      RCLCPP_INFO(
        get_logger(), "[%s]: Manual focus lens position %d",
            get_name(), _videoLensPosition);
    }
    changeFocusMode(_useAutoFocus);
    _auto_focus_timer->cancel();
  }
}

void DepthAICamera::VideoStreamCommand(std_msgs::msg::String::SharedPtr msg)
{
  nlohmann::json cmd{};
  try {
    cmd = nlohmann::json::parse(msg->data.c_str());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Error while parsing JSON string from VideoCommand");
    return;
  }
  if (!cmd["Command"].empty()) {
    std::string command = cmd["Command"];
    std::transform(
      command.begin(), command.end(), command.begin(),
      [](unsigned char c) {return std::tolower(c);});
    if (command == "start" && !_thread_running) {
      int width = _videoWidth;
      int height = _videoHeight;
      int fps = _videoFps;
      int bitrate = _videoBitrate;

      int videoLensPosition = _videoLensPosition;
      std::string encoding = _videoH265 ? "H265" : "H264";
      std::string error_message{};
      bool useMonoCams = _useMonoCams;
      bool useRawColorCam = _useRawColorCam;
      bool useAutoFocus = _useAutoFocus;

      if (!cmd["Width"].empty() && cmd["Width"].is_number_integer()) {
        nlohmann::from_json(cmd["Width"], width);
      }
      if (!cmd["Height"].empty() && cmd["Height"].is_number_integer()) {
        nlohmann::from_json(cmd["Height"], height);
      }
      if (!cmd["Fps"].empty() && cmd["Fps"].is_number_integer()) {
        nlohmann::from_json(cmd["Fps"], fps);
      }
      if (!cmd["Bitrate"].empty() && cmd["Bitrate"].is_number_integer()) {
        nlohmann::from_json(cmd["Bitrate"], bitrate);
      }
      if (!cmd["Encoding"].empty() && cmd["Encoding"].is_string()) {
        nlohmann::from_json(cmd["Encoding"], encoding);
      }
      if (!cmd["UseMonoCams"].empty() && cmd["UseMonoCams"].is_string()) {
        nlohmann::from_json(cmd["UseMonoCams"], useMonoCams);
      }
      if (!cmd["UseAutoFocus"].empty() && cmd["UseAutoFocus"].is_boolean()) {
        nlohmann::from_json(cmd["UseAutoFocus"], useAutoFocus);
      }
      if (!cmd["LensPosition"].empty() && cmd["LensPosition"].is_number_integer()) {
        nlohmann::from_json(cmd["LensPosition"], videoLensPosition);
      }

      if (DepthAIUtils::ValidateCameraParameters(
          width, height, fps, bitrate, videoLensPosition, encoding,
          error_message))
      {
        _videoWidth = width;
        _videoHeight = height;
        _videoFps = fps;
        _videoBitrate = bitrate;
        _videoLensPosition = videoLensPosition;
        _videoH265 = (encoding == "H265");
        _useMonoCams = useMonoCams;
        _useRawColorCam = useRawColorCam;
        _useAutoFocus = useAutoFocus;

        _auto_focus_timer->reset();
        _auto_focus_timer =
          this->create_wall_timer(
          std::chrono::duration<double>(10.0),
          std::bind(&DepthAICamera::AutoFocusTimer, this));
        TryRestarting();
      } else {
        RCLCPP_ERROR(this->get_logger(), error_message.c_str());
      }
    }
    if (command == "change_focus" && _thread_running) {
      bool useAutoFocus = _useAutoFocus;
      if (!cmd["UseAutoFocus"].empty() && cmd["UseAutoFocus"].is_boolean()) {
        nlohmann::from_json(cmd["UseAutoFocus"], useAutoFocus);
      }
      if (_useAutoFocus != useAutoFocus) {
        _useAutoFocus = useAutoFocus;
        changeFocusMode(_useAutoFocus);
        RCLCPP_INFO(
          this->get_logger(), "Change focus mode to %s",
          _useAutoFocus ? "auto" : "manual");
      }
      if (useAutoFocus) {
        RCLCPP_ERROR(this->get_logger(), "Cannot change focus while auto focus is enabled");
      } else {

        int videoLensPosition = get_parameter("lens_position").as_int();

        if (!cmd["LensPosition"].empty() && cmd["LensPosition"].is_number_integer()) {
          nlohmann::from_json(cmd["LensPosition"], videoLensPosition);
          RCLCPP_INFO(this->get_logger(), "Received lens position cmd of %d", videoLensPosition);
        }
        if (videoLensPosition >= 0 || videoLensPosition <= 255) {
          RCLCPP_INFO(this->get_logger(), "Changing focus to %d", videoLensPosition);
          _videoLensPosition = videoLensPosition;
          changeLensPosition(_videoLensPosition);
        } else {
          RCLCPP_ERROR(
            this->get_logger(), "Required video stream 'lens_position' is incorrect.\
            Valid range is 0-255");
        }
      }

    }
    if (command == "stop") {
      if (!_thread_running) {
        RCLCPP_ERROR(this->get_logger(), "The video stream is not running");
        return;
      } 
      RCLCPP_INFO(this->get_logger(), "Stopping video stream");
      _firstFrameReceived = false;
      Stop();
    }
  }
}

void DepthAICamera::TryRestarting()
{
  if (_thread_running) {
    _thread_running = false;
  }

  RCLCPP_INFO(this->get_logger(), "[%s]: (Re)Starting...", get_name());

  _pipeline = std::make_shared<dai::Pipeline>();

  // Using mono cameras adds additional CPU consumption, therefore it is disabled by default
  if (_useMonoCams) {
    auto monoLeft = _pipeline->create<dai::node::MonoCamera>();
    auto monoRight = _pipeline->create<dai::node::MonoCamera>();
    auto xoutLeft = _pipeline->create<dai::node::XLinkOut>();
    auto xoutRight = _pipeline->create<dai::node::XLinkOut>();
    // Setup Grayscale Cameras
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    monoLeft->out.link(xoutLeft->input);
    monoRight->out.link(xoutRight->input);
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
  }
  auto colorCamera = _pipeline->create<dai::node::ColorCamera>();
  auto videoEncoder = _pipeline->create<dai::node::VideoEncoder>();

  auto xoutVideo = _pipeline->create<dai::node::XLinkOut>();
  xoutVideo->setStreamName("enc26xColor");
  // Setup Color Camera
  colorCamera->setBoardSocket(dai::CameraBoardSocket::RGB);
  colorCamera->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);

  // Preview resolution cannot be larger than Video's, thus resolution color camera image is limited
  colorCamera->setPreviewSize(_videoWidth, _videoHeight);
  colorCamera->setVideoSize(_videoWidth, _videoHeight);
  colorCamera->setFps(_videoFps);

  // Like mono cameras, color camera is disabled by default to reduce computational load.
  if (_useRawColorCam) {
    auto xoutColor = _pipeline->create<dai::node::XLinkOut>();
    xoutColor->setStreamName("color");
    colorCamera->preview.link(xoutColor->input);
  }

  Profile encoding = _videoH265 ? Profile::H265_MAIN : Profile::H264_MAIN;
  videoEncoder->setDefaultProfilePreset(_videoWidth, _videoHeight, _videoFps, encoding);
  videoEncoder->setBitrate(_videoBitrate);
  RCLCPP_INFO(
    this->get_logger(), "[%s]: VideoEncoder FPS: %f",
    get_name(), videoEncoder->getFrameRate());

  colorCamera->video.link(videoEncoder->input);
  videoEncoder->bitstream.link(xoutVideo->input);
  auto xinColor = _pipeline->create<dai::node::XLinkIn>();
  xinColor->setStreamName("colorCamCtrl");

  xinColor->out.link(colorCamera->inputControl);
  RCLCPP_INFO(this->get_logger(), "[%s]: Initializing DepthAI camera...", get_name());
  for (int i = 0; i < 5 && !_device; i++) {
    try {
      _device = std::make_shared<dai::Device>(*_pipeline, !_useUSB3);
    } catch (const std::runtime_error & err) {
      RCLCPP_ERROR(get_logger(), "Cannot start DepthAI camera: %s", err.what());
      _device.reset();
    }
  }
  if (!_device) {
    return;
  }

  std::string usbSpeed;
  switch (_device->getUsbSpeed()) {
    case dai::UsbSpeed::UNKNOWN:
      usbSpeed = "Unknown";
      break;
    case dai::UsbSpeed::LOW:
      usbSpeed = "Low";
      break;
    case dai::UsbSpeed::FULL:
      usbSpeed = "Full";
      break;
    case dai::UsbSpeed::HIGH:
      usbSpeed = "High";
      break;
    case dai::UsbSpeed::SUPER:
      usbSpeed = "Super";
      break;
    case dai::UsbSpeed::SUPER_PLUS:
      usbSpeed = "SuperPlus";
      break;
    default:
      usbSpeed = "Not valid";
      break;
  }
  RCLCPP_INFO(
    this->get_logger(), "[%s]: DepthAI Camera USB Speed: %s", get_name(),
    usbSpeed.c_str());

  //_device->startPipeline();
  _colorCamInputQueue = _device->getInputQueue("colorCamCtrl");
  dai::CameraControl colorCamCtrl;
  if (_useAutoFocus) {
    colorCamCtrl.setAutoFocusMode(dai::RawCameraControl::AutoFocusMode::CONTINUOUS_VIDEO);
  } else {
    colorCamCtrl.setAutoFocusMode(dai::RawCameraControl::AutoFocusMode::OFF);
    colorCamCtrl.setManualFocus(_videoLensPosition);
  }

  _colorCamInputQueue->send(colorCamCtrl);

  if (_useRawColorCam) {
    _colorQueue = _device->getOutputQueue("color", 30, false);
    _colorCamCallback =
      _colorQueue->addCallback(
      std::bind(
        &DepthAICamera::onColorCamCallback, this,
        std::placeholders::_1));
  }
  _videoQueue = _device->getOutputQueue("enc26xColor", 30, true);
  if (_useMonoCams) {
    _leftQueue = _device->getOutputQueue("left", 30, false);
    _rightQueue = _device->getOutputQueue("right", 30, false);

    _leftCamCallback =
      _leftQueue->addCallback(
      std::bind(
        &DepthAICamera::onLeftCamCallback, this,
        std::placeholders::_1));
    _rightCamCallback =
      _rightQueue->addCallback(
      std::bind(
        &DepthAICamera::onRightCallback, this,
        std::placeholders::_1));
  }
  _thread_running = true;

  _videoEncoderCallback =
    _videoQueue->addCallback(
    std::bind(
      &DepthAICamera::onVideoEncoderCallback, this,
      std::placeholders::_1));

}

void DepthAICamera::changeLensPosition(int lens_position)
{
  if (!_device) {
    return;
  }
  dai::CameraControl colorCamCtrl;
  colorCamCtrl.setAutoFocusMode(dai::RawCameraControl::AutoFocusMode::OFF);
  colorCamCtrl.setManualFocus(lens_position);
  _colorCamInputQueue->send(colorCamCtrl);
}

void DepthAICamera::changeFocusMode(bool use_auto_focus)
{
  if (!_device) {
    return;
  }
  dai::CameraControl colorCamCtrl;
  if (use_auto_focus) {
    colorCamCtrl.setAutoFocusMode(dai::RawCameraControl::AutoFocusMode::CONTINUOUS_VIDEO);
  } else {
    colorCamCtrl.setAutoFocusMode(dai::RawCameraControl::AutoFocusMode::OFF);
    colorCamCtrl.setManualFocus(_videoLensPosition);
  }
  _colorCamInputQueue->send(colorCamCtrl);
}

void DepthAICamera::onLeftCamCallback(
  const std::shared_ptr<dai::ADatatype> data)
{
	_framesReceivedCounter->Increment();

  (void)data; // Using this pointer does not pop from queue, so we don't need to do anything with it.
  std::vector<std::shared_ptr<dai::ImgFrame>> leftPtrVector =
    _leftQueue->tryGetAll<dai::ImgFrame>();
  RCLCPP_DEBUG(
    this->get_logger(), "[%s]: Received %ld left camera frames...",
    get_name(), leftPtrVector.size());
  for (std::shared_ptr<dai::ImgFrame> & leftPtr : leftPtrVector) {
    auto image = ConvertImage(leftPtr, _left_camera_frame);
    _left_publisher->publish(*image);
  }

}

void DepthAICamera::onRightCallback(
  const std::shared_ptr<dai::ADatatype> data)
{
	_framesReceivedCounter->Increment();

  (void)data;
  std::vector<std::shared_ptr<dai::ImgFrame>> rightPtrVector =
    _rightQueue->tryGetAll<dai::ImgFrame>();
  RCLCPP_DEBUG(
    this->get_logger(), "[%s]: Received %ld right camera frames...",
    get_name(), rightPtrVector.size());
  for (std::shared_ptr<dai::ImgFrame> & rightPtr : rightPtrVector) {
    auto image = ConvertImage(rightPtr, _right_camera_frame);
    _right_publisher->publish(*image);
  }
}

void DepthAICamera::onColorCamCallback(
  const std::shared_ptr<dai::ADatatype> data)
{
  (void)data;
  std::vector<std::shared_ptr<dai::ImgFrame>> colorPtrVector =
    _colorQueue->tryGetAll<dai::ImgFrame>();
  RCLCPP_DEBUG(
    this->get_logger(), "[%s]: Received %ld color camera frames...",
    get_name(), colorPtrVector.size());
  for (std::shared_ptr<dai::ImgFrame> & colorPtr : colorPtrVector) {
    auto image = ConvertImage(colorPtr, _color_camera_frame);
    _color_publisher->publish(*image);
  }
}


void DepthAICamera::onVideoEncoderCallback(
  const std::shared_ptr<dai::ADatatype> data)
{
  (void)data;
  std::vector<std::shared_ptr<dai::ImgFrame>> videoPtrVector =
    _videoQueue->tryGetAll<dai::ImgFrame>();
  RCLCPP_DEBUG(
    this->get_logger(), "[%s]: Received %ld video frames...",
    get_name(), videoPtrVector.size());
  for (std::shared_ptr<dai::ImgFrame> & videoPtr : videoPtrVector) {

    /*
      Old implementation uses getTimestamp, which had a bug where the time is not correct when run at boot.
      getTimestamp is host syncronized and supposed to give the time in host clock.
      However, since the DepthAI camera is starting its boot at the same time as the host,
      The syncronization is not working properly as it tries to syncronize the camera clock with the host.
      Therefore, we use the getTimestampDevice() to get direct device time.
      This implementation will work without any problems for the H264 video streaming.
      However, a host syncronized time is needed for the raw color camera, when doing camera based navigation.
      Otherwise, the time drifts will cause wrong estimations and tracking will be unstable.

      It is also possible to use SequenceNumber for timestamp calculation, and it also works for H264 streaming.
      However, it might still be problematic with the raw color camera. It will be investigated later.
    */
    //const auto stamp = videoPtr->getTimestamp().time_since_epoch().count();
    const auto stamp = videoPtr->getTimestampDevice().time_since_epoch().count();
    //const auto seq = videoPtr->getSequenceNum();
    //int64_t stamp = (int64_t)seq * (1e9/_videoFps); // Use sequence number for timestamp

    CompressedImageMsg video_stream_chunk{};
    video_stream_chunk.header.frame_id = _color_camera_frame;

    // rclcpp::Time can be initialized directly with nanoseconds only.
    // Internally, when given with seconds and nanoseconds, it casts it to nanoseconds anyways.
    video_stream_chunk.header.stamp = rclcpp::Time(stamp, RCL_STEADY_TIME);
    video_stream_chunk.data.swap(videoPtr->getData());
    video_stream_chunk.format = _videoH265 ? "H265" : "H264";
    _video_publisher->publish(video_stream_chunk);
    if (!_firstFrameReceived) {
      _firstFrameReceived = true;
      RCLCPP_INFO(
        this->get_logger(), "[%s]: First frame received!",
        get_name());
    }
  }
}

std::shared_ptr<DepthAICamera::ImageMsg> DepthAICamera::ConvertImage(
  const std::shared_ptr<dai::ImgFrame> input,
  const std::string & frame_id)
{
  auto message = std::make_shared<ImageMsg>();
  const auto stamp = input->getTimestamp();
  const int32_t sec = duration_cast<seconds>(stamp.time_since_epoch()).count();
  const int32_t nsec = duration_cast<nanoseconds>(stamp.time_since_epoch()).count() % 1000000000UL;

  message->header.stamp = rclcpp::Time(sec, nsec, RCL_STEADY_TIME);
  message->header.frame_id = frame_id;

  if (encodingEnumMap.find(input->getType()) != encodingEnumMap.end()) {
    message->encoding = encodingEnumMap[input->getType()];
  }

  message->height = input->getHeight();
  message->width = input->getWidth();
  message->step = input->getData().size() / input->getHeight();
  message->data.swap(input->getData());
  return message;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ctrl::DepthAICamera)
