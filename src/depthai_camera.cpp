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

#include "depthai_ctrl/depthai_camera.hpp"
#include "depthai_ctrl/depthai_utils.h"
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
  /*declare_parameter<std::string>("left_camera_topic", "camera/left/image_raw");
  declare_parameter<std::string>("right_camera_topic", "camera/right/image_raw");
  declare_parameter<std::string>("color_camera_topic", "camera/color/image_raw");
  declare_parameter<std::string>("video_stream_topic", "camera/color/video");
  declare_parameter<std::string>("passthrough_topic", "camera/color/image_passthrough");
  declare_parameter<std::string>("stream_control_topic", "videostreamcmd");
  declare_parameter<std::string>("detection_roi_topic", "detections");*/
  declare_parameter<std::string>("nn_directory", "tiny-yolo-v4_openvino_2021.2_6shave.blob");
  declare_parameter<std::string>("camera_name", "oak");

  /*const std::string left_camera_topic = get_parameter("left_camera_topic").as_string();
  const std::string right_camera_topic = get_parameter("right_camera_topic").as_string();
  const std::string color_camera_topic = get_parameter("color_camera_topic").as_string();
  const std::string video_stream_topic = get_parameter("video_stream_topic").as_string();
  const std::string passthrough_topic = get_parameter("passthrough_topic").as_string();
  const std::string detection_roi_topic = get_parameter("detection_roi_topic").as_string();
  const std::string stream_control_topic = get_parameter("stream_control_topic").as_string();*/
  _nn_directory = get_parameter("nn_directory").as_string();

  _left_publisher = create_publisher<ImageMsg>("camera/left/image_raw", 10);
  _right_publisher = create_publisher<ImageMsg>("camera/right/image_raw", 10);
  _color_publisher = create_publisher<ImageMsg>("camera/color/image_raw", 10);
  _camera_info_publisher = create_publisher<CameraInfoMsg>("camera/color/camera_info", 10);
  _depth_publisher = create_publisher<ImageMsg>("camera/depth/image_raw", 10);
  _passthrough_publisher = create_publisher<ImageMsg>("camera/color/image_passthrough", 10);

  _detection_roi_publisher = create_publisher<vision_msgs::msg::Detection2DArray>(
    "camera/detections", 10);

  _video_publisher = create_publisher<CompressedImageMsg>(
    "camera/color/video", 10);

  _stream_command_subscriber = create_subscription<std_msgs::msg::String>(
    "camera/videostreamcmd", rclcpp::QoS(10).reliable(),
    std::bind(&DepthAICamera::VideoStreamCommand, this, _1));

  _auto_focus_timer =
    this->create_wall_timer(
    std::chrono::duration<double>(10.0),
    std::bind(&DepthAICamera::AutoFocusTimer, this));

  // Video Stream parameters
  declare_parameter<std::string>("encoding", "H264");
  declare_parameter<int>("width", 1280);
  declare_parameter<int>("height", 720);
  declare_parameter<double>("video_fps", 25.);
  declare_parameter<int>("bitrate", 3000000);
  declare_parameter<int>("lens_position", 110);
  declare_parameter<bool>("use_mono_cams", false);
  declare_parameter<bool>("use_raw_color_cam", false);
  declare_parameter<bool>("use_depth", false);
  declare_parameter<bool>("use_video_from_color_cam", true);
  declare_parameter<bool>("use_auto_focus", false);
  declare_parameter<bool>("use_usb_three", false);
  declare_parameter<bool>("use_neural_network", false);
  declare_parameter<bool>("use_passthrough_preview", false);

  _videoWidth = get_parameter("width").as_int();
  _videoHeight = get_parameter("height").as_int();
  _videoFps = get_parameter("video_fps").as_double();
  _videoBitrate = get_parameter("bitrate").as_int();
  _videoLensPosition = get_parameter("lens_position").as_int();
  _videoH265 = (get_parameter("encoding").as_string() == "H265");
  _useMonoCams = get_parameter("use_mono_cams").as_bool();
  _useRawColorCam = get_parameter("use_raw_color_cam").as_bool();
  _useDepth = get_parameter("use_depth").as_bool();
  _useVideoFromColorCam = get_parameter("use_video_from_color_cam").as_bool();
  _useAutoFocus = get_parameter("use_auto_focus").as_bool();
  _useNeuralNetwork = get_parameter("use_neural_network").as_bool();
  _syncNN = get_parameter("use_passthrough_preview").as_bool();
  _cameraName = get_parameter("camera_name").as_string();
  _left_camera_frame = _cameraName + "_left_camera_optical_frame";
  _right_camera_frame = _cameraName + "_right_camera_optical_frame";
  _color_camera_frame = _cameraName + "_rgb_camera_optical_frame";

  RCLCPP_INFO(get_logger(), "[%s]: Initialization complete.", get_name());
  RCLCPP_INFO(get_logger(), "[%s]: Video stream parameters:", get_name());
  RCLCPP_INFO(get_logger(), "[%s]:   Width: %d", get_name(), _videoWidth);
  RCLCPP_INFO(get_logger(), "[%s]:   Height: %d", get_name(), _videoHeight);
  RCLCPP_INFO(get_logger(), "[%s]:   Video FPS: %f", get_name(), _videoFps);
  RCLCPP_INFO(get_logger(), "[%s]:   Bitrate: %d", get_name(), _videoBitrate);
  RCLCPP_INFO(get_logger(), "[%s]:   Lens position: %d", get_name(), _videoLensPosition);
  RCLCPP_INFO(get_logger(), "[%s]:   H265: %s", get_name(), _videoH265 ? "true" : "false");
  RCLCPP_INFO(
    get_logger(), "[%s]:   Use mono cams: %s", get_name(),
    _useMonoCams ? "true" : "false");
  RCLCPP_INFO(
    get_logger(), "[%s]:   Use raw color cam: %s",
    get_name(), _useRawColorCam ? "true" : "false");
  RCLCPP_INFO(
    get_logger(), "[%s]:   Use depth : %s",
    get_name(), _useDepth ? "true" : "false");
  RCLCPP_INFO(
    get_logger(), "[%s]:   Use video from color cam: %s",
    get_name(), _useVideoFromColorCam ? "true" : "false");
  RCLCPP_INFO(
    get_logger(), "[%s]:   Use auto focus: %s",
    get_name(), _useAutoFocus ? "true" : "false");
  RCLCPP_INFO(
    get_logger(), "[%s]:   Use neural network: %s",
    get_name(), _useNeuralNetwork ? "true" : "false");
  RCLCPP_INFO(
    get_logger(), "[%s]:   Use passthrough preview: %s",
    get_name(), _syncNN ? "true" : "false");
  RCLCPP_INFO(get_logger(), "[%s]:   Camera name: %s", get_name(), _cameraName.c_str());
  RCLCPP_INFO(
    get_logger(), "[%s]:   Left camera frame: %s", get_name(),
    _left_camera_frame.c_str());
  RCLCPP_INFO(
    get_logger(), "[%s]:   Right camera frame: %s", get_name(),
    _right_camera_frame.c_str());
  RCLCPP_INFO(
    get_logger(), "[%s]:   Color camera frame: %s", get_name(),
    _color_camera_frame.c_str());

  if (_useNeuralNetwork) {
    RCLCPP_INFO(
      get_logger(), "[%s]: Using neural network, blob path %s",
      get_name(), _nn_directory.c_str());
  }

  // USB2 can only handle one H264 stream from camera. Adding raw camera or mono cameras will
  // cause dropped messages and unstable latencies between frames. When using USB3, we can
  // support multiple streams without any bandwidth issues.
  _useUSB3 = get_parameter("use_usb_three").as_bool();
  _lastFrameTime = rclcpp::Time(0);
  _rgb_camera_info = std::make_unique<CameraInfoMsg>();

  _change_paramaters_srv = create_service<rcl_interfaces::srv::SetParameters>(
    "camera/change_parameters",
    std::bind(&DepthAICamera::changeParametersCallback, this, _1, _2),
    rmw_qos_profile_services_default);
}

void DepthAICamera::AutoFocusTimer()
{
  if (_firstFrameReceived) {
    RCLCPP_INFO(
      get_logger(), "[%s]: Change focus mode to %s",
      get_name(), _useAutoFocus ? "auto" : "manual");
    if (!_useAutoFocus) {
      RCLCPP_INFO(
        get_logger(), "[%s]: Manual focus lens position %d",
        get_name(), _videoLensPosition);
    }
    changeFocusMode(_useAutoFocus);
    _auto_focus_timer->cancel();
  }
}

void DepthAICamera::changeParametersCallback(
  const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
  std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response)
{
  response->results.resize(request->parameters.size());

  bool restart_required = true;
  bool new_camera_state = _thread_running;
  for (size_t i = 0; i < request->parameters.size(); i++) {
    auto param = request->parameters[i];
    const auto & type = param.value.type;
    const auto & name = param.name;
    response->results[i].successful = false;
    if (type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL) {
      RCLCPP_INFO(
        get_logger(), "[%s]: Setting %s to %s", get_name(), name.c_str(),
        param.value.bool_value ? "true" : "false");
      if (name == "use_mono_cams") {
        _useMonoCams = param.value.bool_value;
      } else if (name == "use_raw_color_cam") {
        _useRawColorCam = param.value.bool_value;
      } else if (name == "use_nn") {
        _useNeuralNetwork = param.value.bool_value;
      } else if (name == "sync_nn") {
        _syncNN = param.value.bool_value;
      } else if (name == "use_depth") {
        _useDepth = param.value.bool_value;
      } else if (name == "use_video_from_color_cam") {
        _useVideoFromColorCam = param.value.bool_value;
      } else if (name == "use_auto_focus") {
        if (param.value.bool_value != _useAutoFocus) {
          _useAutoFocus = param.value.bool_value;
          changeFocusMode(_useAutoFocus);
          RCLCPP_INFO(
            this->get_logger(), "Change focus mode to %s",
            _useAutoFocus ? "auto" : "manual");
          restart_required = false;
        }
      } else if (name == "camera_running") {
        new_camera_state = param.value.bool_value;
      } else {
        response->results[i].reason = "Unsupported parameter type";
        continue;
      }
    } else if (param.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
      RCLCPP_INFO(
        get_logger(), "[%s]: Setting %s to %f", get_name(),
        name.c_str(), param.value.double_value);
      if (name == "video_fps") {
        if (param.value.double_value > 60.0 || param.value.double_value < 2.0) {
          response->results[i].reason = "FPS must be between 0.2 and 30";
          continue;
        }
        _videoFps = param.value.double_value;
      } else {
        response->results[i].reason = "Unsupported parameter type";
        continue;
      }
    } else if (param.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      RCLCPP_INFO(
        get_logger(), "[%s]: Setting %s to %ld", get_name(),
        name.c_str(), param.value.integer_value);
      if (name == "width") {
        if (param.value.integer_value > 3840 || param.value.integer_value % 8 != 0) {
          response->results[i].reason = "Width must be a multiple of 8 and less than 3840";
          continue;
        }
        _videoWidth = param.value.integer_value;
      } else if (name == "height") {
        if (param.value.integer_value > 2160 || param.value.integer_value % 8 != 0) {
          response->results[i].reason = "Height must be a multiple of 8 and less than 2160";
          continue;
        }
        _videoHeight = param.value.integer_value;
      } else if (name == "bitrate") {
        if (param.value.integer_value < 40000) {
          response->results[i].reason = "Bitrate must be at least 40000";
          continue;
        }
        _videoBitrate = param.value.integer_value;
      } else if (name == "lens_position") {
        if (param.value.integer_value < 0 || param.value.integer_value > 255) {
          response->results[i].reason = "Lens position must be between 0 and 255";
          continue;
        }
        _videoLensPosition = param.value.integer_value;
        changeLensPosition(_videoLensPosition);
        restart_required = false;
      } else {
        response->results[i].reason = "Unsupported parameter type";
        continue;
      }
    } else if (param.value.type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
      RCLCPP_INFO(
        get_logger(), "[%s]: Setting %s to %s", get_name(),
        name.c_str(), param.value.string_value.c_str());
      if (name == "encoding" &&
        (param.value.string_value == "H264" || param.value.string_value == "H265"))
      {
        if (param.value.string_value != "H264" && param.value.string_value != "H265") {
          response->results[i].reason = "Unsupported encoding type";
          continue;
        }
        _videoH265 = param.value.string_value == "H265" ? true : false;
      }
    } else {
      RCLCPP_ERROR(
        get_logger(), "Set parameter failed, unsupported parameter type %d",
        param.value.type);
      continue;
    }
    response->results[i].successful = true;
  }
  if (!new_camera_state) {
    if (_thread_running) {
      RCLCPP_INFO(this->get_logger(), "Stopping video stream");
      _firstFrameReceived = false;
      Stop();
    } else {
      RCLCPP_INFO(this->get_logger(), "The video stream is not running");
    }
    return;
  }

  if (restart_required) {
    if (_thread_running) {
      RCLCPP_INFO(this->get_logger(), "Restarting video stream");
      _firstFrameReceived = false;
      Stop();
    } else {
      RCLCPP_INFO(this->get_logger(), "Starting video stream");
    }
    _auto_focus_timer->reset();
    _auto_focus_timer =
      this->create_wall_timer(
      std::chrono::duration<double>(10.0),
      std::bind(&DepthAICamera::AutoFocusTimer, this));
    TryRestarting();
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
        _auto_focus_timer->reset();
        _auto_focus_timer =
          this->create_wall_timer(
          std::chrono::duration<double>(10.0),
          std::bind(&DepthAICamera::AutoFocusTimer, this));
        TryRestarting();
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
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    // Setup Grayscale Cameras
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    if (_useDepth) {
      auto stereo = _pipeline->create<dai::node::StereoDepth>();
      auto xoutDepth = _pipeline->create<dai::node::XLinkOut>();
      xoutDepth->setStreamName("disparity");
      // Setup StereoDepth
      stereo->initialConfig.setConfidenceThreshold(200);
      stereo->setSubpixel(false);
      stereo->setLeftRightCheck(false);
      stereo->setExtendedDisparity(false);
      stereo->initialConfig.setMedianFilter(dai::StereoDepthProperties::MedianFilter::KERNEL_7x7);
      stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
      stereo->setDepthAlign(dai::CameraBoardSocket::RGB);
      // Create outputs
      monoLeft->out.link(stereo->left);
      monoRight->out.link(stereo->right);
      // stereo->syncedLeft.link(xoutLeft->input);
      // stereo->syncedRight.link(xoutRight->input);
      stereo->disparity.link(xoutDepth->input);
    } else {
      monoLeft->out.link(xoutLeft->input);
      monoRight->out.link(xoutRight->input);
    }
  }
  auto colorCamera = _pipeline->create<dai::node::ColorCamera>();
  auto videoEncoder = _pipeline->create<dai::node::VideoEncoder>();
  auto xoutVideo = _pipeline->create<dai::node::XLinkOut>();
  xoutVideo->setStreamName("enc26xColor");
  // Setup Color Camera
  colorCamera->setBoardSocket(dai::CameraBoardSocket::RGB);
  colorCamera->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);

  // Preview resolution cannot be larger than Video's, thus resolution color camera image is limited
  if (_useNeuralNetwork) {
    colorCamera->setPreviewSize(416, 416);
  } else {
    colorCamera->setPreviewSize(_videoWidth, _videoHeight);
  }
  colorCamera->setVideoSize(_videoWidth, _videoHeight);
  colorCamera->setFps(_videoFps);

  // Like mono cameras, color camera is disabled by default to reduce computational load.
  auto xoutColor = _pipeline->create<dai::node::XLinkOut>();
  xoutColor->setStreamName("color");
  if (_useRawColorCam) {
    if (_useVideoFromColorCam) {
      xoutColor->input.setBlocking(false);
      xoutColor->input.setQueueSize(1);
      colorCamera->video.link(xoutColor->input);
    } else {
      colorCamera->preview.link(xoutColor->input);
      RCLCPP_WARN_EXPRESSION(
        this->get_logger(),
        _useNeuralNetwork,
        "Not using the video from camera! Color camera preview will be sized to NN input size!");
    }
  }

  auto nnOut = _pipeline->create<dai::node::XLinkOut>();
  auto nnPassthroughOut = _pipeline->create<dai::node::XLinkOut>();
  nnOut->setStreamName("detections");
  nnPassthroughOut->setStreamName("pass");
  if (_useNeuralNetwork) {
    auto detectionNetwork = _pipeline->create<dai::node::YoloDetectionNetwork>();

    colorCamera->setPreviewKeepAspectRatio(false);
    colorCamera->setInterleaved(false);
    colorCamera->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    // Network specific settings
    detectionNetwork->setConfidenceThreshold(0.5f);
    detectionNetwork->setNumClasses(80);
    detectionNetwork->setCoordinateSize(4);
    detectionNetwork->setAnchors({10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319});
    detectionNetwork->setAnchorMasks({{"side26", {1, 2, 3}}, {"side13", {3, 4, 5}}});
    detectionNetwork->setIouThreshold(0.5f);
    detectionNetwork->setBlobPath(_nn_directory);
    detectionNetwork->setNumInferenceThreads(2);

    detectionNetwork->input.setBlocking(false);

    // Linking
    colorCamera->preview.link(detectionNetwork->input);
    if (_syncNN) {
      detectionNetwork->passthrough.link(nnPassthroughOut->input);
    }
    detectionNetwork->out.link(nnOut->input);
  }
  Profile encoding = _videoH265 ? Profile::H265_MAIN : Profile::H264_MAIN;
  videoEncoder->setDefaultProfilePreset(_videoFps, encoding);
  videoEncoder->setBitrate(_videoBitrate);
  videoEncoder->setFrameRate(_videoFps);
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
      RCLCPP_INFO(this->get_logger(), "[%s]: Trying camera initialization...", get_name());
      _device = std::make_shared<dai::Device>(*_pipeline, !_useUSB3);
    } catch (const std::runtime_error & err) {
      RCLCPP_ERROR(get_logger(), "Cannot start DepthAI camera: %s", err.what());
      _device.reset();
    }
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: Checking if initialization of '_device' is done...", get_name());
  if (!_device) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "[%s]: Initialized camera. Reading calibration...", get_name());
  try {
    _calibrationHandler = _device->readCalibration();
    RCLCPP_INFO(this->get_logger(), "[%s]: Calibration read from camera. RGB conventer ", get_name());
    dai::rosBridge::ImageConverter rgbConverter(_color_camera_frame, true);
    sensor_msgs::msg::CameraInfo rgbCameraInfo = rgbConverter.calibrationToCameraInfo(
      _calibrationHandler, dai::CameraBoardSocket::RGB, _videoWidth, _videoHeight);
    RCLCPP_INFO(this->get_logger(), "[%s]: Parsing the RGB camera info...", get_name());
    _rgb_camera_info = std::make_unique<sensor_msgs::msg::CameraInfo>(rgbCameraInfo);

    // print camerainfo
    RCLCPP_INFO(this->get_logger(), "[%s]: CameraInfo:", get_name());
    RCLCPP_INFO(this->get_logger(), "[%s]:   width: %d", get_name(), rgbCameraInfo.width);
    RCLCPP_INFO(this->get_logger(), "[%s]:   height: %d", get_name(), rgbCameraInfo.height);
    RCLCPP_INFO(
      this->get_logger(), "[%s]:   distortion_model: %s",
      get_name(), rgbCameraInfo.distortion_model.c_str());
    RCLCPP_INFO(
      this->get_logger(), "[%s]:   D: [%f, %f, %f, %f, %f]",
      get_name(), rgbCameraInfo.d[0], rgbCameraInfo.d[1], rgbCameraInfo.d[2], rgbCameraInfo.d[3],
      rgbCameraInfo.d[4]);
    RCLCPP_INFO(
      this->get_logger(), "[%s]:   K: [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
      get_name(), rgbCameraInfo.k[0], rgbCameraInfo.k[1], rgbCameraInfo.k[2], rgbCameraInfo.k[3],
      rgbCameraInfo.k[4], rgbCameraInfo.k[5], rgbCameraInfo.k[6], rgbCameraInfo.k[7],
      rgbCameraInfo.k[8]);
    RCLCPP_INFO(
      this->get_logger(), "[%s]:   R: [%f, %f, %f, %f, %f, %f, %f, %f, %f]",
      get_name(), rgbCameraInfo.r[0], rgbCameraInfo.r[1], rgbCameraInfo.r[2], rgbCameraInfo.r[3],
      rgbCameraInfo.r[4], rgbCameraInfo.r[5], rgbCameraInfo.r[6], rgbCameraInfo.r[7],
      rgbCameraInfo.r[8]);
    RCLCPP_INFO(
      this->get_logger(), "[%s]:   P: [%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f]",
      get_name(), rgbCameraInfo.p[0], rgbCameraInfo.p[1], rgbCameraInfo.p[2], rgbCameraInfo.p[3],
      rgbCameraInfo.p[4], rgbCameraInfo.p[5], rgbCameraInfo.p[6], rgbCameraInfo.p[7],
      rgbCameraInfo.p[8], rgbCameraInfo.p[9], rgbCameraInfo.p[10], rgbCameraInfo.p[11]);
  } catch (std::bad_alloc const&) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Bad alloc exception while reading calibration", get_name());
  } catch (std::exception const& e) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Exception while reading calibration: %s", get_name(), e.what());
  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "[%s]: Unknown exception while reading calibration", get_name());
  }

  RCLCPP_WARN(this->get_logger(), "[%s]: Printing USB speed...", get_name());
  RCLCPP_INFO(
    this->get_logger(), "[%s]: DepthAI Camera USB Speed: %s", get_name(),
    usbSpeedEnumMap.at(_device->getUsbSpeed()).c_str());
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
  //dai::rosBridge::ImageConverter rgbConverter(_color_camera_frame, false);
  if (_useNeuralNetwork) {
    _neural_network_converter = std::make_shared<dai::rosBridge::ImgDetectionConverter>(
      _color_camera_frame, _videoWidth, _videoHeight, false);
    _neuralNetworkOutputQueue = _device->getOutputQueue("detections", 30, false);
    _neuralNetworkCallback =
      _neuralNetworkOutputQueue->addCallback(
      std::bind(
        &DepthAICamera::onNeuralNetworkCallback, this,
        std::placeholders::_1));
    if (_syncNN) {
      _passthrough_converter = std::make_shared<dai::rosBridge::ImageConverter>(
        _color_camera_frame, false);
      _passthroughQueue =
        _device->getOutputQueue("pass", 30, false);
      _passthroughCallback =
        _passthroughQueue->addCallback(
        std::bind(
          &DepthAICamera::onPassthroughCallback, this,
          std::placeholders::_1));
    }
  }
  if (_useRawColorCam) {
    _color_camera_converter = std::make_shared<dai::rosBridge::ImageConverter>(
      _color_camera_frame, false);
    _colorQueue = _device->getOutputQueue("color", 30, false);
    _colorCamCallback =
      _colorQueue->addCallback(
      std::bind(
        &DepthAICamera::onColorCamCallback, this,
        std::placeholders::_1));
  }
  _videoQueue = _device->getOutputQueue("enc26xColor", 30, true);
  if (_useMonoCams) {
    if (_useDepth) {
      // _depth_disparity_converter = std::make_shared<dai::ros::DisparityConverter>(
      //   _color_camera_frame, 880, 7.5, 20, 2000);  // TODO(sachin): undo hardcoding of baseline)
      _depthQueue = _device->getOutputQueue("disparity", 30, false);
      _depthCallback =
        _depthQueue->addCallback(
        std::bind(
          &DepthAICamera::onDepthCallback, this,
          std::placeholders::_1));
    } else {
      _left_camera_converter = std::make_shared<dai::rosBridge::ImageConverter>(
        _left_camera_frame, false);
      _right_camera_converter = std::make_shared<dai::rosBridge::ImageConverter>(
        _right_camera_frame,
        false);
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
  (void)data; // Using this pointer does not pop from queue, so we don't need to do anything with it.
  std::vector<std::shared_ptr<dai::ImgFrame>> leftPtrVector =
    _leftQueue->tryGetAll<dai::ImgFrame>();
  RCLCPP_DEBUG(
    this->get_logger(), "[%s]: Received %ld left camera frames...",
    get_name(), leftPtrVector.size());
  for (std::shared_ptr<dai::ImgFrame> & leftPtr : leftPtrVector) {
    auto image = _left_camera_converter->toRosMsgPtr(leftPtr);
    _left_publisher->publish(*image);
  }

}

void DepthAICamera::onRightCallback(
  const std::shared_ptr<dai::ADatatype> data)
{
  (void)data;
  std::vector<std::shared_ptr<dai::ImgFrame>> rightPtrVector =
    _rightQueue->tryGetAll<dai::ImgFrame>();
  RCLCPP_DEBUG(
    this->get_logger(), "[%s]: Received %ld right camera frames...",
    get_name(), rightPtrVector.size());
  for (std::shared_ptr<dai::ImgFrame> & rightPtr : rightPtrVector) {
    auto image = _right_camera_converter->toRosMsgPtr(rightPtr);
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
    auto image = _color_camera_converter->toRosMsgPtr(colorPtr);
    _color_publisher->publish(*image);
    if (_camera_info_publisher->get_subscription_count() > 0) {
      _rgb_camera_info->header.stamp = image->header.stamp;
      _rgb_camera_info->header.frame_id = image->header.frame_id;
      _camera_info_publisher->publish(*_rgb_camera_info);
    }
  }
}

void DepthAICamera::onDepthCallback(
  const std::shared_ptr<dai::ADatatype> data)
{
  (void)data;
  std::vector<std::shared_ptr<dai::ImgFrame>> depthPtrVector =
    _depthQueue->tryGetAll<dai::ImgFrame>();
  RCLCPP_DEBUG(
    this->get_logger(), "[%s]: Received %ld depth camera frames...",
    get_name(), depthPtrVector.size());
  for (std::shared_ptr<dai::ImgFrame> & depthPtr : depthPtrVector) {
    auto image = _color_camera_converter->toRosMsgPtr(depthPtr);
    _depth_publisher->publish(*image);
  }
}

void DepthAICamera::onPassthroughCallback(
  const std::shared_ptr<dai::ADatatype> data)
{
  (void)data;
  std::vector<std::shared_ptr<dai::ImgFrame>> colorPtrVector =
    _passthroughQueue->tryGetAll<dai::ImgFrame>();
  RCLCPP_DEBUG(
    this->get_logger(), "[%s]: Received %ld color camera frames...",
    get_name(), colorPtrVector.size());
  for (std::shared_ptr<dai::ImgFrame> & colorPtr : colorPtrVector) {
    auto image = _passthrough_converter->toRosMsgPtr(colorPtr);
    _passthrough_publisher->publish(*image);
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

    if (!_useRawColorCam && _camera_info_publisher->get_subscription_count() > 0) {
      _rgb_camera_info->header.stamp = video_stream_chunk.header.stamp;
      _rgb_camera_info->header.frame_id = video_stream_chunk.header.frame_id;
      _camera_info_publisher->publish(*_rgb_camera_info);
    }
    if (!_firstFrameReceived) {
      _firstFrameReceived = true;
      RCLCPP_INFO(
        this->get_logger(), "[%s]: First frame received!",
        get_name());
    }
  }
}

void DepthAICamera::onNeuralNetworkCallback(
  const std::shared_ptr<dai::ADatatype> data)
{
  (void)data;
  std::vector<std::shared_ptr<dai::ImgDetections>> detectionsPtrVector =
    _neuralNetworkOutputQueue->tryGetAll<dai::ImgDetections>();
  for (std::shared_ptr<dai::ImgDetections> & detectionsPtr : detectionsPtrVector) {
    auto detections = _neural_network_converter->toRosMsgPtr(detectionsPtr);
    _detection_roi_publisher->publish(*detections);
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ctrl::DepthAICamera)
