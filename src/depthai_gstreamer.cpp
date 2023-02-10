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

#include "depthai_ctrl/depthai_gstreamer.h"
#include "depthai_ctrl/depthai_utils.h"
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/gstbus.h>
#include <gst/gstcaps.h>
#include <gst/gstelement.h>
#include <gst/gstpipeline.h>
#include <depthai_ctrl/gstreamer_interface.hpp>
#include <nlohmann/json.hpp>
#include <mutex>
#include <queue>

using namespace depthai_ctrl;
using std::placeholders::_1;


DepthAIGStreamer::DepthAIGStreamer(int argc, char * argv[])
: Node("depthai_gstreamer"), _impl(nullptr)
{
  _impl = new GstInterface(argc, argv);

  Initialize();
}

DepthAIGStreamer::DepthAIGStreamer(const rclcpp::NodeOptions & options)
: Node("depthai_gstreamer", options), _impl(nullptr)
{

  _impl = new GstInterface(0, 0);

  Initialize();
}

DepthAIGStreamer::~DepthAIGStreamer()
{
  RCLCPP_INFO(get_logger(), "DepthAI GStreamer: stop stream called");
  //_impl->StopStream();
  RCLCPP_INFO(get_logger(), "DepthAI GStreamer: Destroying called");
  delete _impl;
}


void DepthAIGStreamer::Initialize()
{

  _callback_group_timer = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  _callback_group_video_subscriber = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  _callback_group_cmd_subscriber = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  auto video_sub_opt = rclcpp::SubscriptionOptions();
  video_sub_opt.callback_group = _callback_group_video_subscriber;

  auto cmd_sub_opt = rclcpp::SubscriptionOptions();
  cmd_sub_opt.callback_group = _callback_group_cmd_subscriber;

  rclcpp::QoS qos_profile(10);
  _video_subscriber = create_subscription<CompressedImageMsg>(
    "camera/color/video",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&DepthAIGStreamer::GrabVideoMsg, this, std::placeholders::_1), video_sub_opt);

  _stream_command_subscriber = this->create_subscription<std_msgs::msg::String>(
    "gstreamer/videostreamcmd",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&DepthAIGStreamer::VideoStreamCommand, this, std::placeholders::_1), cmd_sub_opt);

  _handle_stream_status_timer = this->create_wall_timer(
    std::chrono::milliseconds(10000),
    std::bind(&DepthAIGStreamer::HandleStreamStatus, this), _callback_group_timer); // 10 sec

  declare_parameter<int>("width", 1280);
  declare_parameter<int>("height", 720);
  declare_parameter<int>("fps", 25);
  declare_parameter<int>("bitrate", 3000000);

  rcl_interfaces::msg::ParameterDescriptor encoding_desc;
  encoding_desc.name = "encoding";
  encoding_desc.type = rclcpp::PARAMETER_STRING;
  encoding_desc.description = "Encoding format of the video stream.";
  encoding_desc.additional_constraints = "Accepted values are H264 and H265.";
  declare_parameter<std::string>("encoding", "H264", encoding_desc);

  rcl_interfaces::msg::ParameterDescriptor start_stream_on_boot_desc;
  start_stream_on_boot_desc.name = "start_stream_on_boot";
  start_stream_on_boot_desc.type = rclcpp::PARAMETER_BOOL;
  start_stream_on_boot_desc.description =
    "The node will start the video stream during "
    "boot if set to true.";
  start_stream_on_boot_desc.additional_constraints =
    "This parameter has no "
    "effect after node has started.";
  declare_parameter<bool>("start_stream_on_boot", true, start_stream_on_boot_desc);

  rcl_interfaces::msg::ParameterDescriptor address_desc;
  address_desc.name = "address";
  address_desc.type = rclcpp::PARAMETER_STRING;
  address_desc.description =
    "Video stream destination address. The Gstreamer pipeline is generated "
    "based on this address.";
  address_desc.additional_constraints =
    "UDP or RTSP addresses are accepted. For example:\n"
    "\trtsp://<user>:<key>@<ip_address>:<port>/<path> (default)\n"
    "\tudp://<ip_address>:<port>";
  const std::string default_stream_path =
    "rtsps://DroneUser:22f6c4de-6144-4f6c-82ea-8afcdf19f316@video-stream.sacplatform.com:8555";
  const std::string ns = std::string(get_namespace());
  declare_parameter<std::string>("address", default_stream_path + ns, address_desc);

  _impl->SetEncoderProfile(get_parameter("encoding").as_string());
  _impl->SetStreamAddress(get_parameter("address").as_string());

  RCLCPP_DEBUG(get_logger(), "Namespace: %s", (default_stream_path + ns).c_str());
  RCLCPP_INFO(get_logger(), "DepthAI GStreamer 1.0.2 started.");
  RCLCPP_INFO(
    get_logger(), "Streaming %s to address: %s",
    _impl->GetEncoderProfile().c_str(), _impl->GetStreamAddress().c_str());
  _is_stop_requested = !get_parameter("start_stream_on_boot").as_bool();
  if (get_parameter("start_stream_on_boot").as_bool()) {
    RCLCPP_INFO(get_logger(), "DepthAI GStreamer: start video stream on boot");

    RCLCPP_INFO(this->get_logger(), "Resetting timer.");
    _handle_stream_status_timer->reset();
    _impl->StartStream();
  }

}

void DepthAIGStreamer::GrabVideoMsg(const CompressedImageMsg::SharedPtr video_msg)
{
  const auto stamp = video_msg->header.stamp;

  RCLCPP_DEBUG(
    get_logger(),
    "[GST %s]RECEIVED CHUNK # %d.%d",
    _impl->IsStreamPlaying() ? "STREAMING" : "STOPPED", stamp.sec, stamp.nanosec);

  g_mutex_lock(&_impl->haveDataCondMutex);
  _impl->queue.push(video_msg);
  // When message queue is too big - delete old messages
  if (_impl->queue.size() > (size_t)_impl->GetEncoderFps() * 2) {
    _impl->queue.pop();
  }
  g_cond_signal(&_impl->haveDataCond);
  g_mutex_unlock(&_impl->haveDataCondMutex);
}


void DepthAIGStreamer::HandleStreamStatus()
{
  if (!_is_stop_requested) {
    if (!_impl->IsStreamPlaying()) {
      if (!_impl->IsStreamStarting()) {
        RCLCPP_INFO(get_logger(), "DepthAI GStreamer: try to start video stream");

        _impl->StartStream();
      } else {
        RCLCPP_INFO(get_logger(), "DepthAI GStreamer: Start failed, stop stream");
        _impl->StopStream();
      }
    } else if (_impl->IsStreamPlaying()) {
      RCLCPP_INFO(
        get_logger(), "DepthAI GStreamer: Stream running okay in %s mode.",
        _impl->IsStreamDefault() ? "default" : "camera streaming");

      if (_impl->IsStreamDefault()) {
        if (_impl->queue.size() > (size_t)_impl->GetEncoderFps()) {
          RCLCPP_INFO(
            get_logger(), "DepthAI GStreamer: Stream is default, but have data with size of %ld.",
            _impl->queue.size());
          RCLCPP_INFO(get_logger(), "DepthAI GStreamer: Switching to camera stream.");
          _impl->StopStream();
        }
      } else {
        if (_video_subscriber->get_publisher_count() == 0) {
          RCLCPP_INFO(get_logger(), "DepthAI GStreamer: Stream is running, but no publisher.");
          RCLCPP_INFO(get_logger(), "DepthAI GStreamer: Switching to default stream.");

          g_mutex_lock(&_impl->haveDataCondMutex);
          RCLCPP_INFO(get_logger(), "DepthAI GStreamer: Clearing queue for start");
          std::queue<CompressedImageMsg::SharedPtr>().swap(_impl->queue);
          g_mutex_unlock(&_impl->haveDataCondMutex);
          _impl->StopStream();
        }
      }
    }
  } else {
    RCLCPP_INFO(get_logger(), "DepthAI GStreamer: Waiting for start command.");
  }
}

void DepthAIGStreamer::VideoStreamCommand(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Command to process: '%s'", msg->data.c_str());
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
    if (command == "start") {
      if (!_impl->IsStreamPlaying()) {
        if (!cmd["Address"].empty()) {
          std::string res{};
          const std::string address = cmd["Address"];
          if (!DepthAIUtils::ValidateAddressParameters(address, res)) {
            RCLCPP_WARN(this->get_logger(), res.c_str());
            return;
          }
          _impl->SetStreamAddress(address);
        }

        if (!cmd["Encoding"].empty()) {
          const std::string encoding = cmd["Encoding"];
          if (!DepthAIUtils::ValidateEncodingProfile(encoding)) {
            RCLCPP_WARN(this->get_logger(), "Wrong video encoding profile");
            return;
          }
          _impl->SetEncoderProfile(encoding);
        }

        RCLCPP_INFO(this->get_logger(), "Start video streaming.");
        _is_stop_requested = false;
        RCLCPP_INFO(this->get_logger(), "Resetting timer.");
        _handle_stream_status_timer->reset();
        RCLCPP_INFO(this->get_logger(), "Resetted timer.");
        g_mutex_lock(&_impl->haveDataCondMutex);
        RCLCPP_INFO(get_logger(), "DepthAI GStreamer: Clearing queue for start");
        std::queue<CompressedImageMsg::SharedPtr>().swap(_impl->queue);
        g_mutex_unlock(&_impl->haveDataCondMutex);
        _impl->StartStream();
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Video stream already running.");
    } else if (command == "stop") {
      if (_impl->IsStreamPlaying()) {
        RCLCPP_INFO(this->get_logger(), "Stop video streaming.");
        _is_stop_requested = true;
        RCLCPP_INFO(this->get_logger(), "Resetting timer.");
        _handle_stream_status_timer->reset();
        RCLCPP_INFO(this->get_logger(), "Resetted timer.");
        _impl->StopStream();


      } else {
        RCLCPP_INFO(this->get_logger(), "Video stream already stopped.");
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Unknown command: %s", command.c_str());
    }
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ctrl::DepthAIGStreamer)
