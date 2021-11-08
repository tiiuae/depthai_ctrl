#include "depthai_gstreamer.h"
#include "depthai_utils.h"
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/gstbus.h>
#include <gst/gstcaps.h>
#include <gst/gstelement.h>
#include <gst/gstpipeline.h>
#include <gstreamer_interface.hpp>
#include <nlohmann/json.hpp>
#include <mutex>
#include <queue>

using namespace depthai_ctrl;
using std::placeholders::_1;


DepthAIGStreamer::DepthAIGStreamer(int argc, char * argv[])
: Node("depthai_gstreamer"), _impl(nullptr)
{
  _impl = new GstInterface(argc, argv);
  if (_impl != nullptr) {
    _impl = _impl;
  }
  Initialize();
}

DepthAIGStreamer::DepthAIGStreamer(const rclcpp::NodeOptions & options)
: Node("depthai_gstreamer", options), _impl(nullptr)
{
  
  _impl = new GstInterface(0, 0);
  if (_impl != nullptr) {
    _impl = _impl;
  }
  Initialize();
}

DepthAIGStreamer::~DepthAIGStreamer()
{
  RCLCPP_INFO(get_logger(), "DepthAI GStreamer: stop stream called");
  _impl->StopStream();
  //RCLCPP_INFO(get_logger(), "DepthAI GStreamer: Destroying called");
  //delete _impl;
}

//bool DepthAIGStreamer::isStreamPlaying() {return _impl->isStreamPlaying;}
//bool DepthAIGStreamer::isStreamDefault() {return _impl->isStreamDefault;}

void DepthAIGStreamer::Initialize()
{
  declare_parameter<std::string>("video_stream_topic", "camera/color/video");

  const std::string video_stream_topic = get_parameter("video_stream_topic").as_string();
  _video_subscriber = create_subscription<CompressedImageMsg>(
    video_stream_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&DepthAIGStreamer::GrabVideoMsg, this, std::placeholders::_1));

  _stream_command_subscriber = this->create_subscription<std_msgs::msg::String>(
    "videostreamcmd",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&DepthAIGStreamer::VideoStreamCommand, this, std::placeholders::_1));

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
  declare_parameter<bool>("start_stream_on_boot", false, start_stream_on_boot_desc);

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

  if (get_parameter("start_stream_on_boot").as_bool()) {
    RCLCPP_INFO(get_logger(), "DepthAI GStreamer: start video stream on boot");
    _impl->BuildPipeline();
  }

}

void DepthAIGStreamer::GrabVideoMsg(const CompressedImageMsg::SharedPtr video_msg)
{
  const auto stamp = video_msg->header.stamp;
  RCLCPP_DEBUG(
    get_logger(),
    "RECEIVED CHUNK #" + std::to_string(stamp.sec) + "." + std::to_string(stamp.nanosec));
  
    g_mutex_lock(&_impl->haveDataCondMutex);
  _impl->queueMutex.lock();
  _impl->queue.push(video_msg);
  // When message queue is too big - delete old messages
  if (_impl->queue.size() > 1000) {
    _impl->queue.pop();
  }
  _impl->queueMutex.unlock();
    g_cond_signal(&_impl->haveDataCond);
    g_mutex_unlock(&_impl->haveDataCondMutex);
  /*if (!_impl->IsStreamPlaying()) {
    _impl->StartStream();
  }*/
}

void DepthAIGStreamer::VideoStreamCommand(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Command to process: '%s'", msg->data.c_str());
  auto cmd = nlohmann::json::parse(msg->data.c_str());
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
            RCLCPP_WARN(this->get_logger(), res);
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
        _impl->StartStream();
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Video stream already running.");
    } else if (command == "stop") {
      RCLCPP_INFO(this->get_logger(), "Stop video streaming.");
      _impl->StopStream();
    } else {
      RCLCPP_INFO(this->get_logger(), "Unknown command: %s", command.c_str());
    }
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ctrl::DepthAIGStreamer)
