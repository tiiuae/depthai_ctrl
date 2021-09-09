#include "depthai_gstreamer.h"
#include "depthai_gstreamer_impl.h"

using namespace depthai_ctrl;
using std::placeholders::_1;


DepthAIGStreamer::DepthAIGStreamer(int argc, char* argv[])
    : Node("depthai_gstreamer"),
    _impl(new Impl(&argc, &argv))
{
    Initialize();
}

DepthAIGStreamer::DepthAIGStreamer(const rclcpp::NodeOptions& options)
    : Node("depthai_gstreamer", options),
      _impl(new Impl(nullptr, nullptr))
{
    Initialize();
}

bool DepthAIGStreamer::IsIpAddressValid(const std::string& ip_address)
{
    struct sockaddr_in sa;
    int result = inet_pton(AF_INET, ip_address.c_str(), &(sa.sin_addr));

    return result != 0;
}


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
    std::string stream_path =
        "rtsps://DroneUser:22f6c4de-6144-4f6c-82ea-8afcdf19f316@video-stream.sacplatform.com:8555";
    std::string ns = std::string(get_namespace());
    declare_parameter<std::string>("address", stream_path + ns, address_desc);

    RCLCPP_DEBUG(get_logger(), "Namespace: %s", (stream_path + ns).c_str());

    _impl->SetEncoderWidth(get_parameter("width").as_int());
    _impl->SetEncoderHeight(get_parameter("height").as_int());
    _impl->SetEncoderFps(get_parameter("fps").as_int());
    _impl->SetEncoderBitrate(get_parameter("bitrate").as_int());
    _impl->SetEncoderProfile(get_parameter("encoding").as_string());
    _impl->SetStreamAddress(get_parameter("address").as_string());


    RCLCPP_INFO(get_logger(), "DepthAI GStreamer 1.0.0 started.");


    if (get_parameter("start_stream_on_boot").as_bool())
    {
        RCLCPP_INFO(get_logger(), "DepthAI GStreamer: start stream on boot");
        _impl->CreatePipeLine();
    }

    _parameter_setter =
        rclcpp::Node::add_on_set_parameters_callback(std::bind(&DepthAIGStreamer::SetParameters, this, _1));
}

DepthAIGStreamer::~DepthAIGStreamer()
{
    _impl->DestroyPipeline();
    _impl.reset();
}

void DepthAIGStreamer::GrabVideoMsg(const CompressedImageMsg::SharedPtr video_msg)
{
    _impl->_message_queue_mutex.lock();
    _impl->_message_queue.push(video_msg);
    _impl->_message_queue_mutex.unlock();
    RCLCPP_INFO(get_logger(), "RECEIVED VIDEO CHUNK");
    // When message queue is too big - delete old messages
    if (_impl->_message_queue.size() > 1000)
    {
        _impl->_message_queue.pop();
    }
}

void DepthAIGStreamer::ValidateAddressParameters(const std::string address,
                                                 rcl_interfaces::msg::SetParametersResult& res)
{
    std::string udp_protocol = "udp://";
    std::string rtsp_protocol = "rtsp://";
    std::string protocol = "";
    std::string addr = address;

    if (addr.size() == 0)
    {
        SetRclCppError(res, "Empty address.");
        return;
    }

    if (addr.find(udp_protocol) == 0)
    {
        protocol = "udp";
        addr.erase(0, udp_protocol.size());
    }
    else if (addr.find(rtsp_protocol) == 0)
    {
        protocol = "rtsp";
        addr.erase(0, rtsp_protocol.size());
    }
    else
    {
        SetRclCppError(res, "Not valid protocol in stream address.");
        return;
    }

    std::string ip_addr, port;
    if (protocol == "udp")
    {
        ip_addr = addr.substr(0, addr.find(":"));
        if (ip_addr.size() == 0 || !IsIpAddressValid(ip_addr))
        {
            SetRclCppError(res, "Not valid IP address.");
            return;
        }
        port = addr.substr(addr.find(":") + 1);
        if (port.size() == 0)
        {
            SetRclCppError(res, "Empty port in address.");
            return;
        }
    }
    else if (protocol == "rtsp")
    {
        std::string user, key, path;
        user = addr.substr(0, addr.find(":"));
        if (user.size() == 0)
        {
            SetRclCppError(res, "Empty user in address.");
            return;
        }
        addr = addr.substr(addr.find(":") + 1);
        key = addr.substr(0, addr.find("@"));
        if (key.size() == 0)
        {
            SetRclCppError(res, "Empty key in address.");
            return;
        }
        addr = addr.substr(addr.find("@") + 1);
        ip_addr = addr.substr(0, addr.find(":"));
        if (ip_addr.size() == 0 || !IsIpAddressValid(ip_addr))
        {
            SetRclCppError(res, "Not valid IP address.");
            return;
        }
        addr = addr.substr(addr.find(":") + 1);
        port = addr.substr(0, addr.find("/"));
        if (port.size() == 0)
        {
            SetRclCppError(res, "Empty port in address.");
            return;
        }
        path = addr.substr(addr.find("/") + 1);
        if (path.size() == 0)
        {
            SetRclCppError(res, "Empty path in address.");
            return;
        }
    }
}

rcl_interfaces::msg::SetParametersResult DepthAIGStreamer::SetParameters(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto& parameter : parameters)
    {
        if (parameter.get_name() == "encoding")
        {
            std::string encoding_val = parameter.as_string();
            std::transform(encoding_val.begin(), encoding_val.end(), encoding_val.begin(), ::toupper);
            if ((encoding_val != "H264") && (encoding_val != "H265"))
            {
                result.successful = false;
                result.reason = "Not valid encoding. Allowed H264 and H265.";
                RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                continue;
            }
        }

        if (parameter.get_name() == "width")
        {
            int width_val = parameter.as_int();
            if (width_val % 8 != 0)
            {
                result.successful = false;
                result.reason = "Width must be multiple of 8 for H26x encoder profile.";
                RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                continue;
            }
            if (width_val > 4096)
            {
                result.successful = false;
                result.reason = "Width must be smaller than 4096 for H26x encoder profile.";
                RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                continue;
            }
        }

        if (parameter.get_name() == "height")
        {
            int height_val = parameter.as_int();
            if (height_val % 8 != 0)
            {
                result.successful = false;
                result.reason = "Heigth must be multiple of 8 for H26x encoder profile.";
                RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                continue;
            }
            if (height_val > 4096)
            {
                result.successful = false;
                result.reason = "Height must be smaller than 4096 for H26x encoder profile.";
                RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                continue;
            }
        }
        if (parameter.get_name() == "address")
        {
            if (_impl->IsStreamPlaying())
            {
                result.successful = false;
                result.reason = "Cannot change stream address while stream is playing.";
                return result;
            }
            std::string addr = parameter.as_string();
            ValidateAddressParameters(addr, result);
            if (!result.successful)
            {
                result.successful = false;
                result.reason = "Address is wrong.";
            }
        }
    }
    return result;
}

void DepthAIGStreamer::VideoStreamCommand(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Command to process: '%s'", msg->data.c_str());
    auto cmd = nlohmann::json::parse(msg->data.c_str());
    if (!cmd["Address"].empty())
    {
        std::string address = cmd["Address"];
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        ValidateAddressParameters(address, result);
        if (!result.successful)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in stream address. Stream cannot be started.");
            return;
        }
        _impl->SetStreamAddress(address);
    }
    if (!cmd["Command"].empty())
    {
        std::string command = cmd["Command"];
        std::transform(
            command.begin(), command.end(), command.begin(), [](unsigned char c) { return std::tolower(c); });
        if (command == "start")
        {
            if (!_impl->IsStreamPlaying())
            {
                _impl->SetEncoderWidth(get_parameter("width").as_int());
                _impl->SetEncoderHeight(get_parameter("height").as_int());
                _impl->SetEncoderFps(get_parameter("fps").as_int());
                _impl->SetEncoderBitrate(get_parameter("bitrate").as_int());
                _impl->SetEncoderProfile(get_parameter("encoding").as_string());
                _impl->SetStreamAddress(get_parameter("address").as_string());

                RCLCPP_INFO(this->get_logger(), "Start DepthAI camera streaming.");
                _impl->CreatePipeLine();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "DepthAI camera already streaming.");
        }
        else if (command == "stop")
        {
            RCLCPP_INFO(this->get_logger(), "Stop DepthAI camera streaming.");
            _impl->DestroyPipeline();
        }
    }
}



#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ctrl::DepthAIGStreamer)