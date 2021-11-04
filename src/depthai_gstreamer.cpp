#include "depthai_gstreamer.h"
#include "depthai_utils.h"
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/gstbus.h>
#include <gst/gstcaps.h>
#include <gst/gstelement.h>
#include <gst/gstpipeline.h>
#include <nlohmann/json.hpp>
#include <mutex>
#include <queue>

using namespace depthai_ctrl;
using std::placeholders::_1;

/// GStreamer part independent from ROS
struct DepthAIGStreamer::Impl
{
    GstElement *pipeline {}, *appSource {};
    std::queue<CompressedImageMsg::SharedPtr> queue {};
    std::mutex queueMutex {};
    GstClockTime stamp0 {};
    std::atomic<bool> isStreamPlaying {false};
    std::atomic<bool> isStreamDefault {false};
    std::thread gstThread {};
    std::string encoderProfile {};
    std::string streamAddress {};

    Impl() = default;

    ~Impl()
    {
        StopStream();
    }

    void StopStream()
    {
        isStreamPlaying = false;
        if (gstThread.joinable())
        {
            gstThread.join();
        }
    }

    void StartStream()
    {
        isStreamPlaying = true;
        gstThread = std::thread(std::bind(DepthAIGStreamer::Impl::GStreamerThread, this));
    }

    static void GStreamerThread(DepthAIGStreamer::Impl* data)
    {
        std::cout << "GStreamerThread started." << std::endl;
        const bool is_udp_protocol = (data->streamAddress.find("udp://") == 0);
        const std::string h26xparse = (data->encoderProfile == "H264") ? "h264parse" : "h265parse";
        const std::string h26xencoder = (data->encoderProfile == "H264") ? "x264enc" : "x265enc";
        const std::string gstFormat = (data->encoderProfile == "H264") ? "video/x-h264" : "video/x-h265";
        std::string payload = " ";
        std::string sink{};

        if (is_udp_protocol)
        {
            std::string host = DepthAIUtils::ReadIpFromUdpAddress(data->streamAddress);
            int port = DepthAIUtils::ReadPortFromUdpAddress(data->streamAddress);

            sink = "udpsink host=" + host +  " port=" + std::to_string(port) + " ";
            payload = (data->encoderProfile == "H264") ? "! rtph264pay " : "! rtph265pay ";
        }
        else
        {
            sink = "rtspclientsink protocols=tcp tls-validation-flags=0 location=" + data->streamAddress;
        }

        if (data->queue.empty())
        {
            // wait 1 sec in case we have delay in ros
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        if (data->queue.empty()) // video-data is not available - use "default" video output
        {
            data->isStreamDefault = true;

            const std::string pipeline_string = "videotestsrc name=source pattern=chroma-zone-plate "
                                                "! video/x-raw,format=I420,width=1280,height=720 "
                                                "! textoverlay text=\"Camera not detected\" valignment=4 halignment=1 font-desc=Sans "
                                                "! videoconvert ! " + h26xencoder + " speed-preset=2 ! queue "
                                                + payload + "! " + sink;
            std::cout << "Starting no-camera pipeline:" << std::endl;
            std::cout << pipeline_string << std::endl;
            GError* parse_error = nullptr;
            data->pipeline = gst_parse_launch(pipeline_string.c_str(), &parse_error);
            if(parse_error != nullptr)
            {
                std::cerr << "Gst Parse Error " << parse_error->code << ": " <<  parse_error->message << std::endl;
                g_clear_error(&parse_error);
                parse_error = nullptr;
                data->isStreamPlaying = false;
                return;
            }
            g_assert(data->pipeline);
        }
        else
        {
            data->isStreamDefault = false;

            const std::string pipeline_string = "appsrc name=source ! " + h26xparse +" " + payload + "! " + sink;
            std::cout << "Starting pipeline:" << std::endl;
            std::cout << pipeline_string << std::endl;
            GError* parse_error = nullptr;
            data->pipeline = gst_parse_launch(pipeline_string.c_str(), &parse_error);
            if(parse_error != nullptr)
            {
                std::cerr << "Gst Parse Error " << parse_error->code << ": " <<  parse_error->message << std::endl;
                g_clear_error(&parse_error);
                parse_error = nullptr;
                data->isStreamPlaying = false;
                return;
            }
            g_assert(data->pipeline);
            data->appSource = gst_bin_get_by_name(GST_BIN(data->pipeline), "source");
            g_assert(data->appSource);
            g_assert(GST_IS_APP_SRC(data->appSource));

            g_object_set(G_OBJECT(data->appSource),
                         "stream-type",
                         0,
                         "is-live",
                         TRUE,
                         "block",
                         FALSE,
                         "format",
                         GST_FORMAT_TIME,
                         nullptr);
        }

        const auto status = gst_element_set_state(data->pipeline, GST_STATE_PLAYING);
        std::cout << "GStreamerThread: Pipeline Change State Status: " << std::to_string(status) << std::endl;

        while (data->isStreamPlaying)
        {
            if (!data->queue.empty() && !data->isStreamDefault)
            {
                auto videoPtr = data->queue.front();
                data->queueMutex.lock();
                data->queue.pop();
                data->queueMutex.unlock();

                auto& frame = videoPtr->data;
                GstBuffer* buffer = gst_buffer_new_and_alloc(frame.size());
                gst_buffer_fill(buffer, 0, &frame[0], frame.size());
                const auto stamp = videoPtr->header.stamp;
                const GstClockTime gst_stamp = stamp.sec * 1000000000UL + stamp.nanosec;

                if (data->stamp0 == 0)
                {
                    data->stamp0 = gst_stamp;
                }

                const GstClockTime local_stamp = gst_stamp - data->stamp0;
                GST_BUFFER_PTS(buffer) = local_stamp;

                const auto result = ::gst_app_src_push_buffer(GST_APP_SRC(data->appSource), buffer);
                std::cout << "Submitted: " << gst_stamp << " local=" << local_stamp << ": " << std::to_string(result)
                          << std::endl;
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        std::cout << "GStreamerThread released" << std::endl;

        /* Free resources */
        if (data->pipeline != nullptr)
        {
            gst_element_set_state(data->pipeline, GST_STATE_NULL);
            gst_object_unref(data->pipeline);
            data->pipeline = nullptr;
        }
    }
};

DepthAIGStreamer::DepthAIGStreamer(int argc, char* argv[])
    : Node("depthai_gstreamer"), _impl(new Impl())
{
    if(!gst_is_initialized())
    {
        gst_init(&argc, &argv);
    }
    Initialize();
}

DepthAIGStreamer::DepthAIGStreamer(const rclcpp::NodeOptions& options)
    : Node("depthai_gstreamer", options), _impl(new Impl())
{
    if(!gst_is_initialized())
    {
        gst_init(nullptr, nullptr);
    }
    Initialize();
}

DepthAIGStreamer::~DepthAIGStreamer()
{
    _impl->StopStream();
}

bool DepthAIGStreamer::isStreamPlaying() { return _impl->isStreamPlaying;}
bool DepthAIGStreamer::isStreamDefault() { return _impl->isStreamDefault;}

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

    _impl->encoderProfile = get_parameter("encoding").as_string();
    _impl->streamAddress = get_parameter("address").as_string();

    RCLCPP_DEBUG(get_logger(), "Namespace: %s", (default_stream_path + ns).c_str());
    RCLCPP_INFO(get_logger(), "DepthAI GStreamer 1.0.2 started.");
    RCLCPP_INFO(get_logger(), "Streaming %s to address: %s", _impl->encoderProfile.c_str(), _impl->streamAddress.c_str());

    if (get_parameter("start_stream_on_boot").as_bool())
    {
        RCLCPP_INFO(get_logger(), "DepthAI GStreamer: start video stream on boot");
        _impl->StartStream();
    }

}

void DepthAIGStreamer::GrabVideoMsg(const CompressedImageMsg::SharedPtr video_msg)
{
    const auto stamp = video_msg->header.stamp;
    RCLCPP_INFO(get_logger(), "RECEIVED CHUNK #" + std::to_string(stamp.sec) + "." + std::to_string(stamp.nanosec));
    _impl->queueMutex.lock();
    _impl->queue.push(video_msg);
    // When message queue is too big - delete old messages
    if (_impl->queue.size() > 1000)
    {
        _impl->queue.pop();
    }
    _impl->queueMutex.unlock();
}

void DepthAIGStreamer::VideoStreamCommand(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Command to process: '%s'", msg->data.c_str());
    auto cmd = nlohmann::json::parse(msg->data.c_str());
    if (!cmd["Command"].empty())
    {
        std::string command = cmd["Command"];
        std::transform(
            command.begin(), command.end(), command.begin(), [](unsigned char c) { return std::tolower(c); });
        if (command == "start")
        {
            if (!_impl->isStreamPlaying)
            {
                if(!cmd["Address"].empty())
                {
                    std::string res{};
                    const std::string address = cmd["Address"];
                    if(!DepthAIUtils::ValidateAddressParameters(address, res))
                    {
                        RCLCPP_WARN(this->get_logger(), res);
                        return;
                    }
                    _impl->streamAddress = address;
                }

                if(!cmd["Encoding"].empty())
                {
                    const std::string encoding = cmd["Encoding"];
                    if(!DepthAIUtils::ValidateEncodingProfile(encoding))
                    {
                        RCLCPP_WARN(this->get_logger(), "Wrong video encoding profile");
                        return;
                    }
                    _impl->encoderProfile = encoding;
                }

                RCLCPP_INFO(this->get_logger(), "Start video streaming.");
                _impl->StartStream();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Video stream already running.");
        }
        else if (command == "stop")
        {
            RCLCPP_INFO(this->get_logger(), "Stop video streaming.");
            _impl->StopStream();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Unknown command: %s", command.c_str());
        }
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ctrl::DepthAIGStreamer)