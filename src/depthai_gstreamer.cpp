#include "depthai_gstreamer.h"
#include "depthai_utils.h"
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/gstbus.h>
#include <gst/gstcaps.h>
#include <gst/gstelement.h>
#include <gst/gstpipeline.h>
#include <nlohmann/json.hpp>
#include <iomanip>
#include <mutex>
#include <queue>

using namespace depthai_ctrl;
using std::placeholders::_1;

/// GStreamer part independent from ROS
struct DepthAIGStreamer::Impl
{
    GstElement *pipeline{}, *appSource{};
    std::queue<CompressedImageMsg::SharedPtr> queue{};
    std::mutex queueMutex{};
    GstClockTime stamp0{};
    std::atomic<bool> isStreamPlaying{false};
    std::atomic<bool> isStreamDefault{false};
    std::atomic<bool> isErrorDetected{false};
    std::thread gstThread{};
    std::string encoderProfile{};
    std::string streamAddress{};
    uint64_t frameCount = 0;

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
        ClearGstPipeline();
    }

    void StartStream()
    {
        stamp0 = 0;
        frameCount = 0;
        isStreamPlaying = true;
        isErrorDetected = false;
        gstThread = std::thread(std::bind(DepthAIGStreamer::Impl::GStreamerThread, this));
    }

    /// Creates GST Pipeline, but not starts it yet
    bool CreateGstPipeline()
    {
        const bool is_udp_protocol = (streamAddress.find("udp://") == 0);
        const std::string h26Xparse = (encoderProfile == "H264") ? " ! h264parse" : " ! h265parse";
        const std::string h26Xencoder = (encoderProfile == "H264") ? " ! x264enc " : " ! x265enc ";
        const std::string gstFormat = (encoderProfile == "H264") ? "video/x-h264" : "video/x-h265";
        const std::string h26Xcaps = (encoderProfile == "H264") ?
                                     "! video/x-h264,pass=5,profile=baseline,trellis=false,tune=zerolatency,threads=0,speed-preset=superfast,subme=1,bitrate=4000" : "";
        std::string payload = " ";
        std::string sink{};

        if (is_udp_protocol)
        {
            std::string host = DepthAIUtils::ReadIpFromUdpAddress(streamAddress);
            int port = DepthAIUtils::ReadPortFromUdpAddress(streamAddress);

            sink = "udpsink name=sink host=" + host + " port=" + std::to_string(port) + " ";
            payload = (encoderProfile == "H264") ? "! rtph264pay " : "! rtph265pay ";
        }
        else
        {
            sink = "! rtspclientsink name=sink latency=500 rtx-time=0 protocols=tcp tls-validation-flags=0 location=" +
                   streamAddress;
        }

        isStreamDefault = queue.empty();

        std::string pipeline_string;
        if(isStreamDefault)
        {
            pipeline_string = "videotestsrc pattern=ball ! video/x-raw,format=I420,width=1280,height=720,framerate=30/1 "
                   "! textoverlay text=\"Camera not detected\" valignment=4 halignment=1 font-desc=Sans "
                   "! videoconvert " + h26Xencoder + h26Xcaps + "! queue " + payload + sink;
        }
        else
        {
            pipeline_string = "appsrc name=source ! " + gstFormat + ",profile=baseline,stream-format=byte-stream ! h264parse ! queue " + payload + sink;
        }

        std::cout << "Create GST Pipeline:" << std::endl;
        std::cout << pipeline_string << std::endl;

        GError* parse_error = nullptr;
        pipeline = gst_parse_launch(pipeline_string.c_str(), &parse_error);
        if (parse_error != nullptr)
        {
            std::cerr << "Gst Pipeline Error " << parse_error->code << ": " << parse_error->message << std::endl;
            std::cerr << "Streaming is aborted!" << std::endl;
            g_clear_error(&parse_error);
            parse_error = nullptr;
            return false;
        }

        if(pipeline == nullptr)
        {
            return false;
        }

        if(!isStreamDefault)
        {
            appSource = gst_bin_get_by_name(GST_BIN(pipeline), "source");
            if(appSource == nullptr || !GST_IS_APP_SRC(appSource))
            {
                return false;
            }

            g_object_set(G_OBJECT(appSource),
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
        return true;
    }

    void ClearGstPipeline()
    {
        if (pipeline != nullptr)
        {
            gst_element_set_state(pipeline, GST_STATE_NULL);
            gst_object_unref(pipeline);
            pipeline = nullptr;
        }

    }

    /// Synchronous way to feed data into AppSrc
    void PushAppSrcData()
    {
        if (!queue.empty() && !isStreamDefault)
        {
            auto videoPtr = queue.front();
            queueMutex.lock();
            queue.pop();
            queueMutex.unlock();

            auto& frame = videoPtr->data;
            GstBuffer* buffer = gst_buffer_new_and_alloc(frame.size());
            gst_buffer_fill(buffer, 0, &frame[0], frame.size());
            const auto stamp = videoPtr->header.stamp;
            const GstClockTime gst_stamp = stamp.sec * 1000000000UL + stamp.nanosec;

            stamp0 = (stamp0 == 0) ? gst_stamp : stamp0;

            const GstClockTime local_stamp = gst_stamp - stamp0;
            GST_BUFFER_PTS(buffer) = local_stamp;
            const auto local_sec = local_stamp / 1000000000UL;
            const auto local_nano = local_stamp % 1000000000UL;
            const auto result = ::gst_app_src_push_buffer(GST_APP_SRC(appSource), buffer);

            if (frameCount < 100 || frameCount % 100 == 0)
            {
                std::cout << "Submitted to Appsrc # " << frameCount;
                std::cout << ": chunk time=" << stamp.sec << "." << std::setw(9) << std::setfill('0') << stamp.nanosec;
                std::cout << ", GSTream time=" << local_sec << "." << std::setw(9) << std::setfill('0') << local_nano;
                std::cout << " ; result: " << std::to_string(result);
                if (result != GST_FLOW_OK)
                {
                    std::cout << " (NOT OK)";
                }
                std::cout << std::endl;
            }

            frameCount++;
        }
    }

    static void GStreamerThread(DepthAIGStreamer::Impl* data)
    {
        std::cout << "GStreamerThread started." << std::endl;

        // main loop, which tries to recover from GST Errors
        while(data->isStreamPlaying)
        {
            // delay before next attempt
            std::this_thread::sleep_for(std::chrono::seconds(1));

            if(!data->CreateGstPipeline())
            {
                data->isStreamPlaying = false;
                break;
            }

            auto bus = gst_element_get_bus (data->pipeline);
            gst_element_set_state(data->pipeline, GST_STATE_PLAYING);
            data->isErrorDetected = false;

            while (data->isStreamPlaying && !data->isErrorDetected)
            {
                if(data->isStreamDefault || data->queue.empty())
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }
                else
                {
                    data->PushAppSrcData();
                }

                GstMessage *message = gst_bus_poll (bus, GST_MESSAGE_ERROR, 1);
                if(message != nullptr && message->type == GST_MESSAGE_ERROR)
                {
                    // When the application receives an error message it should stop playback of the pipeline
                    // and not assume that more data will be played
                    data->isErrorDetected = true;

                    GError *err{};
                    gchar *dbg_info{};

                    gst_message_parse_error (message, &err, &dbg_info);
                    g_printerr ("ERROR from element %s: %s\n",
                                GST_OBJECT_NAME (message->src), err->message);
                    g_printerr ("Debugging info: %s\n", (dbg_info) ? dbg_info : "none");
                    g_error_free (err);
                    g_free (dbg_info);
                }
            }

            gst_object_unref (bus);
            data->ClearGstPipeline();
        }

        data->ClearGstPipeline();
        std::cout << "GStreamerThread released" << std::endl;
    }
};

DepthAIGStreamer::DepthAIGStreamer(int argc, char* argv[]) : Node("depthai_gstreamer"), _impl(new Impl())
{
    if (!gst_is_initialized())
    {
        gst_init(&argc, &argv);
    }
    Initialize();
}

DepthAIGStreamer::DepthAIGStreamer(const rclcpp::NodeOptions& options)
    : Node("depthai_gstreamer", options), _impl(new Impl())
{
    if (!gst_is_initialized())
    {
        gst_init(nullptr, nullptr);
    }
    Initialize();
}

DepthAIGStreamer::~DepthAIGStreamer()
{
    _impl->StopStream();
}

bool DepthAIGStreamer::IsStreamPlaying()
{
    return _impl->isStreamPlaying;
}
bool DepthAIGStreamer::IsStreamDefault()
{
    return _impl->isStreamDefault;
}
bool DepthAIGStreamer::IsErrorDetected()
{
    return _impl->isErrorDetected;
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
    const std::string default_stream_path = "rtsp://127.0.0.1:8554";

    const std::string ns = std::string(get_namespace());
    declare_parameter<std::string>("address", default_stream_path + ns, address_desc);

    _impl->encoderProfile = get_parameter("encoding").as_string();
    _impl->streamAddress = get_parameter("address").as_string();

    RCLCPP_DEBUG(get_logger(), "Namespace: %s", (default_stream_path + ns).c_str());
    RCLCPP_INFO(get_logger(), "DepthAI GStreamer 1.0.2 started.");
    RCLCPP_INFO(get_logger(), "Streaming %s to address: %s", _impl->encoderProfile.c_str(),
        _impl->streamAddress.c_str());

    if (get_parameter("start_stream_on_boot").as_bool())
    {
        RCLCPP_INFO(get_logger(), "DepthAI GStreamer: start video stream on boot");
        _impl->StartStream();
    }
}

void DepthAIGStreamer::GrabVideoMsg(const CompressedImageMsg::SharedPtr video_msg)
{
//    const auto stamp = video_msg->header.stamp;
//    static uint64_t frame_count = 0UL;
//    if (frame_count < 100 || frame_count % 100 == 0)
//    {
//        RCLCPP_INFO(
//            get_logger(), "Received video-chunk #%d at time: %d.%09d sec ", frame_count, stamp.sec, stamp.nanosec);
//    }
//
//    frame_count++;

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
    nlohmann::json cmd{};
    try
    {
        cmd = nlohmann::json::parse(msg->data.c_str());
    }
    catch(...)
    {
        RCLCPP_ERROR(this->get_logger(), "Error while parsing JSON string from VideoCommand");
        return;
    }

    if (!cmd["Command"].empty())
    {
        std::string command = cmd["Command"];
        std::transform(
            command.begin(), command.end(), command.begin(), [](unsigned char c) { return std::tolower(c); });
        if (command == "start")
        {
            if (!_impl->isStreamPlaying)
            {
                if (!cmd["Address"].empty())
                {
                    std::string res{};
                    const std::string address = cmd["Address"];
                    if (!DepthAIUtils::ValidateAddressParameters(address, res))
                    {
                        RCLCPP_WARN(this->get_logger(), res);
                        return;
                    }
                    _impl->streamAddress = address;
                }

                if (!cmd["Encoding"].empty())
                {
                    const std::string encoding = cmd["Encoding"];
                    if (!DepthAIUtils::ValidateEncodingProfile(encoding))
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