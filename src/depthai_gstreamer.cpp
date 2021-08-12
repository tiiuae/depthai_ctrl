#include "depthai_gstreamer.h"

using namespace depthai_ctrl;
using std::placeholders::_1;

DepthAiGStreamer::DepthAiGStreamer(int argc, char* argv[])
    : Node("depthai_gstreamer"),
      mIsStreamPlaying(false),
      mEncoderWidth(1280),
      mEncoderHeight(720),
      mEncoderFps(25),
      mEncoderBitrate(3000000),
      mEncoderProfile("H264"),
      mLoop(nullptr),
      mPipeline(nullptr),
      mAppsrc(nullptr),
      mH26xparse(nullptr),
      mH26xpay(nullptr),
      mUdpSink(nullptr),
      mBusWatchId(0),
      mBus(nullptr),
      mNeedDataSignalId(0),
      mLoopThread(nullptr),
      mQueue1(nullptr),
      mRtspSink(nullptr),
      mGstTimestamp(0),
      mTestSrc(nullptr),
      mTextOverlay(nullptr),
      mH26xEnc(nullptr),
      mTestSrcFilter(nullptr),
      mH26xEncFilter(nullptr),
      mStreamPlayingCheckTimerId(0),
      mLoopContext(nullptr)
{
    gst_init(&argc, &argv);
    Initialize();
}

DepthAiGStreamer::DepthAiGStreamer(const rclcpp::NodeOptions & options)
    : Node("depthai_gstreamer", options),
      mIsStreamPlaying(false),
      mEncoderWidth(1280),
      mEncoderHeight(720),
      mEncoderFps(25),
      mEncoderBitrate(3000000),
      mEncoderProfile("H264"),
      mLoop(nullptr),
      mPipeline(nullptr),
      mAppsrc(nullptr),
      mH26xparse(nullptr),
      mH26xpay(nullptr),
      mUdpSink(nullptr),
      mBusWatchId(0),
      mBus(nullptr),
      mNeedDataSignalId(0),
      mLoopThread(nullptr),
      mQueue1(nullptr),
      mRtspSink(nullptr),
      mGstTimestamp(0),
      mTestSrc(nullptr),
      mTextOverlay(nullptr),
      mH26xEnc(nullptr),
      mTestSrcFilter(nullptr),
      mH26xEncFilter(nullptr),
      mStreamPlayingCheckTimerId(0),
      mLoopContext(nullptr)
{
    char* argv[] = {""};
    int argc = 1;
    gst_init(&argc, reinterpret_cast<char***>(&argv));
}

void DepthAiGStreamer::Initialize()
{
    declare_parameter<std::string>("address", "rtsp://127.0.0.1:8554/mystream");
    mStreamAddress = this->get_parameter("address").as_string();

    mLoopContext = g_main_context_default();
    mLoop = g_main_loop_new(mLoopContext, false);

    declare_parameter<std::string>("video_stream_topic", "camera/color/video");

    const std::string video_stream_topic = get_parameter("video_stream_topic").as_string();
    _video_subscriber = create_subscription<CompressedImageMsg>(video_stream_topic, rclcpp::SystemDefaultsQoS(), std::bind(&DepthAiGStreamer::GrabVideoMsg, this, std::placeholders::_1));

    _video_stream_command_subscriber = this->create_subscription<std_msgs::msg::String>(
        "videostreamcmd",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&DepthAiGStreamer::VideoStreamCommand, this, std::placeholders::_1));

    rcl_interfaces::msg::ParameterDescriptor encoding_desc;
    encoding_desc.name = "encoding";
    encoding_desc.type = rclcpp::PARAMETER_STRING;
    encoding_desc.description = "Encoding format of the video stream.";
    encoding_desc.additional_constraints = "Accepted values are H264 and H265.";
    declare_parameter<std::string>("encoding", "H264", encoding_desc);
    declare_parameter<int>("width", 1280);
    declare_parameter<int>("height", 720);
    declare_parameter<int>("fps", 25);
    declare_parameter<int>("bitrate", 3000000);

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
    std::string stream_path =
        "rtsps://DroneUser:22f6c4de-6144-4f6c-82ea-8afcdf19f316@video-stream.sacplatform.com:8555";
    std::string ns = std::string(this->get_namespace());
    declare_parameter<std::string>("address", stream_path + ns, address_desc);
    RCLCPP_DEBUG(this->get_logger(), "Namespace: %s", (stream_path + ns).c_str());

    _parameter_setter =
        rclcpp::Node::add_on_set_parameters_callback(std::bind(&DepthAiGStreamer::SetParameters, this, _1));

    mEncoderWidth = get_parameter("width").as_int();
    mEncoderHeight = get_parameter("height").as_int();
    mEncoderFps = get_parameter("fps").as_int();
    mEncoderBitrate = get_parameter("bitrate").as_int();
    mEncoderProfile = get_parameter("encoding").as_string();
    mStreamAddress = get_parameter("address").as_string();

    if (get_parameter("start_stream_on_boot").as_bool())
    {
        RCLCPP_INFO(this->get_logger(), "Start DepthAI GStreamer video stream.");
        CreatePipeline();
    }
}

void DepthAiGStreamer::GrabVideoMsg(const CompressedImageMsg::SharedPtr video_msg)
{
    std::lock_guard<std::mutex> lock(_message_queue_mutex);
    _message_queue.push(video_msg);
}

DepthAiGStreamer::~DepthAiGStreamer()
{
    DestroyPipeline();
    if (mLoop != nullptr)
    {
        g_main_loop_unref(mLoop);
        mLoop = nullptr;
    }
}

void DepthAiGStreamer::DestroyPipeline(void)
{
    if (mPipeline != nullptr)
    {
        gst_element_set_state(mPipeline, GST_STATE_NULL);
        gst_object_unref(GST_OBJECT(mPipeline));
        mPipeline = nullptr;
    }
    if (mBus != nullptr)
    {
        gst_object_unref(mBus);
        mBus = nullptr;
    }
    if (mBusWatchId != 0)
    {
        g_source_remove(mBusWatchId);
        mBusWatchId = 0;
    }
}

void DepthAiGStreamer::BuildDefaultPipeline(void)
{
    bool is_udp_protocol = (mStreamAddress.find("udp://") == 0);

    mPipeline = gst_pipeline_new("default_pipeline");

    // Video test source.
    mTestSrc = gst_element_factory_make("videotestsrc", "source");
    g_object_set(G_OBJECT(mTestSrc), "pattern", 2, NULL);
    mTestSrcFilter = gst_element_factory_make("capsfilter", "source_filter");
    g_object_set(G_OBJECT(mTestSrcFilter),
                 "caps",
                 gst_caps_new_simple("video/x-raw",
                                     "format",
                                     G_TYPE_STRING,
                                     "I420",
                                     "width",
                                     G_TYPE_INT,
                                     mEncoderWidth,
                                     "height",
                                     G_TYPE_INT,
                                     mEncoderHeight,
                                     "framerate",
                                     GST_TYPE_FRACTION,
                                     mEncoderFps,
                                     1,
                                     NULL),
                 NULL);

    // Text overlay.
    mTextOverlay = gst_element_factory_make("textoverlay", "text");
    g_object_set(G_OBJECT(mTextOverlay),
                 "text",
                 "Camera not detected!",
                 "valignment",
                 4,  // 4 = center
                 "halignment",
                 1,  // 1 = center
                 "font-desc",
                 "Sans, 42",
                 NULL);

    // Software encoder and parser.
    if (mEncoderProfile == "H265")
    {
        mH26xEnc = gst_element_factory_make("x265enc", "encoder");
        g_object_set(G_OBJECT(mH26xEnc),
                     "bitrate",
                     500,  // 500 kbit/sec
                     "speed-preset",
                     2,  // 2 = superfast
                     "tune",
                     4,  // 4 = zerolatency
                     NULL);
        mH26xparse = gst_element_factory_make("h265parse", "parser");
    }
    else
    {
        mH26xEnc = gst_element_factory_make("x264enc", "encoder");
        mH26xparse = gst_element_factory_make("h264parse", "parser");
    }

    // Sink element. UDP or RTSP client.
    if (is_udp_protocol)
    {
        // UDP Sink
        if (mEncoderProfile == "H265")
        {
            mH26xpay = gst_element_factory_make("rtph265pay", "payload");
        }
        else
        {
            mH26xpay = gst_element_factory_make("rtph264pay", "payload");
        }
        g_object_set(G_OBJECT(mH26xpay), "pt", 96, NULL);
        mUdpSink = gst_element_factory_make("udpsink", "udp_sink");
        g_object_set(G_OBJECT(mUdpSink), "host", ReadIpAddresFromUdpAddress().c_str(), NULL);
        g_object_set(G_OBJECT(mUdpSink), "port", ReadPortFromUdpAddress(), NULL);
    }
    else
    {
        // RTSP client sink
        mRtspSink = gst_element_factory_make("rtspclientsink", "rtsp_sink");
        g_object_set(G_OBJECT(mRtspSink),
                     "protocols",
                     4,  // 4 = tcp
                     "tls-validation-flags",
                     0,
                     "location",
                     mStreamAddress.c_str(),
                     NULL);
    }

    // Caps definition for source element.
    std::string profile = mEncoderProfile;
    std::stringstream ss;
    std::string gstFormat;
    std::transform(profile.begin(), profile.end(), profile.begin(), ::tolower);
    ss << "video/x-" << profile;
    ss >> gstFormat;
    mH26xEncFilter = gst_element_factory_make("capsfilter", "encoder_filter");
    g_object_set(G_OBJECT(mH26xEncFilter),
                 "caps",
                 gst_caps_new_simple(gstFormat.c_str(),
                                     "profile",
                                     G_TYPE_STRING,
                                     "baseline",
                                     "pass",
                                     G_TYPE_INT,
                                     5,
                                     "trellis",
                                     G_TYPE_BOOLEAN,
                                     false,
                                     "tune",
                                     G_TYPE_STRING,
                                     "zero-latency",
                                     "threads",
                                     G_TYPE_INT,
                                     0,
                                     "speed-preset",
                                     G_TYPE_STRING,
                                     "superfast",
                                     "subme",
                                     G_TYPE_INT,
                                     1,
                                     "bitrate",
                                     G_TYPE_INT,
                                     4000,
                                     NULL),
                 NULL);

    mBus = gst_pipeline_get_bus(GST_PIPELINE(mPipeline));
    mBusWatchId = gst_bus_add_watch(mBus, gst_StreamEventCallBack, this);
    gst_object_unref(mBus);
    mBus = nullptr;

    if (is_udp_protocol)
    {
        gst_bin_add_many(GST_BIN(mPipeline),
                         mTestSrc,
                         mTestSrcFilter,
                         mTextOverlay,
                         mH26xEnc,
                         mH26xEncFilter,
                         mH26xparse,
                         mH26xpay,
                         mUdpSink,
                         NULL);
        gst_element_link_many(
            mTestSrc, mTestSrcFilter, mTextOverlay, mH26xEnc, mH26xEncFilter, mH26xparse, mH26xpay, mUdpSink, NULL);
    }
    else
    {
        gst_bin_add_many(GST_BIN(mPipeline),
                         mTestSrc,
                         mTestSrcFilter,
                         mTextOverlay,
                         mH26xEnc,
                         mH26xEncFilter,
                         mH26xparse,
                         mRtspSink,
                         NULL);
        gst_element_link_many(
            mTestSrc, mTestSrcFilter, mTextOverlay, mH26xEnc, mH26xEncFilter, mH26xparse, mRtspSink, NULL);
    }

    mLoopThread = g_thread_new("GstThread", (GThreadFunc)DepthAiGStreamer::gst_PlayStream, this);
}

void DepthAiGStreamer::CreatePipeline(void)
{
    if (!IsVideoStreamAvailable())
    {
        g_printerr("Warning: video stream is not available. Start default stream.\n");
        BuildDefaultPipeline();
        return;
    }

    bool is_udp_protocol = (mStreamAddress.find("udp://") == 0);

    mPipeline = gst_pipeline_new("rgbCamSink_pipeline");
    // Source element.
    mAppsrc = gst_element_factory_make("appsrc", "source");
    g_object_set(G_OBJECT(mAppsrc), "do-timestamp", true, "is-live", true, "block", true, "stream-type", 0, NULL);
    gst_util_set_object_arg(G_OBJECT(mAppsrc), "format", "GST_FORMAT_TIME");
    // H26x parser. Is this really needed?
    if (mEncoderProfile == "H265")
    {
        mH26xparse = gst_element_factory_make("h265parse", "parser");
    }
    else
    {
        mH26xparse = gst_element_factory_make("h264parse", "parser");
    }
    mQueue1 = gst_element_factory_make("queue", "queue1");
    // Sink element. UDP or RTSP client.
    if (is_udp_protocol)
    {
        // UDP Sink
        if (mEncoderProfile == "H265")
        {
            mH26xpay = gst_element_factory_make("rtph265pay", "payload");
        }
        else
        {
            mH26xpay = gst_element_factory_make("rtph264pay", "payload");
        }
        g_object_set(G_OBJECT(mH26xpay), "pt", 96, NULL);
        mUdpSink = gst_element_factory_make("udpsink", "udp_sink");
        g_object_set(G_OBJECT(mUdpSink), "host", ReadIpAddresFromUdpAddress().c_str(), NULL);
        g_object_set(G_OBJECT(mUdpSink), "port", ReadPortFromUdpAddress(), NULL);
    }
    else
    {
        // RTSP client sink
        mRtspSink = gst_element_factory_make("rtspclientsink", "rtsp_sink");
        g_object_set(G_OBJECT(mRtspSink),
                     "protocols",
                     4,  // 4 = tcp
                     "tls-validation-flags",
                     0,
                     "location",
                     mStreamAddress.c_str(),
                     NULL);
    }

    // Caps definition for source element.
    std::string profile = mEncoderProfile;
    std::stringstream ss;
    std::string gstFormat;
    std::transform(profile.begin(), profile.end(), profile.begin(), ::tolower);
    ss << "video/x-" << profile;
    ss >> gstFormat;
    g_object_set(G_OBJECT(mAppsrc),
                 "caps",
                 gst_caps_new_simple(gstFormat.c_str(),
                                     "width",
                                     G_TYPE_INT,
                                     mEncoderWidth,
                                     "height",
                                     G_TYPE_INT,
                                     mEncoderHeight,
                                     "framerate",
                                     GST_TYPE_FRACTION,
                                     mEncoderFps,
                                     1,
                                     NULL),
                 NULL);

    mH26xEncFilter = gst_element_factory_make("capsfilter", "encoder_filter");
    g_object_set(G_OBJECT(mH26xEncFilter),
                 "caps",
                 gst_caps_new_simple(gstFormat.c_str(),
                                     "profile",
                                     G_TYPE_STRING,
                                     "main",
                                     "stream-format",
                                     G_TYPE_STRING,
                                     "byte-stream",
                                     NULL),
                 NULL);

    mBus = gst_pipeline_get_bus(GST_PIPELINE(mPipeline));
    mBusWatchId = gst_bus_add_watch(mBus, gst_StreamEventCallBack, this);
    gst_object_unref(mBus);
    mBus = nullptr;

    if (is_udp_protocol)
    {
        gst_bin_add_many(
            GST_BIN(mPipeline), mAppsrc, mH26xEncFilter, mH26xparse, mQueue1, mH26xpay, mUdpSink, NULL);
        gst_element_link_many(mAppsrc, mH26xEncFilter, mH26xparse, mQueue1, mH26xpay, mUdpSink, NULL);
    }
    else
    {
        gst_bin_add_many(GST_BIN(mPipeline), mAppsrc, mH26xEncFilter, mH26xparse, mQueue1, mRtspSink, NULL);
        gst_element_link_many(mAppsrc, mH26xEncFilter, mH26xparse, mQueue1, mRtspSink, NULL);
    }
    mLoopThread = g_thread_new("GstThread", (GThreadFunc)DepthAiGStreamer::gst_PlayStream, this);

    mNeedDataSignalId = g_signal_connect(mAppsrc, "need-data", G_CALLBACK(gst_NeedDataCallBack), this);
}
//
//void DepthAiGStreamer::StopStream(void)
//{
//    GstFlowReturn ret;
//
//    if (mNeedDataSignalId != 0)
//    {
//        g_signal_handler_disconnect(mAppsrc, mNeedDataSignalId);
//    }
//    if (mAppsrc != nullptr)
//    {
//        g_signal_emit_by_name(mAppsrc, "end-of-stream", &ret);
//        if (ret != GST_FLOW_OK)
//        {
//            g_printerr("Error: Emit end-of-stream failed\n");
//        }
//    }
//    if (mPipeline != nullptr)
//    {
//        gst_element_set_state(mPipeline, GST_STATE_NULL);
//    }
//    if (mLoop != nullptr)
//    {
//        g_main_loop_quit(mLoop);
//    }
//    if (mLoopThread != nullptr)
//    {
//        g_thread_join(mLoopThread);
//    }
//    mIsStreamPlaying = false;
//}
//
//
//void DepthAiGStreamer::SetEncoderWidth(int width)
//{
//    if (width > 4096)
//    {
//        g_printerr("Width must be smaller than 4096 for H26x encoder profile.\n");
//        return;
//    }
//    if (width % 8 != 0)
//    {
//        g_printerr("Width must be multiple of 8 for H26x encoder profile.\n");
//        return;
//    }
//    mEncoderWidth = width;
//    //depthAICam->SetEncoderWidth(mEncoderWidth);
//}
//
//void DepthAiGStreamer::SetEncoderHeight(int height)
//{
//    if (height > 4096)
//    {
//        g_printerr("Height must be smaller than 4096 for H26x encoder profile.\n");
//        return;
//    }
//    if (height % 8 != 0)
//    {
//        g_printerr("Height must be multiple of 8 for H26x encoder profile.\n");
//        return;
//    }
//    mEncoderHeight = height;
//    //depthAICam->SetEncoderHeight(mEncoderHeight);
//}
//
//void DepthAiGStreamer::SetEncoderFps(int fps)
//{
//    if (fps > 60)
//    {
//        g_printerr("Too high frames per second.\n");
//        return;
//    }
//    mEncoderFps = fps;
//    //depthAICam->SetEncoderFps(mEncoderFps);
//}
//
//void DepthAiGStreamer::SetEncoderProfile(std::string profile)
//{
//    std::transform(profile.begin(), profile.end(), profile.begin(), ::toupper);
//    if (profile != "H264" && profile != "H265")
//    {
//        g_printerr("Not valid H26x profile.\n");
//        return;
//    }
//    mEncoderProfile = profile;
//    //depthAICam->SetEncoderProfile(mEncoderProfile);
//}
//
//void DepthAiGStreamer::SetStreamAddress(const std::string address)
//{
//    mStreamAddress = address;
//    if (mRtspSink)
//    {
//        g_object_set(G_OBJECT(mRtspSink),
//                     "protocols",
//                     4,  // 4 = tcp
//                     "location",
//                     mStreamAddress.c_str(),
//                     NULL);
//    }
//}


void* DepthAiGStreamer::gst_PlayStream(gpointer data)
{
    auto depthAIGst = static_cast<DepthAiGStreamer*>(data);
//    CameraNode* depthAICam = depthAIGst->depthAICam;
//    if (depthAICam != nullptr)
//    {
//        depthAICam->StartStreaming();
//    }
    gst_element_set_state(depthAIGst->mPipeline, GST_STATE_PLAYING);
    g_main_loop_run(depthAIGst->mLoop);
    g_thread_exit(depthAIGst->mLoopThread);

    return nullptr;
}

gboolean DepthAiGStreamer::gst_MissingPluginMessage(GstMessage* msg)
{
    if (GST_MESSAGE_TYPE(msg) != GST_MESSAGE_ELEMENT || gst_message_get_structure(msg) == NULL)
        return FALSE;

    return gst_structure_has_name(gst_message_get_structure(msg), "missing-plugin");
}

gboolean DepthAiGStreamer::gst_StreamEventCallBack(GstBus* bus, GstMessage* message, gpointer data)
{
    (void)bus;
    g_debug("%s: Got %s message\n", __FUNCTION__, GST_MESSAGE_TYPE_NAME(message));

    DepthAiGStreamer* depthAIGst = (DepthAiGStreamer*)data;
    GstTagList* list = nullptr;

    switch (GST_MESSAGE_TYPE(message))
    {
        case GST_MESSAGE_STREAM_STATUS:
            GstStreamStatusType statusType;
            GstElement* element;

            gst_message_parse_stream_status(message, &statusType, &element);
            g_print("Element %s stream status type %d.\n", GST_OBJECT_NAME(message->src), statusType);
            break;

        case GST_MESSAGE_PROGRESS:
            GstProgressType progressType;
            gchar *code, *text;

            gst_message_parse_progress(message, &progressType, &code, &text);
            switch (progressType)
            {
                case GST_PROGRESS_TYPE_START:
                case GST_PROGRESS_TYPE_CONTINUE:
                case GST_PROGRESS_TYPE_COMPLETE:
                case GST_PROGRESS_TYPE_CANCELED:
                case GST_PROGRESS_TYPE_ERROR:
                default:
                    break;
            }
            g_print("Progress: (%s) %s\n", code, text);
            g_free(code);
            g_free(text);
            break;

        case GST_MESSAGE_NEW_CLOCK:
            GstClock* clock;

            gst_message_parse_new_clock(message, &clock);
            g_print("New clock: %s\n", (clock ? GST_OBJECT_NAME(clock) : "NULL"));
            break;

        case GST_MESSAGE_LATENCY:
            g_print("Redistribute latency...\n");
            gst_bin_recalculate_latency(GST_BIN(depthAIGst->mPipeline));
            break;

        case GST_MESSAGE_ELEMENT:
            if (gst_MissingPluginMessage(message))
            {
                const gchar* desc;

                desc = gst_missing_plugin_message_get_description(message);
                g_print("Missing element: %s\n", desc ? desc : "(no description)");
            }
            break;

        case GST_MESSAGE_STATE_CHANGED:
            GstState old_state, new_state;

            gst_message_parse_state_changed(message, &old_state, &new_state, NULL);
            g_print("Element %s changed state from %s to %s.\n",
                    GST_OBJECT_NAME(message->src),
                    gst_element_state_get_name(old_state),
                    gst_element_state_get_name(new_state));
            if (g_strrstr(GST_OBJECT_NAME(message->src), "rtspbin") && new_state == GST_STATE_PLAYING)
            {
                depthAIGst->mIsStreamPlaying = true;
            }
            break;

        case GST_MESSAGE_EOS:
            g_print("End of stream.\n");
            g_main_loop_quit(depthAIGst->mLoop);
            break;

        case GST_MESSAGE_TAG:
            list = gst_tag_list_new_empty();

            gst_message_parse_tag(message, &list);

            g_print("Tag: %s.\n", gst_tag_list_to_string(list));
            gst_tag_list_unref(list);
            break;

        case GST_MESSAGE_WARNING:
            gchar* warnDebug;
            GError* warning;

            gst_message_parse_warning(message, &warning, &warnDebug);
            g_free(warnDebug);

            g_warning("Warning: %s.\n", warning->message);
            g_error_free(warning);
            break;

        case GST_MESSAGE_ERROR:
            gchar* errDebug;
            GError* error;
            GSource* source;

            gst_message_parse_error(message, &error, &errDebug);
            g_printerr("ERROR from element %s: %s\n", GST_OBJECT_NAME(message->src), error->message);
            g_printerr("Debugging info: %s\n", (errDebug) ? errDebug : "none");
            if (error->code == G_FILE_ERROR_NODEV &&
                g_strrstr(error->message, "Could not open resource for reading and writing"))
            {
                GstFlowReturn ret;
                if (depthAIGst->mNeedDataSignalId != 0)
                {
                    g_signal_handler_disconnect(depthAIGst->mAppsrc, depthAIGst->mNeedDataSignalId);
                }
                if (depthAIGst->mAppsrc != nullptr)
                {
                    g_signal_emit_by_name(depthAIGst->mAppsrc, "end-of-stream", &ret);
                    if (ret != GST_FLOW_OK)
                    {
                        g_printerr("Error: Emit end-of-stream failed\n");
                    }
                }
                if (depthAIGst->mPipeline != nullptr)
                {
                    gst_element_set_state(depthAIGst->mPipeline, GST_STATE_NULL);
                }
                depthAIGst->mIsStreamPlaying = false;
                // Restart stream after two seconds.
                source = g_timeout_source_new(2000);
                g_source_set_callback(source,
                                      DepthAiGStreamer::gst_StreamRestartCallback,
                                      depthAIGst,
                                      DepthAiGStreamer::StreamPlayingRestartDone);
                g_source_attach(source, depthAIGst->mLoopContext);
                g_source_unref(source);
            }
            g_error_free(error);
            g_free(errDebug);
            break;

        default:
            break;
    }

    return true;
}

void DepthAiGStreamer::gst_NeedDataCallBack(GstElement* appsrc, guint unused_size, gpointer user_data)
{
    (void)unused_size;
    GstFlowReturn ret;
    GstBuffer* buffer;
    auto depthAIGst = static_cast<DepthAiGStreamer*>(user_data);


    CompressedImageMsg::SharedPtr image;
    while(!bool(image))
    {
        std::cerr << "TRY TO GET SOME DATA!" << std::endl;
        depthAIGst->_message_queue_mutex.lock();
        if(!depthAIGst->_message_queue.empty())
        {
            image = depthAIGst->_message_queue.back();
            depthAIGst->_message_queue.pop();
        }
        depthAIGst->_message_queue_mutex.unlock();
    }
    std::cerr << "WE GOT SOME DATA!" << std::endl;
    auto& frame = image->data;

    guint size = frame.size();
    buffer = gst_buffer_new_allocate(NULL, size, NULL);
    gst_buffer_fill(buffer, 0, (gconstpointer)(&frame[0]), size);

    GST_BUFFER_PTS(buffer) = depthAIGst->mGstTimestamp;
    GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, (int)depthAIGst->mEncoderFps);
    depthAIGst->mGstTimestamp += GST_BUFFER_DURATION(buffer);

    g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);
    if (ret != GST_FLOW_OK && ret != GST_FLOW_FLUSHING)
    {
        g_main_loop_quit(depthAIGst->mLoop);
    }
}

gboolean DepthAiGStreamer::gst_StreamRestartCallback(gpointer user_data)
{
    auto depthAIGst = static_cast<DepthAiGStreamer*>(user_data);

    g_debug("Restart stream because of connection failed.\n");
    if (depthAIGst->mAppsrc)
    {
        depthAIGst->mNeedDataSignalId =
            g_signal_connect(depthAIGst->mAppsrc, "need-data", G_CALLBACK(gst_NeedDataCallBack), depthAIGst);
    }
    gst_element_set_state(depthAIGst->mPipeline, GST_STATE_PLAYING);

    return G_SOURCE_REMOVE;
}

std::string DepthAiGStreamer::ReadIpAddresFromUdpAddress(void)
{
    // String format is: udp://<ip_addr>:<port>
    std::string addr = mStreamAddress;
    std::string udp_protocol = "udp://";
    addr.erase(0, udp_protocol.size());

    return addr.substr(0, addr.find(":"));
}

int DepthAiGStreamer::ReadPortFromUdpAddress(void)
{
    // String format is: udp://<ip_addr>:<port>
    std::string addr = mStreamAddress;
    std::string udp_protocol = "udp://";
    addr.erase(0, udp_protocol.size());

    return atoi(addr.substr(addr.find(":") + 1).c_str());
}

void DepthAiGStreamer::ValidateAddressParameters(const std::string address, rcl_interfaces::msg::SetParametersResult& res)
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

rcl_interfaces::msg::SetParametersResult DepthAiGStreamer::SetParameters(const std::vector<rclcpp::Parameter>& parameters)
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
            }
            if (width_val > 4096)
            {
                result.successful = false;
                result.reason = "Width must be smaller than 4096 for H26x encoder profile.";
                RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
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
            }
            if (height_val > 4096)
            {
                result.successful = false;
                result.reason = "Height must be smaller than 4096 for H26x encoder profile.";
                RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
            }
        }

        if (parameter.get_name() == "address")
        {
            if (IsStreamPlaying())
            {
                result.successful = false;
                result.reason = "Cannot change stream address while stream is playing.";
                return result;
            }
            std::string addr = parameter.as_string();
            ValidateAddressParameters(addr, result);
            if (!result.successful)
            {
                return result;
            }
            mStreamAddress = addr;

        }
    }
    return result;
}

void DepthAiGStreamer::VideoStreamCommand(const std_msgs::msg::String::SharedPtr msg)
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
        mStreamAddress = address;
    }
    if (!cmd["Command"].empty())
    {

        std::string command = cmd["Command"];
        std::transform(
            command.begin(), command.end(), command.begin(), [](unsigned char c) { return std::tolower(c); });
        if (command == "start")
        {
            if (!IsStreamPlaying())
            {
                mEncoderWidth = get_parameter("width").as_int();
                mEncoderHeight = get_parameter("height").as_int();
                mEncoderFps = get_parameter("fps").as_int();
                mEncoderBitrate = get_parameter("bitrate").as_int();
                mEncoderProfile = get_parameter("encoding").as_string();
                mStreamAddress = get_parameter("address").as_string();

                RCLCPP_INFO(this->get_logger(), "Start DepthAI camera streaming.");
                CreatePipeline();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "DepthAI camera already streaming.");
        }
        else if (command == "stop")
        {
            RCLCPP_INFO(this->get_logger(), "Stop DepthAI camera streaming.");
        }
    }
}



#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(depthai_ctrl::DepthAiGStreamer)