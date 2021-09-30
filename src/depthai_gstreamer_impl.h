#ifndef FOG_SW_DEPTHAI_GSTREAMER_IMPL_H
#define FOG_SW_DEPTHAI_GSTREAMER_IMPL_H

#include <arpa/inet.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/gstbus.h>
#include <gst/gstcaps.h>
#include <gst/gstelement.h>
#include <gst/gstpipeline.h>
#include <mutex>
#include <queue>

namespace depthai_ctrl
{

/// GStreamer part independent from ROS
struct DepthAIGStreamer::Impl
{
    std::queue<CompressedImageMsg::SharedPtr> _message_queue;
    std::mutex _message_queue_mutex;

    bool IsDeviceAvailable() {return !_message_queue.empty();}

    Impl(int* argc, char **argv[]) : /*depthAICam(nullptr),*/ mLoop(nullptr), mPipeline(nullptr), mAppsrc(nullptr),
                                         mH26xparse(nullptr), mH26xpay(nullptr), mUdpSink(nullptr), mBusWatchId(0),
                                         mBus(nullptr), mNeedDataSignalId(0), mLoopThread(nullptr), mQueue1(nullptr),
                                         mRtspSink(nullptr), mIsStreamPlaying(false), mEncoderWidth(1280),
                                         mEncoderHeight(720), mEncoderFps(25), mEncoderBitrate(3000000),
                                         mEncoderProfile("H264"), mGstTimestamp(0), mTestSrc(nullptr),
                                         mTextOverlay(nullptr), mH26xEnc(nullptr), mTestSrcFilter(nullptr),
                                         mH26xEncFilter(nullptr), mStreamPlayingCheckTimerId(0)
    {
        mStreamAddress = "";
        gst_init(argc, argv);
        mLoopContext = g_main_context_default();
        mLoop = g_main_loop_new(mLoopContext, false);
        //depthAICam = new DepthAICam();
    }

    ~Impl()
    {
//        if (depthAICam != nullptr) {
//            delete(depthAICam);
//        }
        DestroyPipeline();
        if (mLoop != nullptr) {
            g_main_loop_unref(mLoop);
            mLoop = nullptr;
        }
    }

    void DestroyPipeline(void)
    {
        if (mPipeline != nullptr) {
            gst_element_set_state(mPipeline, GST_STATE_NULL);
            gst_object_unref(GST_OBJECT(mPipeline));
            mPipeline = nullptr;
        }
        if (mBus != nullptr) {
            gst_object_unref(mBus);
            mBus = nullptr;
        }
        if (mBusWatchId != 0) {
            g_source_remove(mBusWatchId);
            mBusWatchId = 0;
        }
        mIsStreamPlaying = false;
    }

    void BuildDefaultPipeline(void)
    {
        bool is_udp_protocol = (mStreamAddress.find("udp://") == 0);

        mPipeline = gst_pipeline_new("default_pipeline");

        // Video test source.
        mTestSrc = gst_element_factory_make("videotestsrc", "source");
        g_object_set(G_OBJECT(mTestSrc), "pattern", 2, NULL);
        mTestSrcFilter = gst_element_factory_make("capsfilter", "source_filter");
        g_object_set(G_OBJECT(mTestSrcFilter), "caps",
                     gst_caps_new_simple("video/x-raw",
                                         "format", G_TYPE_STRING, "I420",
                                         "width", G_TYPE_INT, mEncoderWidth,
                                         "height", G_TYPE_INT, mEncoderHeight,
                                         "framerate", GST_TYPE_FRACTION, mEncoderFps, 1,
                                         NULL), NULL);

        // Text overlay.
        mTextOverlay = gst_element_factory_make("textoverlay", "text");
        g_object_set(G_OBJECT(mTextOverlay),
                     "text", "Camera not detected!",
                     "valignment", 4, // 4 = center
                     "halignment", 1, // 1 = center
                     "font-desc", "Sans, 42",
                     NULL);

        // Software encoder and parser.
        if (mEncoderProfile == "H265") {
            mH26xEnc = gst_element_factory_make("x265enc", "encoder");
            g_object_set(G_OBJECT(mH26xEnc),
                         "bitrate", 500, // 500 kbit/sec
                         "speed-preset", 2, // 2 = superfast
                         "tune", 4, // 4 = zerolatency
                         NULL);
            mH26xparse = gst_element_factory_make("h265parse", "parser");
        } else {
            mH26xEnc = gst_element_factory_make("x264enc", "encoder");
            mH26xparse = gst_element_factory_make("h264parse", "parser");
        }

        // Sink element. UDP or RTSP client.
        if (is_udp_protocol) {
            // UDP Sink
            if (mEncoderProfile == "H265") {
                mH26xpay = gst_element_factory_make("rtph265pay", "payload");
            } else {
                mH26xpay = gst_element_factory_make("rtph264pay", "payload");
            }
            g_object_set(G_OBJECT(mH26xpay), "pt", 96, NULL);
            mUdpSink = gst_element_factory_make("udpsink", "udp_sink");
            g_object_set(G_OBJECT(mUdpSink), "host", ReadIpAddresFromUdpAddress().c_str(), NULL);
            g_object_set(G_OBJECT(mUdpSink), "port", ReadPortFromUdpAddress(), NULL);
        } else {
            // RTSP client sink
            mRtspSink = gst_element_factory_make("rtspclientsink", "rtsp_sink");
            g_object_set(G_OBJECT(mRtspSink),
                         "protocols", 4, // 4 = tcp
                         "tls-validation-flags", 0,
                         "location", mStreamAddress.c_str(),
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
        g_object_set(G_OBJECT(mH26xEncFilter), "caps",
                     gst_caps_new_simple(gstFormat.c_str(),
                                         "profile", G_TYPE_STRING, "baseline",
                                         "pass", G_TYPE_INT, 5,
                                         "trellis", G_TYPE_BOOLEAN, false,
                                         "tune", G_TYPE_STRING, "zero-latency",
                                         "threads", G_TYPE_INT, 0,
                                         "speed-preset", G_TYPE_STRING, "superfast",
                                         "subme", G_TYPE_INT, 1,
                                         "bitrate", G_TYPE_INT, 4000,
                                         NULL), NULL);

        mBus = gst_pipeline_get_bus(GST_PIPELINE(mPipeline));
        mBusWatchId = gst_bus_add_watch(mBus, StreamEventCallBack, this);
        gst_object_unref(mBus);
        mBus = nullptr;

        if (is_udp_protocol) {
            gst_bin_add_many(GST_BIN(mPipeline), mTestSrc, mTestSrcFilter, mTextOverlay, mH26xEnc, mH26xEncFilter, mH26xparse, mH26xpay, mUdpSink, NULL);
            gst_element_link_many(mTestSrc, mTestSrcFilter, mTextOverlay, mH26xEnc, mH26xEncFilter, mH26xparse, mH26xpay, mUdpSink, NULL);
        } else {
            gst_bin_add_many(GST_BIN(mPipeline), mTestSrc, mTestSrcFilter, mTextOverlay, mH26xEnc, mH26xEncFilter, mH26xparse, mRtspSink, NULL);
            gst_element_link_many(mTestSrc, mTestSrcFilter, mTextOverlay, mH26xEnc, mH26xEncFilter, mH26xparse, mRtspSink, NULL);
        }

        mLoopThread = g_thread_new("GstThread", (GThreadFunc)Impl::PlayStream, this);
    }

    void CreatePipeLine(void)
    {
//        if (!/*depthAICam->*/IsDeviceAvailable()) {
//            g_printerr("Warning: device not available. Start default stream.\n");
//            BuildDefaultPipeline();
//            return;
//        }

        bool is_udp_protocol = (mStreamAddress.find("udp://") == 0);

        mPipeline = gst_pipeline_new("rgbCamSink_pipeline");
        // Source element.
        mAppsrc = gst_element_factory_make("appsrc", "source");
        g_object_set(G_OBJECT(mAppsrc),
                     "do-timestamp", true,
                     "is-live", true,
                     "block", false,
                     "format", "GST_FORMAT_TIME",
                     "stream-type", 0,
                     NULL);
        gst_util_set_object_arg(G_OBJECT(mAppsrc), "format", "GST_FORMAT_TIME");
        // H26x parser. Is this really needed?
        if (mEncoderProfile == "H265") {
            mH26xparse = gst_element_factory_make("h265parse", "parser");
        } else {
            mH26xparse = gst_element_factory_make("h264parse", "parser");
        }
        mQueue1 = gst_element_factory_make("queue", "queue1");
        // Sink element. UDP or RTSP client.
        if (is_udp_protocol) {
            // UDP Sink
            if (mEncoderProfile == "H265") {
                mH26xpay = gst_element_factory_make("rtph265pay", "payload");
            } else {
                mH26xpay = gst_element_factory_make("rtph264pay", "payload");
            }
            g_object_set(G_OBJECT(mH26xpay), "pt", 96, NULL);
            mUdpSink = gst_element_factory_make("udpsink", "udp_sink");
            g_object_set(G_OBJECT(mUdpSink), "host", ReadIpAddresFromUdpAddress().c_str(), NULL);
            g_object_set(G_OBJECT(mUdpSink), "port", ReadPortFromUdpAddress(), NULL);
        } else {
            // RTSP client sink
            mRtspSink = gst_element_factory_make("rtspclientsink", "rtsp_sink");
            g_object_set(G_OBJECT(mRtspSink),
                         "protocols", 4, // 4 = tcp
                         "tls-validation-flags", 0,
                         "location", mStreamAddress.c_str(),
                         NULL);
        }

        // Caps definition for source element.
        std::string profile = mEncoderProfile;
        std::stringstream ss;
        std::string gstFormat;
        std::transform(profile.begin(), profile.end(), profile.begin(), ::tolower);
        ss << "video/x-" << profile;
        ss >> gstFormat;
//        g_object_set(G_OBJECT(mAppsrc), "caps",
//                     gst_caps_new_simple(gstFormat.c_str(),
//                                         "width", G_TYPE_INT, mEncoderWidth,
//                                         "height", G_TYPE_INT, mEncoderHeight,
//                                         "framerate", GST_TYPE_FRACTION, mEncoderFps, 1,
//                                         NULL), NULL);

//        mH26xEncFilter = gst_element_factory_make("capsfilter", "encoder_filter");
//        g_object_set(G_OBJECT(mH26xEncFilter), "caps",
//                     gst_caps_new_simple(gstFormat.c_str(),
//                                         "profile", G_TYPE_STRING, "main",
//                                         "stream-format", G_TYPE_STRING, "byte-stream",
//                                         NULL), NULL);
//
        mBus = gst_pipeline_get_bus(GST_PIPELINE(mPipeline));
        mBusWatchId = gst_bus_add_watch(mBus, StreamEventCallBack, this);
        gst_object_unref(mBus);
        mBus = nullptr;

        if (is_udp_protocol) {
            gst_bin_add_many(GST_BIN(mPipeline), mAppsrc/*, mH26xEncFilter*/, mH26xparse, mQueue1, mH26xpay, mUdpSink, NULL);
            gst_element_link_many(mAppsrc, /*mH26xEncFilter,*/ mH26xparse, mQueue1, mH26xpay, mUdpSink, NULL);
        } else {
            gst_bin_add_many(GST_BIN(mPipeline), mAppsrc/*, mH26xEncFilter*/, mH26xparse, mQueue1, mRtspSink, NULL);
            gst_element_link_many(mAppsrc,/* mH26xEncFilter,*/ mH26xparse, mQueue1, mRtspSink, NULL);
        }

        sourceid = 0;
        mLoopThread = g_thread_new("GstThread", (GThreadFunc)Impl::PlayStream, this);

        mNeedDataSignalId = g_signal_connect(mAppsrc, "need-data", G_CALLBACK(NeedDataCallBack), this);
        g_signal_connect(mAppsrc, "enough-data", G_CALLBACK(StopFeedHandler), this);
    }

    void StopStream(void)
    {
        GstFlowReturn ret;

        if (mNeedDataSignalId != 0) {
            g_signal_handler_disconnect(mAppsrc, mNeedDataSignalId);
        }
        if (mAppsrc != nullptr) {
            g_signal_emit_by_name(mAppsrc, "end-of-stream", &ret);
            if (ret != GST_FLOW_OK) {
                g_printerr("Error: Emit end-of-stream failed\n");
            }
        }
        if (mPipeline != nullptr) {
            gst_element_set_state(mPipeline, GST_STATE_NULL);
        }
        if (mLoop != nullptr) {
            g_main_loop_quit(mLoop);
        }
        if (mLoopThread != nullptr) {
            g_thread_join(mLoopThread);
        }
        mIsStreamPlaying = false;
    }

    bool IsStreamPlaying(void) { return mIsStreamPlaying; }

    void SetEncoderWidth(int width)
    {
        if (width > 4096) {
            g_printerr("Width must be smaller than 4096 for H26x encoder profile.\n");
            return;
        }
        if (width % 8 != 0) {
            g_printerr("Width must be multiple of 8 for H26x encoder profile.\n");
            return;
        }
        mEncoderWidth = width;
        //depthAICam->SetEncoderWidth(mEncoderWidth);
    }

    int GetEncoderWidth() { return mEncoderWidth; }

    void SetEncoderHeight(int height)
    {
        if (height > 4096) {
            g_printerr("Height must be smaller than 4096 for H26x encoder profile.\n");
            return;
        }
        if (height % 8 != 0) {
            g_printerr("Height must be multiple of 8 for H26x encoder profile.\n");
            return;
        }
        mEncoderHeight = height;
        //depthAICam->SetEncoderHeight(mEncoderHeight);
    }

    int GetEncoderHeight() { return mEncoderHeight; }

    void SetEncoderFps(int fps)
    {
        if (fps > 60) {
            g_printerr("Too high frames per second.\n");
            return;
        }
        mEncoderFps = fps;
        //depthAICam->SetEncoderFps(mEncoderFps);
    }

    int GetEncoderFps() { return mEncoderFps; }

    void SetEncoderBitrate(int bitrate)
    {
        mEncoderBitrate = bitrate;
        //depthAICam->SetEncoderBitrate(mEncoderBitrate);
    }

    int GetEncoderBitrate() { return mEncoderBitrate; }

    void SetEncoderProfile(std::string profile)
    {
        std::transform(profile.begin(), profile.end(), profile.begin(), ::toupper);
        if (profile != "H264" && profile != "H265") {
            g_printerr("Not valid H26x profile.\n");
            return;
        }
        mEncoderProfile = profile;
        //depthAICam->SetEncoderProfile(mEncoderProfile);
    }

    const std::string & GetEncoderProfile() { return mEncoderProfile; }

    void SetStreamAddress(const std::string address)
    {
        mStreamAddress = address;
//        if (mRtspSink != nullptr) {
//            g_object_set(G_OBJECT(mRtspSink),
//                         "protocols", 4, // 4 = tcp
//                         "location", mStreamAddress.c_str(),
//                         NULL);
//        }
    }

    //DepthAICam *depthAICam;


    static void *PlayStream(gpointer data)
    {
        Impl *depthAIGst = (Impl *)data;
//        DepthAICam * depthAICam = depthAIGst->depthAICam;
//        if (depthAICam != nullptr) {
//            depthAICam->StartStreaming();
//        }
        gst_element_set_state(depthAIGst->mPipeline, GST_STATE_PLAYING);
        g_main_loop_run(depthAIGst->mLoop);
        g_thread_exit(depthAIGst->mLoopThread);

        return nullptr;
    }

    static gboolean gst_is_missing_plugin_message(GstMessage *msg)
    {
        if (GST_MESSAGE_TYPE (msg) != GST_MESSAGE_ELEMENT
            || gst_message_get_structure (msg) == NULL)
            return FALSE;

        return gst_structure_has_name(gst_message_get_structure(msg), "missing-plugin");
    }

    static const gchar *gst_missing_plugin_message_get_description(GstMessage *msg)
    {
        return gst_structure_get_string(gst_message_get_structure(msg), "name");
    }

    static gboolean StreamEventCallBack(GstBus *bus, GstMessage *message, gpointer data)
    {
        (void)bus;
        g_debug("%s: Got %s message\n", __FUNCTION__, GST_MESSAGE_TYPE_NAME(message));

        Impl *depthAIGst = (Impl *)data;
        GstTagList *list = nullptr;

        switch (GST_MESSAGE_TYPE (message)) {
            case GST_MESSAGE_STREAM_STATUS:
                GstStreamStatusType statusType;
                GstElement *element;

                gst_message_parse_stream_status(message, &statusType, &element);
                g_print("Element %s stream status type %d.\n",
                        GST_OBJECT_NAME(message->src),
                        statusType);
                break;

            case GST_MESSAGE_PROGRESS:
                GstProgressType progressType;
                gchar *code, *text;

                gst_message_parse_progress (message, &progressType, &code, &text);
                switch (progressType) {
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
                GstClock *clock;

                gst_message_parse_new_clock(message, &clock);
                g_print("New clock: %s\n", (clock ? GST_OBJECT_NAME(clock) : "NULL"));
                break;

            case GST_MESSAGE_LATENCY:
                g_print("Redistribute latency...\n");
                gst_bin_recalculate_latency(GST_BIN(depthAIGst->mPipeline));
                break;

            case GST_MESSAGE_ELEMENT:
                if (gst_is_missing_plugin_message(message)) {
                    const gchar *desc;

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
                if (g_strrstr(GST_OBJECT_NAME(message->src), "rtspbin")
                    && new_state == GST_STATE_PLAYING) {
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
                gchar  *warnDebug;
                GError *warning;

                gst_message_parse_warning(message, &warning, &warnDebug);
                g_free(warnDebug);

                g_warning("Warning: %s.\n", warning->message);
                g_error_free(warning);
                break;

            case GST_MESSAGE_ERROR:
                gchar  *errDebug;
                GError *error;
                GSource *source;

                gst_message_parse_error(message, &error, &errDebug);
                g_printerr("ERROR from element %s: %s\n",
                           GST_OBJECT_NAME(message->src), error->message);
                g_printerr("Debugging info: %s\n", (errDebug) ? errDebug : "none");
                if (error->code == G_FILE_ERROR_NODEV &&
                    g_strrstr(error->message, "Could not open resource for reading and writing")) {
                    GstFlowReturn ret;
                    if (depthAIGst->mNeedDataSignalId != 0) {
                        g_signal_handler_disconnect(depthAIGst->mAppsrc, depthAIGst->mNeedDataSignalId);
                    }
                    if (depthAIGst->mAppsrc != nullptr) {
                        g_signal_emit_by_name(depthAIGst->mAppsrc, "end-of-stream", &ret);
                        if (ret != GST_FLOW_OK) {
                            g_printerr("Error: Emit end-of-stream failed\n");
                        }
                    }
                    if (depthAIGst->mPipeline != nullptr) {
                        gst_element_set_state(depthAIGst->mPipeline, GST_STATE_NULL);
                    }
                    depthAIGst->mIsStreamPlaying = false;
                    // Restart stream after two seconds.
                    source = g_timeout_source_new(2000);
                    g_source_set_callback(source,
                                          Impl::StreamPlayingRestartCallback,
                                          depthAIGst,
                                          Impl::StreamPlayingRestartDone);
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

    static void NeedDataCallBack(GstElement *appsrc, guint unused_size, gpointer data)
    {
        std::cerr << "NEED DATA: ";
        auto impl = static_cast<DepthAIGStreamer::Impl*>(data);
        if (impl->sourceid == 0)
        {
            impl->sourceid = g_idle_add((GSourceFunc)PushDataHandler, data);
            std::cerr << "Start feeding into source " << std::to_string(impl->sourceid);
        }
        else
        {
            std::cerr << "source already exists";
        }
        std::cerr << std::endl;

    }

    static gboolean PushDataHandler(gpointer data, GstElement* appsrc)
    {
        auto impl = static_cast<DepthAIGStreamer::Impl*>(data);
        CompressedImageMsg::SharedPtr image;
        impl->_message_queue_mutex.lock();
        std::cerr << "PUSH SOME DATA; (" << impl->_message_queue.size() << " chunks left in buffer)" << std::endl;
        if (!impl->_message_queue.empty())
        {
            image = impl->_message_queue.front();
            impl->_message_queue.pop();
        }
        impl->_message_queue_mutex.unlock();

        if (image)
        {
            auto& frame = image->data;

            guint size = frame.size();
            GstBuffer* buffer = gst_buffer_new_allocate(NULL, size, NULL);
            gst_buffer_fill(buffer, 0, (gconstpointer)(&frame[0]), size);

            GST_BUFFER_PTS(buffer) = impl->mGstTimestamp;
            // GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, (int)depthAIGst->mEncoderFps);
            GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 8);
            impl->mGstTimestamp += GST_BUFFER_DURATION(buffer);

            GstFlowReturn ret{};
            g_signal_emit_by_name(impl->mAppsrc, "push-buffer", buffer, &ret);
            gst_buffer_unref(buffer);
            if (ret != GST_FLOW_OK /*&& ret != GST_FLOW_FLUSHING*/)
            {
                GST_DEBUG("PushDataHandler error");
                g_main_loop_quit(impl->mLoop);
                return FALSE;
            }
            return TRUE;
        }
        return FALSE;
    }

    static void StopFeedHandler(GstElement* appsrc, gpointer data)
    {
        std::cerr << "ENOUGH DATA: " << std::endl;
        auto impl = static_cast<DepthAIGStreamer::Impl*>(data);
        if (impl->sourceid != 0)
        {
            g_debug("Stop feeding\n");
            g_source_remove(impl->sourceid);
            impl->sourceid = 0;
        }
    }


    static gboolean StreamPlayingRestartCallback(gpointer user_data)
    {
        Impl *depthAIGst = (Impl *)user_data;

        g_debug("Restart stream because of connection failed.\n");
        if (depthAIGst->mAppsrc) {
            depthAIGst->mNeedDataSignalId = g_signal_connect(
                depthAIGst->mAppsrc,"need-data",
                G_CALLBACK(NeedDataCallBack), depthAIGst);
            g_signal_connect(depthAIGst->mAppsrc, "enough-data", G_CALLBACK(StopFeedHandler), depthAIGst);

        }
        gst_element_set_state(depthAIGst->mPipeline, GST_STATE_PLAYING);

        return G_SOURCE_REMOVE;
    }

    // Use this function to execute code when timer is removed.
    static void StreamPlayingRestartDone(gpointer user_data)
    {
        (void)user_data;
    }

  private:
    std::string ReadIpAddresFromUdpAddress(void)
    {
        // String format is: udp://<ip_addr>:<port>
        std::string addr = mStreamAddress;
        std::string udp_protocol = "udp://";
        addr.erase(0, udp_protocol.size());

        return addr.substr(0, addr.find(":"));
    }

    int ReadPortFromUdpAddress(void)
    {
        // String format is: udp://<ip_addr>:<port>
        std::string addr = mStreamAddress;
        std::string udp_protocol = "udp://";
        addr.erase(0, udp_protocol.size());

        return atoi(addr.substr(addr.find(":") + 1).c_str());
    }
    GMainLoop *mLoop;
    GstElement *mPipeline;
    GstElement *mAppsrc;
    GstElement *mH26xparse;
    GstElement *mH26xpay;
    GstElement *mUdpSink;
    guint mBusWatchId;
    GstBus *mBus;
    guint mNeedDataSignalId;
    GThread *mLoopThread;
    GstElement *mQueue1;
    GstElement *mRtspSink;
    bool mIsStreamPlaying;
    int mEncoderWidth;
    int mEncoderHeight;
    int mEncoderFps;
    int mEncoderBitrate;
    std::string mEncoderProfile;
    GstClockTime mGstTimestamp;
    std::string mStreamAddress;
    GstElement *mTestSrc;
    GstElement *mTextOverlay;
    GstElement *mH26xEnc;
    GstElement *mTestSrcFilter;
    GstElement *mH26xEncFilter;
    guint mStreamPlayingCheckTimerId;
    GMainContext *mLoopContext;
    guint sourceid = 0;
};




}  // namespace depthai_ctrl

#endif  // FOG_SW_DEPTHAI_GSTREAMER_IMPL_H
