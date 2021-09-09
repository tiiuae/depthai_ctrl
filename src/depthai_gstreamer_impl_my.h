#ifndef FOG_SW_DEPTHAI_GSTREAMER_IMPL_H
#define FOG_SW_DEPTHAI_GSTREAMER_IMPL_H

#include <arpa/inet.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/gstbus.h>
#include <gst/gstcaps.h>
#include <gst/gstelement.h>
#include <gst/gstpipeline.h>
#include <nlohmann/json.hpp>
#include <mutex>
#include <queue>

namespace depthai_ctrl
{

/// GStreamer part independent from ROS
struct DepthAIGStreamer::Impl
{
    std::queue<CompressedImageMsg::SharedPtr> _message_queue;
    std::mutex _message_queue_mutex;

    bool mIsStreamPlaying = false;
    int mEncoderWidth = 1280;
    int mEncoderHeight = 720;
    int mEncoderFps = 25;
    int mEncoderBitrate = 3000000;
    std::string mEncoderProfile = "H264";
    std::string mStreamAddress = "192.168.0.1";

    GMainLoop* mLoop = nullptr;
    GstElement* mPipeline = nullptr;
    GstElement* mAppsrc = nullptr;

    guint mBusWatchId = 0;
    GstBus* mBus = nullptr;
    guint mNeedDataSignalId = 0;
    GThread* mLoopThread = nullptr;
    GstClockTime mGstTimestamp = 0;
    GMainContext* mLoopContext = nullptr;
    guint sourceid = 0;
    GTimer* timer;

    Impl(int* argc, char** argv[])
    {
        gst_init(argc, argv);
        auto mLoopContext = g_main_context_default();
        mLoop = g_main_loop_new(mLoopContext, false);
        timer = g_timer_new();
    }

    ~Impl()
    {
        g_timer_destroy(timer);
        DestroyPipeline();
        if (mLoop != nullptr)
        {
            g_main_loop_unref(mLoop);
            mLoop = nullptr;
        }
        gst_deinit();
    }

    bool IsVideoStreamAvailable()
    {
        std::lock_guard<std::mutex> lock(_message_queue_mutex);
        return !_message_queue.empty();
    }

    std::string ReadIpFromUdpAddress()
    {
        // String format is: udp://<ip_addr>:<port>
        std::string addr = mStreamAddress;
        std::string udp_protocol = "udp://";
        addr.erase(0, udp_protocol.size());

        return addr.substr(0, addr.find(":"));
    }

    int ReadPortFromUdpAddress()
    {
        // String format is: udp://<ip_addr>:<port>
        std::string addr = mStreamAddress;
        std::string udp_protocol = "udp://";
        addr.erase(0, udp_protocol.size());

        return atoi(addr.substr(addr.find(":") + 1).c_str());
    }

    //    void PushVideoChunk(CompressedImageMsg::SharedPtr video_data)
    //    {
    //        if(mAppsrc != nullptr)
    //        {
    //            auto& frame = video_data->data;
    //            guint size = frame.size();
    //            GstBuffer* buffer = gst_buffer_new_allocate(NULL, size, NULL);
    //            gst_buffer_fill(buffer, 0, (gconstpointer)(&frame[0]), size);
    //            GST_BUFFER_PTS(buffer) = mGstTimestamp;
    //            GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 1);
    //            mGstTimestamp += GST_BUFFER_DURATION(buffer);
    //            gst_app_src_push_buffer((GstAppSrc*)mAppsrc, buffer);
    //        }
    //    }

    static void* MainLoopRunner(gpointer data);
    static gboolean StreamRestartHandler(gpointer data);
    static gboolean BusMessageHandler(GstBus* bus, GstMessage* message, gpointer data);

    static void NeedDataHandler(GstElement* appsrc, guint unused_size, gpointer data)
    {
        std::cerr << "NEED DATA: " << std::endl;
        auto impl = static_cast<DepthAIGStreamer::Impl*>(data);
        if (impl->sourceid == 0)
        {
            g_debug("Start feeding\n");
            impl->sourceid = g_idle_add((GSourceFunc)PushDataHandler, data);
        }
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

    static gboolean PushDataHandler(GstElement* appsrc, gpointer data)
    {
        auto impl = static_cast<DepthAIGStreamer::Impl*>(data);
        CompressedImageMsg::SharedPtr image;
        impl->_message_queue_mutex.lock();
        std::cerr << "TRY TO GET SOME DATA: " << impl->_message_queue.size() << std::endl;
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
            GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 1);
            impl->mGstTimestamp += GST_BUFFER_DURATION(buffer);

            GstFlowReturn ret{};
            g_signal_emit_by_name(impl->mAppsrc, "push-buffer", buffer, &ret);
            gst_buffer_unref(buffer);
            if (ret != GST_FLOW_OK /*&& ret != GST_FLOW_FLUSHING*/)
            {
                GST_DEBUG("some error");
                // g_main_loop_quit(depthAIGst->mLoop);
                return FALSE;
            }
            return TRUE;
        }
        return FALSE;
    }

    // Use this function to execute code when timer is removed.
    static void StreamPlayingRestartDone(gpointer) { ; }

    void DestroyPipeline()
    {
        if (mBusWatchId != 0)
        {
            g_source_remove(mBusWatchId);
            mBusWatchId = 0;
        }
        if (g_main_loop_is_running(mLoop))
        {
            g_main_loop_quit(mLoop);
        }
        if (mPipeline != nullptr)
        {
            gst_element_set_state(mPipeline, GST_STATE_PAUSED);
            // gst_element_set_state(mPipeline, GST_STATE_NULL);
            gst_object_unref(GST_OBJECT(mPipeline));
            mPipeline = nullptr;
        }
        //        if (mBus != nullptr)
        //        {
        //            gst_object_unref(mBus);
        //            mBus = nullptr;
        //        }
    }

    //    void BuildDefaultPipeline()
    //    {
    //        bool is_udp_protocol = (mStreamAddress.find("udp://") == 0);
    //
    //        mPipeline = gst_pipeline_new("default_pipeline");
    //
    //        // Video test source.
    //        auto mTestSrc = gst_element_factory_make("videotestsrc", "source");
    //        g_object_set(G_OBJECT(mTestSrc), "pattern", 2, NULL);
    //        auto mTestSrcFilter = gst_element_factory_make("capsfilter", "source_filter");
    //        g_object_set(G_OBJECT(mTestSrcFilter),
    //                     "caps",
    //                     gst_caps_new_simple("video/x-raw",
    //                                         "format",
    //                                         G_TYPE_STRING,
    //                                         "I420",
    //                                         "width",
    //                                         G_TYPE_INT,
    //                                         mEncoderWidth,
    //                                         "height",
    //                                         G_TYPE_INT,
    //                                         mEncoderHeight,
    //                                         "framerate",
    //                                         GST_TYPE_FRACTION,
    //                                         mEncoderFps,
    //                                         1,
    //                                         NULL),
    //                     NULL);
    //
    //        // Text overlay.
    //        auto mTextOverlay = gst_element_factory_make("textoverlay", "text");
    //        g_object_set(G_OBJECT(mTextOverlay),
    //                     "text",
    //                     "Camera not detected!",
    //                     "valignment",
    //                     4,  // 4 = center
    //                     "halignment",
    //                     1,  // 1 = center
    //                     "font-desc",
    //                     "Sans, 42",
    //                     NULL);
    //
    //        GstElement* mH26xEnc{};
    //        GstElement* mH26xparse{};
    //        GstElement* mH26xpay{};
    //        // GstElement* mUdpSink{};
    //        // GstElement* mRtspSink{};
    //        GstElement* mSink{};
    //        // Software encoder and parser.
    //        if (mEncoderProfile == "H265")
    //        {
    //            mH26xEnc = gst_element_factory_make("x265enc", "encoder");
    //            g_object_set(G_OBJECT(mH26xEnc),
    //                         "bitrate",
    //                         500,  // 500 kbit/sec
    //                         "speed-preset",
    //                         2,  // 2 = superfast
    //                         "tune",
    //                         4,  // 4 = zerolatency
    //                         NULL);
    //            mH26xparse = gst_element_factory_make("h265parse", "parser");
    //        }
    //        else
    //        {
    //            mH26xEnc = gst_element_factory_make("x264enc", "encoder");
    //            mH26xparse = gst_element_factory_make("h264parse", "parser");
    //        }
    //
    //        // Sink element. UDP or RTSP client.
    //        if (is_udp_protocol)
    //        {
    //            // UDP Sink
    //            if (mEncoderProfile == "H265")
    //            {
    //                mH26xpay = gst_element_factory_make("rtph265pay", "payload");
    //            }
    //            else
    //            {
    //                mH26xpay = gst_element_factory_make("rtph264pay", "payload");
    //            }
    //            g_object_set(G_OBJECT(mH26xpay), "pt", 96, NULL);
    //            mSink = gst_element_factory_make("udpsink", "udp_sink");
    //            g_object_set(G_OBJECT(mSink), "host", ReadIpFromUdpAddress().c_str(), NULL);
    //            g_object_set(G_OBJECT(mSink), "port", ReadPortFromUdpAddress(), NULL);
    //        }
    //        else
    //        {
    //            // RTSP client sink
    //            mSink = gst_element_factory_make("rtspclientsink", "rtsp_sink");
    //            g_object_set(G_OBJECT(mSink),
    //                         "protocols",
    //                         4,  // 4 = tcp
    //                         "tls-validation-flags",
    //                         0,
    //                         "location",
    //                         mStreamAddress.c_str(),
    //                         NULL);
    //        }
    //
    //        // Caps definition for source element.
    //        std::string profile = mEncoderProfile;
    //        std::stringstream ss;
    //        std::string gstFormat;
    //        std::transform(profile.begin(), profile.end(), profile.begin(), ::tolower);
    //        ss << "video/x-" << profile;
    //        ss >> gstFormat;
    //        auto mH26xEncFilter = gst_element_factory_make("capsfilter", "encoder_filter");
    //        g_object_set(G_OBJECT(mH26xEncFilter),
    //                     "caps",
    //                     gst_caps_new_simple(gstFormat.c_str(),
    //                                         "profile",
    //                                         G_TYPE_STRING,
    //                                         "baseline",
    //                                         "pass",
    //                                         G_TYPE_INT,
    //                                         5,
    //                                         "trellis",
    //                                         G_TYPE_BOOLEAN,
    //                                         false,
    //                                         "tune",
    //                                         G_TYPE_STRING,
    //                                         "zero-latency",
    //                                         "threads",
    //                                         G_TYPE_INT,
    //                                         0,
    //                                         "speed-preset",
    //                                         G_TYPE_STRING,
    //                                         "superfast",
    //                                         "subme",
    //                                         G_TYPE_INT,
    //                                         1,
    //                                         "bitrate",
    //                                         G_TYPE_INT,
    //                                         4000,
    //                                         NULL),
    //                     NULL);
    //
    //        mBus = gst_pipeline_get_bus(GST_PIPELINE(mPipeline));
    //        mBusWatchId = gst_bus_add_watch(mBus, BusMessageHandler, gpointer(this));
    //        gst_object_unref(mBus);
    //        mBus = nullptr;
    //
    ////    if (is_udp_protocol) {
    ////        gst_bin_add_many(GST_BIN(mPipeline), mTestSrc, mTestSrcFilter, mTextOverlay, mH26xEnc, mH26xEncFilter,
    /// mH26xparse, mH26xpay, mUdpSink, NULL); /        gst_element_link_many(mTestSrc, mTestSrcFilter, mTextOverlay,
    /// mH26xEnc, mH26xEncFilter, mH26xparse, mH26xpay, mUdpSink, NULL); /    } else { /
    /// gst_bin_add_many(GST_BIN(mPipeline), mTestSrc, mTestSrcFilter, mTextOverlay, mH26xEnc, mH26xEncFilter,
    /// mH26xparse, mRtspSink, NULL); /        gst_element_link_many(mTestSrc, mTestSrcFilter, mTextOverlay, mH26xEnc,
    /// mH26xEncFilter, mH26xparse, mRtspSink, NULL); /    }
    //        gst_bin_add_many(GST_BIN(mPipeline),
    //                         mTestSrc,
    //                         mTestSrcFilter,
    //                         mTextOverlay,
    //                         mH26xEnc,
    //                         mH26xEncFilter,
    //                         mH26xparse,
    //                         mH26xpay,
    //                         mSink,
    //                         NULL);
    //        gst_element_link_many(
    //            mTestSrc, mTestSrcFilter, mTextOverlay, mH26xEnc, mH26xEncFilter, mH26xparse, mH26xpay, mSink, NULL);
    //
    //        mLoopThread = g_thread_new("GstThread", PlayStream, this);
    //    }

    void CreatePipeline2()
    {
        const bool is_udp_protocol = (mStreamAddress.find("udp://") == 0);
        const std::string h26xparse = (mEncoderProfile == "H246") ? "h264parse" : "h265parse";
        const std::string gstFormat = (mEncoderProfile == "H246") ? "video/x-h264" : "video/x-h265";
        std::string payload = " ";
        std::string sink{};

        if (is_udp_protocol)
        {
            sink = "udpsink host=127.0.0.1 port=1234";
            payload = (mEncoderProfile == "H246") ? "! rtph264pay " : "! rtph265pay ";
        }
        else
        {
            sink = "rtspclientsink location=" + mStreamAddress;
        }
        //,profile=main,stream-format=byte-stream
        std::string pipeline_string = "appsrc name=source ! capsfilter caps="+gstFormat+"! h264parse ! queue " + payload + "! " + sink;
        std::cout << pipeline_string << std::endl;
        mPipeline = gst_parse_launch(pipeline_string.c_str(), NULL);
        g_assert(mPipeline);

        mAppsrc = gst_bin_get_by_name(GST_BIN(mPipeline), "source");
        g_assert(mAppsrc);
        g_assert(GST_IS_APP_SRC(mAppsrc));

        mNeedDataSignalId = g_signal_connect(mAppsrc, "need-data", G_CALLBACK(NeedDataHandler), this);
        g_signal_connect(mAppsrc, "enough-data", G_CALLBACK(StopFeedHandler), this);

        auto caps = gst_caps_new_simple(gstFormat.c_str(),
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
                                        NULL);

        g_object_set(G_OBJECT(mAppsrc), "caps", caps, NULL);

        auto mBus = gst_pipeline_get_bus(GST_PIPELINE(mPipeline));
        g_assert(mBus);

        mBusWatchId = gst_bus_add_watch(mBus, BusMessageHandler, this);
        gst_object_unref(mBus);

        mLoopThread = g_thread_new("GstThread", MainLoopRunner, this);
        gst_element_set_state(mPipeline, GST_STATE_PLAYING);
    }

    //    void CreatePipeline()
    //    {
    //        //        if (!IsVideoStreamAvailable())
    //        //        {
    //        //            g_printerr("Warning: video stream is not available. Start default stream.\n");
    //        //            BuildDefaultPipeline();
    //        //            return;
    //        //        }
    //
    //        const bool is_udp_protocol = (mStreamAddress.find("udp://") == 0);
    //
    //        // App Source element.
    //        mAppsrc = gst_element_factory_make("appsrc", "source");
    //        g_object_set(G_OBJECT(mAppsrc),
    //                     "do-timestamp",
    //                     true,
    //                     "is-live",
    //                     true,
    //                     //"block", true,
    //                     "stream-type",
    //                     0,
    //                     "format",
    //                     "GST_FORMAT_TIME",
    //                     nullptr);
    //
    //        // gst_util_set_object_arg(G_OBJECT(mAppsrc), "format", "GST_FORMAT_TIME");
    //
    //        // Caps definition for App Source element.
    //        std::string profile = mEncoderProfile;
    //        std::stringstream ss;
    //        std::string gstFormat;
    //        std::transform(profile.begin(), profile.end(), profile.begin(), ::tolower);
    //        ss << "video/x-" << profile;
    //        ss >> gstFormat;
    //        g_object_set(G_OBJECT(mAppsrc),
    //                     "caps",
    //                     gst_caps_new_simple(gstFormat.c_str(),
    //                                         "width",
    //                                         G_TYPE_INT,
    //                                         mEncoderWidth,
    //                                         "height",
    //                                         G_TYPE_INT,
    //                                         mEncoderHeight,
    //                                         "framerate",
    //                                         GST_TYPE_FRACTION,
    //                                         mEncoderFps,
    //                                         1,
    //                                         NULL),
    //                     NULL);
    //
    //        // H26x parser. Is this really needed?
    //        GstElement* mH26xparse{};
    //        GstElement* mQueue1{};
    //        GstElement* mH26xpay{};
    //        GstElement* mUdpSink{};
    //        GstElement* mRtspSink{};
    //
    //        if (mEncoderProfile == "H265")
    //        {
    //            mH26xparse = gst_element_factory_make("h265parse", "parser");
    //        }
    //        else
    //        {
    //            mH26xparse = gst_element_factory_make("h264parse", "parser");
    //        }
    //        mQueue1 = gst_element_factory_make("queue", "queue1");
    //
    //        // Sink element. UDP or RTSP client.
    //        if (is_udp_protocol)
    //        {
    //            // UDP Sink
    //            if (mEncoderProfile == "H265")
    //            {
    //                mH26xpay = gst_element_factory_make("rtph265pay", "payload");
    //            }
    //            else
    //            {
    //                mH26xpay = gst_element_factory_make("rtph264pay", "payload");
    //            }
    //            g_object_set(G_OBJECT(mH26xpay), "pt", 96, NULL);
    //            mUdpSink = gst_element_factory_make("udpsink", "udp_sink");
    //            g_object_set(G_OBJECT(mUdpSink), "host", ReadIpFromUdpAddress().c_str(), NULL);
    //            g_object_set(G_OBJECT(mUdpSink), "port", ReadPortFromUdpAddress(), NULL);
    //        }
    //        else
    //        {
    //            // RTSP client sink
    //            mRtspSink = gst_element_factory_make("rtspclientsink", "rtsp_sink");
    //            g_object_set(G_OBJECT(mRtspSink),
    //                         "protocols",
    //                         4,  // 4 = tcp
    //                         "tls-validation-flags",
    //                         0,
    //                         "location",
    //                         mStreamAddress.c_str(),
    //                         NULL);
    //        }
    //
    //        auto mH26xEncFilter = gst_element_factory_make("capsfilter", "encoder_filter");
    //        g_object_set(G_OBJECT(mH26xEncFilter),
    //                     "caps",
    //                     gst_caps_new_simple(gstFormat.c_str(),
    //                                         "profile",
    //                                         G_TYPE_STRING,
    //                                         "main",
    //                                         "stream-format",
    //                                         G_TYPE_STRING,
    //                                         "byte-stream",
    //                                         NULL),
    //                     NULL);
    //
    //        mBus = gst_pipeline_get_bus(GST_PIPELINE(mPipeline));
    //        mBusWatchId = gst_bus_add_watch(mBus, BusMessageHandler, this);
    //        gst_object_unref(mBus);
    //        mBus = nullptr;
    //
    //        mPipeline = gst_pipeline_new("rgbCamSink_pipeline");
    //        if (is_udp_protocol)
    //        {
    //            gst_bin_add_many(
    //                GST_BIN(mPipeline), mAppsrc, mH26xEncFilter, mH26xparse, mQueue1, mH26xpay, mUdpSink, NULL);
    //            gst_element_link_many(mAppsrc, mH26xEncFilter, mH26xparse, mQueue1, mH26xpay, mUdpSink, NULL);
    //        }
    //        else
    //        {
    //            gst_bin_add_many(GST_BIN(mPipeline), mAppsrc, mH26xEncFilter, mH26xparse, mQueue1, mRtspSink, NULL);
    //            gst_element_link_many(mAppsrc, mH26xEncFilter, mH26xparse, mQueue1, mRtspSink, NULL);
    //        }
    //
    //        mAppsrc = gst_bin_get_by_name (GST_BIN(mPipeline), "source");
    //
    //        mNeedDataSignalId = g_signal_connect(mAppsrc, "need-data", G_CALLBACK(NeedDataHandler), this);
    //        g_signal_connect (mAppsrc, "enough-data", G_CALLBACK(StopFeedHandler), this);
    //
    //        mLoopThread = g_thread_new("GstThread", MainLoopRunner, this);
    //
    //
    //        // g_signal_connect(mAppsrc, "enough-data", G_CALLBACK(StopFeedHandler), this);
    //    }
};
//
// void DepthAIGStreamer::Impl::NeedDataHandler(GstElement* appsrc, guint unused_size, gpointer data)
//{
//    auto impl = static_cast<DepthAIGStreamer::Impl*>(data);
//    if (impl->sourceid == 0)
//    {
//        g_debug("Start feeding\n");
//        impl->sourceid = g_idle_add((GSourceFunc)PushData, data);
//    }
//}
//
// gboolean DepthAIGStreamer::Impl::PushData(gpointer data)
//{
//    auto impl = static_cast<DepthAIGStreamer::Impl*>(data);
//    CompressedImageMsg::SharedPtr image;
//    impl->_message_queue_mutex.lock();
//    std::cerr << "TRY TO GET SOME DATA: " << impl->_message_queue.size() << std::endl;
//    // image = depthAIGst->_last_message;
//    if (!impl->_message_queue.empty())
//    {
//        image = impl->_message_queue.front();
//        if (impl->_message_queue.size() > 1)
//        {
//            impl->_message_queue.pop();
//        }
//    }
//    impl->_message_queue_mutex.unlock();
//
//    if (image)
//    {
//        auto& frame = image->data;
//
//        guint size = frame.size();
//        GstBuffer* buffer = gst_buffer_new_allocate(NULL, size, NULL);
//        gst_buffer_fill(buffer, 0, (gconstpointer)(&frame[0]), size);
//
//        GST_BUFFER_PTS(buffer) = impl->mGstTimestamp;
//        // GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, (int)depthAIGst->mEncoderFps);
//        GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 1);
//        impl->mGstTimestamp += GST_BUFFER_DURATION(buffer);
//
//        //        GST_BUFFER_PTS(buffer) = GST_CLOCK_TIME_NONE;//depthAIGst->mGstTimestamp;
//        //        GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, 1);
//
//        GST_DEBUG("feed buffer");
//        GstFlowReturn ret{};
//        g_signal_emit_by_name(impl->mAppsrc, "push-buffer", buffer, &ret);
//        gst_buffer_unref(buffer);
//        if (ret != GST_FLOW_OK /*&& ret != GST_FLOW_FLUSHING*/)
//        {
//            GST_DEBUG("some error");
//            // g_main_loop_quit(depthAIGst->mLoop);
//            return FALSE;
//        }
//        return TRUE;
//    }
//    return FALSE;
//}
//
// void DepthAIGStreamer::Impl::StopFeedHandler(GstElement* source, gpointer data)
//{
//    auto impl = static_cast<DepthAIGStreamer::Impl*>(data);
//    if (impl->sourceid != 0)
//    {
//        g_debug("Stop feeding\n");
//        g_source_remove(impl->sourceid);
//        impl->sourceid = 0;
//    }
//}

gboolean DepthAIGStreamer::Impl::StreamRestartHandler(gpointer data)
{
    g_debug("Restart stream because of connection failed.\n");
    auto impl = static_cast<DepthAIGStreamer::Impl*>(data);
    //    if (impl->mAppsrc)
    //    {
    //        impl->mNeedDataSignalId = g_signal_connect(impl->mAppsrc, "need-data", G_CALLBACK(NeedDataHandler),
    //        data); g_signal_connect(impl->mAppsrc, "enough-data", G_CALLBACK(StopFeedHandler), data);
    //    }
    // gst_element_set_state(impl->mPipeline, GST_STATE_PLAYING);

    return G_SOURCE_REMOVE;
}

void* DepthAIGStreamer::Impl::MainLoopRunner(gpointer data)
{
    auto impl = static_cast<DepthAIGStreamer::Impl*>(data);
    gst_element_set_state(impl->mPipeline, GST_STATE_PLAYING);
    g_main_loop_run(impl->mLoop);
    gst_element_set_state(impl->mPipeline, GST_STATE_PAUSED);
    // g_thread_exit(impl->mLoopThread);
    return nullptr;
}

gboolean DepthAIGStreamer::Impl::BusMessageHandler(GstBus* bus, GstMessage* message, gpointer data)
{
    (void)bus;
    g_debug("%s: Got %s message\n", __FUNCTION__, GST_MESSAGE_TYPE_NAME(message));

    auto impl = static_cast<DepthAIGStreamer::Impl*>(data);
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
            gst_bin_recalculate_latency(GST_BIN(impl->mPipeline));
            break;

        case GST_MESSAGE_ELEMENT:
            if (GST_MESSAGE_TYPE(message) != GST_MESSAGE_ELEMENT || gst_message_get_structure(message) == NULL)
            {
                break;
            }
            else if (gst_structure_has_name(gst_message_get_structure(message), "missing-plugin"))
            {
                const gchar* desc;
                desc = gst_structure_get_string(gst_message_get_structure(message), "name");
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
                impl->mIsStreamPlaying = true;
            }
            break;

        case GST_MESSAGE_EOS:
            g_print("End of stream.\n");
            g_main_loop_quit(impl->mLoop);
            break;

        case GST_MESSAGE_TAG:
        {
            GstTagList* list = gst_tag_list_new_empty();

            gst_message_parse_tag(message, &list);

            g_print("Tag: %s.\n", gst_tag_list_to_string(list));
            gst_tag_list_unref(list);
            break;
        }
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
                if (impl->mNeedDataSignalId != 0)
                {
                    g_signal_handler_disconnect(impl->mAppsrc, impl->mNeedDataSignalId);
                }
                if (impl->mAppsrc != nullptr)
                {
                    g_signal_emit_by_name(impl->mAppsrc, "end-of-stream", &ret);
                    if (ret != GST_FLOW_OK)
                    {
                        g_printerr("Error: Emit end-of-stream failed\n");
                    }
                }
                if (impl->mPipeline != nullptr)
                {
                    gst_element_set_state(impl->mPipeline, GST_STATE_NULL);
                }
                impl->mIsStreamPlaying = false;
                // Restart stream after two seconds.
                source = g_timeout_source_new(2000);
                g_source_set_callback(source, (GSourceFunc)(StreamRestartHandler), data, StreamPlayingRestartDone);
                g_source_attach(source, impl->mLoopContext);
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

}  // namespace depthai_ctrl

#endif  // FOG_SW_DEPTHAI_GSTREAMER_IMPL_H
