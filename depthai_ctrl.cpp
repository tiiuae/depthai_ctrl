#include <memory>
#include <thread>
#include <iostream>
#include <chrono>
#include <stdlib.h>
#include <algorithm>
#include <string>
#include <cctype>
#include <arpa/inet.h>

#include <depthai/depthai.hpp>

#include <gst/gst.h>
#include <gst/gstbus.h>
#include <gst/gstpipeline.h>
#include <gst/gstelement.h>
#include <gst/gstcaps.h>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

#define DEPTHAI_CTRL_VER_MAJOR 0
#define DEPTHAI_CTRL_VER_MINOR 5
#define DEPTHAI_CTRL_VER_PATCH 3

#define DEPTHAI_CTRL_VERSION (DEPTHAI_CTRL_VER_MAJOR * 10000 + DEPTHAI_CTRL_VER_MINOR * 100 + DEPTHAI_CTRL_VER_PATCH)


class DepthAICam
{
    public:
        DepthAICam() : device(nullptr), encColorOutput(nullptr), mIsDeviceAvailable(true),
                       mEncoderWidth(1280), mEncoderHeight(720), mEncoderFps(25),
                       mEncoderBitrate(3000000),
                       mEncoderProfile(dai::VideoEncoderProperties::Profile::H264_MAIN)
        {
            try {
                device = new dai::Device(mPipeline, true);
            } catch (const std::runtime_error& err) {
                std::cout << "DepthAI runtime error: " << err.what() << std::endl;
                mIsDeviceAvailable = false;
            }
        }

        ~DepthAICam()
        {
            if (device != nullptr) {
                delete(device);
            }
        }

        void StartStreaming(void)
        {
            if (!mIsDeviceAvailable) {
                return;
            }
            if (encColorOutput != nullptr) {
                // Pipeline started.
                // TODO: check if DepthAI API has a better way to check this.
                return;
            }
            if (device != nullptr) {
                delete(device);
            }
            BuildPipeline();
            try {
                device = new dai::Device(mPipeline, true);
            } catch (const std::runtime_error& err) {
                std::cout << "DepthAI runtime error: " << err.what() << std::endl;
                mIsDeviceAvailable = false;
                return;
            }
            device->startPipeline();

            encColorOutput = device->getOutputQueue("enc26xColor", 30, true);
            colorCamInput = device->getInputQueue("colorCamCtrl");

            dai::CameraControl colorCamCtrl;
            colorCamCtrl.setAutoFocusMode(dai::RawCameraControl::AutoFocusMode::AUTO);
            //colorCamCtrl.setManualFocus(255);
            colorCamInput->send(colorCamCtrl);
        }

        std::shared_ptr<dai::ImgFrame> GetFrame(void)
        {
            return encColorOutput->get<dai::ImgFrame>();
        }

        bool IsDeviceAvailable(void)
        {
            return mIsDeviceAvailable;
        }

        void BuildPipeline(void)
        {
            colorCam = mPipeline.create<dai::node::ColorCamera>();
            colorCamVideoEnc = mPipeline.create<dai::node::VideoEncoder>();
            colorCamXLinkOut = mPipeline.create<dai::node::XLinkOut>();

            colorCamXLinkOut->setStreamName("enc26xColor");
            colorCam->setBoardSocket(dai::CameraBoardSocket::RGB);
            colorCam->setVideoSize(mEncoderWidth, mEncoderHeight);
            colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);

            colorCamVideoEnc->setDefaultProfilePreset(mEncoderWidth,
                                                    mEncoderHeight,
                                                    mEncoderFps,
                                                    mEncoderProfile);
            colorCamVideoEnc->setBitrate(mEncoderBitrate);
            colorCam->video.link(colorCamVideoEnc->input);
            colorCamVideoEnc->bitstream.link(colorCamXLinkOut->input);

            colorCamXLinkIn = mPipeline.create<dai::node::XLinkIn>();
            colorCamXLinkIn->setStreamName("colorCamCtrl");

            colorCamXLinkIn->out.link(colorCam->inputControl);
        }

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
        }

        int GetEncoderHeight() { return mEncoderHeight; }

        void SetEncoderFps(int fps)
        {
            if (fps > 60) {
                g_printerr("Too high frames per second.\n");
                return;
            }
            mEncoderFps = fps;
        }

        int GetEncoderFps() { return mEncoderFps; }

        void SetEncoderBitrate(int bitrate)
        {
            mEncoderBitrate = bitrate;
        }

        int GetEncoderBitrate() { return mEncoderBitrate; }

        void SetEncoderProfile(std::string profile)
        {
            std::transform(profile.begin(), profile.end(), profile.begin(), ::toupper);
            if (profile != "H264" && profile != "H265") {
                g_printerr("Not valid H26x profile.\n");
                return;
            }
            if (profile == "H265") {
                mEncoderProfile = dai::VideoEncoderProperties::Profile::H265_MAIN;
            } else {
                mEncoderProfile = dai::VideoEncoderProperties::Profile::H264_MAIN;
            }
        }

        const dai::VideoEncoderProperties::Profile & GetEncoderProfile() { return mEncoderProfile; }

    private:
        dai::Device *device;
        dai::Pipeline mPipeline;
        std::shared_ptr<dai::DataOutputQueue> encColorOutput;
        std::shared_ptr<dai::node::ColorCamera> colorCam;
        std::shared_ptr<dai::node::VideoEncoder> colorCamVideoEnc;
        std::shared_ptr<dai::node::XLinkOut> colorCamXLinkOut;
        std::shared_ptr<dai::node::XLinkIn> colorCamXLinkIn;
        std::shared_ptr<dai::DataInputQueue> colorCamInput;
        bool mIsDeviceAvailable;
        int mEncoderWidth;
        int mEncoderHeight;
        int mEncoderFps;
        int mEncoderBitrate;
        dai::VideoEncoderProperties::Profile mEncoderProfile;
};


class DepthAIGst
{
    public:
        DepthAIGst(int argc, char *argv[]) : depthAICam(nullptr), mLoop(nullptr), mPipeline(nullptr), mAppsrc(nullptr),
                                            mH26xparse(nullptr), mH26xpay(nullptr), mUdpSink(nullptr), mBusWatchId(0),
                                            mBus(nullptr), mNeedDataSignalId(0), mLoopThread(nullptr), mQueue1(nullptr),
                                            mRtspSink(nullptr), mIsStreamPlaying(false), mEncoderWidth(1280),
                                            mEncoderHeight(720), mEncoderFps(25), mEncoderBitrate(3000000),
                                            mEncoderProfile("H264"), mGstTimestamp(0), mTestSrc(nullptr),
                                            mTextOverlay(nullptr), mH26xEnc(nullptr), mTestSrcFilter(nullptr),
                                            mH26xEncFilter(nullptr), mStreamPlayingCheckTimerId(0)
        {
            mStreamAddress = "";
            gst_init(&argc, &argv);
            mLoopContext = g_main_context_default();
            mLoop = g_main_loop_new(mLoopContext, false);
            depthAICam = new DepthAICam();
        }

        ~DepthAIGst()
        {
            if (depthAICam != nullptr) {
                delete(depthAICam);
            }
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
                g_object_set(G_OBJECT(mH26xEnc),
                    "pass", 0, // cbr
                    "bitrate", 500, // 500 kbit/sec
                    "tune", 4, // 4 = zerolatency
                    NULL);
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
                    "profile", G_TYPE_STRING, "main",
                    "stream-format", G_TYPE_STRING, "byte-stream",
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

            mLoopThread = g_thread_new("GstThread", (GThreadFunc)DepthAIGst::PlayStream, this);
        }

        void CreatePipeLine(void)
        {
            if (!depthAICam->IsDeviceAvailable()) {
                g_printerr("Warning: device not available. Start default stream.\n");
                BuildDefaultPipeline();
                return;
            }

            bool is_udp_protocol = (mStreamAddress.find("udp://") == 0);

            mPipeline = gst_pipeline_new("rgbCamSink_pipeline");
            // Source element.
            mAppsrc = gst_element_factory_make("appsrc", "source");
            g_object_set(G_OBJECT(mAppsrc),
                "do-timestamp", true,
                "is-live", true,
                "block", true,
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
            g_object_set(G_OBJECT(mAppsrc), "caps",
                gst_caps_new_simple(gstFormat.c_str(),
                    "width", G_TYPE_INT, mEncoderWidth,
                    "height", G_TYPE_INT, mEncoderHeight,
                    "framerate", GST_TYPE_FRACTION, mEncoderFps, 1,
                    NULL), NULL);

            mH26xEncFilter = gst_element_factory_make("capsfilter", "encoder_filter");
            g_object_set(G_OBJECT(mH26xEncFilter), "caps",
                gst_caps_new_simple(gstFormat.c_str(),
                    "profile", G_TYPE_STRING, "main",
                    "stream-format", G_TYPE_STRING, "byte-stream",
                    NULL), NULL);

            mBus = gst_pipeline_get_bus(GST_PIPELINE(mPipeline));
            mBusWatchId = gst_bus_add_watch(mBus, StreamEventCallBack, this);
            gst_object_unref(mBus);
            mBus = nullptr;
            
            if (is_udp_protocol) {
                gst_bin_add_many(GST_BIN(mPipeline), mAppsrc, mH26xEncFilter, mH26xparse, mQueue1, mH26xpay, mUdpSink, NULL);
                gst_element_link_many(mAppsrc, mH26xEncFilter, mH26xparse, mQueue1, mH26xpay, mUdpSink, NULL);
            } else {
                gst_bin_add_many(GST_BIN(mPipeline), mAppsrc, mH26xEncFilter, mH26xparse, mQueue1, mRtspSink, NULL);
                gst_element_link_many(mAppsrc, mH26xEncFilter, mH26xparse, mQueue1, mRtspSink, NULL);
            }

            mLoopThread = g_thread_new("GstThread", (GThreadFunc)DepthAIGst::PlayStream, this);

            mNeedDataSignalId = g_signal_connect(mAppsrc, "need-data", G_CALLBACK(NeedDataCallBack), this);
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
            depthAICam->SetEncoderWidth(mEncoderWidth);
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
            depthAICam->SetEncoderHeight(mEncoderHeight);
        }

        int GetEncoderHeight() { return mEncoderHeight; }

        void SetEncoderFps(int fps)
        {
            if (fps > 60) {
                g_printerr("Too high frames per second.\n");
                return;
            }
            mEncoderFps = fps;
            depthAICam->SetEncoderFps(mEncoderFps);
        }

        int GetEncoderFps() { return mEncoderFps; }

        void SetEncoderBitrate(int bitrate)
        {
            mEncoderBitrate = bitrate;
            depthAICam->SetEncoderBitrate(mEncoderBitrate);
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
            depthAICam->SetEncoderProfile(mEncoderProfile);
        }

        const std::string & GetEncoderProfile() { return mEncoderProfile; }

        void SetStreamAddress(const std::string address)
        {
            mStreamAddress = address;
            if (mRtspSink) {
                g_object_set(G_OBJECT(mRtspSink),
                    "protocols", 4, // 4 = tcp
                    "location", mStreamAddress.c_str(),
                    NULL);
            }
        }

        DepthAICam *depthAICam;

    protected:
        static void *PlayStream(gpointer data)
        {
            DepthAIGst *depthAIGst = (DepthAIGst *)data;
            DepthAICam * depthAICam = depthAIGst->depthAICam;
            if (depthAICam != nullptr) {
                depthAICam->StartStreaming();
            }
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
 
            DepthAIGst *depthAIGst = (DepthAIGst *)data;
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
                        DepthAIGst::StreamPlayingRestartCallback,
                        depthAIGst,
                        DepthAIGst::StreamPlayingRestartDone);
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

        static void NeedDataCallBack(GstElement *appsrc, guint unused_size, gpointer user_data)
        {
            (void)unused_size;
            GstFlowReturn ret;
            GstBuffer *buffer;
            DepthAIGst *depthAIGst = (DepthAIGst *)user_data;
            auto frame = depthAIGst->depthAICam->GetFrame();

            guint size = frame->getData().size();
            buffer = gst_buffer_new_allocate(NULL, size, NULL);
            gst_buffer_fill(buffer, 0, (gconstpointer)(frame->getData().data()), size);

            GST_BUFFER_PTS(buffer) = depthAIGst->mGstTimestamp;
            GST_BUFFER_DURATION(buffer) = gst_util_uint64_scale_int(1, GST_SECOND, (int)depthAIGst->mEncoderFps);
            depthAIGst->mGstTimestamp += GST_BUFFER_DURATION(buffer);

            g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
            gst_buffer_unref(buffer);
            if (ret != GST_FLOW_OK && ret != GST_FLOW_FLUSHING) {
                g_main_loop_quit(depthAIGst->mLoop);
            }
        }

        static gboolean StreamPlayingRestartCallback(gpointer user_data)
        {
            DepthAIGst *depthAIGst = (DepthAIGst *)user_data;

            g_debug("Restart stream because of connection failed.\n");
            if (depthAIGst->mAppsrc) {
                depthAIGst->mNeedDataSignalId = g_signal_connect(
                    depthAIGst->mAppsrc,"need-data",
                    G_CALLBACK(NeedDataCallBack), depthAIGst);
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
};


class DepthAICamCtrl : public rclcpp::Node
{
    public:
        DepthAICamCtrl(DepthAIGst *depthAIGst)
        : Node("depthai_cam_ctrl"), mDepthAIGst(nullptr)
        {
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "videostreamcmd", rclcpp::SystemDefaultsQoS(), std::bind(&DepthAICamCtrl::depthai_rgb_cam_cmd_cb,
                this, _1));
            if (depthAIGst != nullptr) {
                mDepthAIGst = depthAIGst;
            }
            rcl_interfaces::msg::ParameterDescriptor encoding_desc;
            encoding_desc.name = "encoding";
            encoding_desc.type = rclcpp::PARAMETER_STRING;
            encoding_desc.description = "Encoding format of the video stream.";
            encoding_desc.additional_constraints = "Accepted values are H264 and H265.";
            this->declare_parameter<std::string>("encoding", "H264", encoding_desc);
            this->declare_parameter<int>("width", 1280);
            this->declare_parameter<int>("height", 720);
            this->declare_parameter<int>("fps", 25);
            this->declare_parameter<int>("bitrate", 3000000);
            rcl_interfaces::msg::ParameterDescriptor start_stream_on_boot_desc;
            start_stream_on_boot_desc.name = "start_stream_on_boot";
            start_stream_on_boot_desc.type = rclcpp::PARAMETER_BOOL;
            start_stream_on_boot_desc.description = "The node will start the video stream during " \
                                                    "boot if set to true.";
            start_stream_on_boot_desc.additional_constraints = "This parameter has no " \
                                                            "effect after node has started.";
            this->declare_parameter<bool>("start_stream_on_boot", true, start_stream_on_boot_desc);
            rcl_interfaces::msg::ParameterDescriptor address_desc;
            address_desc.name = "address";
            address_desc.type = rclcpp::PARAMETER_STRING;
            address_desc.description = "Video stream destination address. The Gstreamer pipeline is generated " \
                                    "based on this address.";
            address_desc.additional_constraints = "UDP or RTSP addresses are accepted. For example:\n" \
                                                "\trtsp://<user>:<key>@<ip_address>:<port>/<path> (default)\n" \
                                                "\tudp://<ip_address>:<port>";
            std::string stream_path = "rtsp://DroneUser:22f6c4de-6144-4f6c-82ea-8afcdf19f316@35.187.169.6:8554";
            std::string ns = std::string(this->get_namespace());
            this->declare_parameter<std::string>("address",
                                                stream_path + ns,
                                                address_desc);
            RCLCPP_DEBUG(this->get_logger(), "Namespace: %s", (stream_path + ns).c_str());

            mStreamAddress = this->get_parameter("address").as_string();

            param_cb_handle = rclcpp::Node::add_on_set_parameters_callback(
                std::bind(&DepthAICamCtrl::depthai_set_param_cb, this, _1));

            if (this->get_parameter("start_stream_on_boot").as_bool()) {
                mDepthAIGst->SetEncoderWidth(this->get_parameter("width").as_int());
                mDepthAIGst->SetEncoderHeight(this->get_parameter("height").as_int());
                mDepthAIGst->SetEncoderFps(this->get_parameter("fps").as_int());
                mDepthAIGst->SetEncoderBitrate(this->get_parameter("bitrate").as_int());
                mDepthAIGst->SetEncoderProfile(this->get_parameter("encoding").as_string());
                mDepthAIGst->SetStreamAddress(mStreamAddress);
                RCLCPP_INFO(this->get_logger(), "Start DepthAI GStreamer video stream.");
                depthAIGst->CreatePipeLine();
            }
        }

    private:
        void SetRclCppError(rcl_interfaces::msg::SetParametersResult & res, std::string msg)
        {
            res.successful = false;
            res.reason = msg;
            RCLCPP_ERROR(this->get_logger(), "%s", res.reason.c_str());
        }

        bool IsIpAddressValid(const std::string &ip_address)
        {
            struct sockaddr_in sa;
            int result = inet_pton(AF_INET, ip_address.c_str(), &(sa.sin_addr));

            return result != 0;
        }

        void ValidateAddressParameters(const std::string address, rcl_interfaces::msg::SetParametersResult & res)
        {
            std::string udp_protocol = "udp://";
            std::string rtsp_protocol = "rtsp://";
            std::string protocol = "";
            std::string addr = address;

            if (addr.size() == 0) {
                SetRclCppError(res, "Empty address.");
                return;
            }

            if (addr.find(udp_protocol) == 0) {
                protocol = "udp";
                addr.erase(0, udp_protocol.size());
            } else if (addr.find(rtsp_protocol) == 0) {
                protocol = "rtsp";
                addr.erase(0, rtsp_protocol.size());
            } else {
                SetRclCppError(res, "Not valid protocol in stream address.");
                return;
            }

            std::string ip_addr, port;
            if (protocol == "udp") {
                ip_addr = addr.substr(0, addr.find(":"));
                if (ip_addr.size() == 0 || !IsIpAddressValid(ip_addr)) {
                    SetRclCppError(res, "Not valid IP address.");
                    return;
                }
                port = addr.substr(addr.find(":") + 1);
                if (port.size() == 0) {
                    SetRclCppError(res, "Empty port in address.");
                    return;
                }
            } else if (protocol == "rtsp") {
                std::string user, key, path;
                user = addr.substr(0, addr.find(":"));
                if (user.size() == 0) {
                    SetRclCppError(res, "Empty user in address.");
                    return;
                }
                addr = addr.substr(addr.find(":") + 1);
                key = addr.substr(0, addr.find("@"));
                if (key.size() == 0) {
                    SetRclCppError(res, "Empty key in address.");
                    return;
                }
                addr = addr.substr(addr.find("@") + 1);
                ip_addr = addr.substr(0, addr.find(":"));
                if (ip_addr.size() == 0 || !IsIpAddressValid(ip_addr)) {
                    SetRclCppError(res, "Not valid IP address.");
                    return;
                }
                addr = addr.substr(addr.find(":") + 1);
                port = addr.substr(0, addr.find("/"));
                if (port.size() == 0) {
                    SetRclCppError(res, "Empty port in address.");
                    return;
                }
                path = addr.substr(addr.find("/") + 1);
                if (path.size() == 0) {
                    SetRclCppError(res, "Empty path in address.");
                    return;
                }
            }
        }

        rcl_interfaces::msg::SetParametersResult
        depthai_set_param_cb(const std::vector<rclcpp::Parameter> & parameters)
        {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto & parameter : parameters) {
                if (parameter.get_name() == "encoding") {
                    std::string encoding_val = parameter.as_string();
                    std::transform(encoding_val.begin(), encoding_val.end(), encoding_val.begin(), ::toupper);
                    if ((encoding_val != "H264") && (encoding_val != "H265")) {
                        result.successful = false;
                        result.reason = "Not valid encoding. Allowed H264 and H265.";
                        RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                    }
                }

                if (parameter.get_name() == "width") {
                    int width_val = parameter.as_int();
                    if (width_val % 8 != 0) {
                        result.successful = false;
                        result.reason = "Width must be multiple of 8 for H26x encoder profile.";
                        RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                    }
                    if (width_val > 4096) {
                        result.successful = false;
                        result.reason = "Width must be smaller than 4096 for H26x encoder profile.";
                        RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                    }
                }

                if (parameter.get_name() == "height") {
                    int height_val = parameter.as_int();
                    if (height_val % 8 != 0) {
                        result.successful = false;
                        result.reason = "Heigth must be multiple of 8 for H26x encoder profile.";
                        RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                    }
                    if (height_val > 4096) {
                        result.successful = false;
                        result.reason = "Height must be smaller than 4096 for H26x encoder profile.";
                        RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
                    }
                }

                if (parameter.get_name() == "address") {
                    if (mDepthAIGst->IsStreamPlaying()) {
                        result.successful = false;
                        result.reason = "Cannot change stream address while stream is playing.";
                        return result;
                    }
                    std::string addr = parameter.as_string();
                    ValidateAddressParameters(addr, result);
                    if (!result.successful) {
                        return result;
                    }
                    mStreamAddress = addr;
                    mDepthAIGst->SetStreamAddress(mStreamAddress);
                }
            }
            return result;
        }

        void depthai_rgb_cam_cmd_cb(const std_msgs::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "Command to process: '%s'", msg->data.c_str());
            auto cmd = nlohmann::json::parse(msg->data.c_str());
            if (!cmd["Address"].empty()) {
                if (mDepthAIGst == nullptr) {
                    /* Do nothing if stream already running. */
                    return;
                }
                std::string address = cmd["Address"];
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                ValidateAddressParameters(address, result);
                if (!result.successful) {
                    RCLCPP_ERROR(this->get_logger(), "Error in stream address. Stream cannot be started.");
                    return;
                }
                mStreamAddress = address;
            }
            if (!cmd["Command"].empty()) {
                if (mDepthAIGst == nullptr) {
                    /* Do nothing if stream already running. */
                    return;
                }
                std::string command = cmd["Command"];
                std::transform(command.begin(), command.end(),
                    command.begin(), [](unsigned char c){ return std::tolower(c); });
                if (command == "start") {
                    if (!mDepthAIGst->IsStreamPlaying()) {
                        mDepthAIGst->SetEncoderWidth(this->get_parameter("width").as_int());
                        mDepthAIGst->SetEncoderHeight(this->get_parameter("height").as_int());
                        mDepthAIGst->SetEncoderFps(this->get_parameter("fps").as_int());
                        mDepthAIGst->SetEncoderProfile(this->get_parameter("encoding").as_string());
                        mDepthAIGst->SetStreamAddress(mStreamAddress);
                        RCLCPP_INFO(this->get_logger(), "Start DepthAI camera streaming.");
                        mDepthAIGst->CreatePipeLine();
                        return;
                    }
                    RCLCPP_INFO(this->get_logger(), "DepthAI camera already streaming.");
                } else if (command == "stop") {
                    RCLCPP_INFO(this->get_logger(), "Stop DepthAI camera streaming.");
                }
            }
        }

        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        DepthAIGst *mDepthAIGst;
        OnSetParametersCallbackHandle::SharedPtr param_cb_handle;
        std::string mStreamAddress;
};


int main(int argc, char * argv[])
{
    std::cout << "Init DepthAI Gstreamer pipeline." << std::endl;
    DepthAIGst depthAIGst(argc, argv);

    rclcpp::init(argc, argv);

    std::cout << "Start ROS2 DepthAI subscriber." << std::endl;
    rclcpp::spin(std::make_shared<DepthAICamCtrl>(&depthAIGst));

    std::cout << "Stop ROS2 DepthAI subscriber." << std::endl;
    depthAIGst.StopStream();
    rclcpp::shutdown();

    return 0;
}
