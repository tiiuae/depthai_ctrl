#include <memory>
#include <thread>
#include <iostream>
#include <chrono>
#include <stdlib.h>
#include <algorithm>
#include <string>
#include <cctype>

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
#define DEPTHAI_CTRL_VER_MINOR 4
// The CI build will add a build number after the minor version.

#define DEPTHAI_CTRL_VERSION (DEPTHAI_CTRL_VER_MAJOR * 100 + DEPTHAI_CTRL_VER_MINOR)


class DepthAICam
{
    public:
        DepthAICam() : device(nullptr)
        {
            colorCam = mPipeline.create<dai::node::ColorCamera>();
            colorCamVideoEnc = mPipeline.create<dai::node::VideoEncoder>();
            colorCamXLinkOut = mPipeline.create<dai::node::XLinkOut>();

            colorCamXLinkOut->setStreamName("enc264Color");
            colorCam->setBoardSocket(dai::CameraBoardSocket::RGB);
            colorCam->setVideoSize(1280, 720);
            colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);

            colorCamVideoEnc->setDefaultProfilePreset(1280, 720, 25, dai::VideoEncoderProperties::Profile::H264_MAIN);
            colorCam->video.link(colorCamVideoEnc->input);
            colorCamVideoEnc->bitstream.link(colorCamXLinkOut->input);

            colorCamXLinkIn = mPipeline.create<dai::node::XLinkIn>();
            colorCamXLinkIn->setStreamName("colorCamCtrl");

            colorCamXLinkIn->out.link(colorCam->inputControl);

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
            if (mIsDeviceAvailable) {
                device->startPipeline();

                encColorOutput = device->getOutputQueue("enc264Color", 30, true);
                colorCamInput = device->getInputQueue("colorCamCtrl");

                dai::CameraControl colorCamCtrl;
                colorCamCtrl.setAutoFocusMode(dai::RawCameraControl::AutoFocusMode::AUTO);
                //colorCamCtrl.setManualFocus(255);
                colorCamInput->send(colorCamCtrl);
            }
        }

        std::shared_ptr<dai::ImgFrame> GetFrame(void)
        {
            return encColorOutput->get<dai::ImgFrame>();
        }

        bool IsDeviceAvailable(void)
        {
            return mIsDeviceAvailable;
        }

    private:
        dai::Device *device;
        dai::Pipeline mPipeline;
        std::shared_ptr<dai::DataOutputQueue> encColorOutput;
        std::shared_ptr<dai::node::ColorCamera> colorCam;
        std::shared_ptr<dai::node::VideoEncoder> colorCamVideoEnc;
        std::shared_ptr<dai::node::XLinkOut> colorCamXLinkOut;
        std::shared_ptr<dai::node::XLinkIn> colorCamXLinkIn;
        std::shared_ptr<dai::DataInputQueue> colorCamInput;
        bool mIsDeviceAvailable = true;
};


class DepthAIGst
{
    public:
        DepthAIGst(int argc, char *argv[]) : depthAICam(nullptr), mLoop(nullptr), mPipeline(nullptr), mAppsrc(nullptr),
                                            mH264parse(nullptr), mH264pay(nullptr), mUdpSink(nullptr), mBusWatchId(0),
                                            mBus(nullptr), mNeedDataSignalId(0), mLoopThread(nullptr), mQueue1(nullptr),
                                            mIsStreamPlaying(false)
        {
            gst_init(&argc, &argv);
            mLoop = g_main_loop_new(NULL, false);
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

        void DestroyPipeline(void) {
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
            }
        }

        void CreatePipeLine(void) {
            if (!depthAICam->IsDeviceAvailable()) {
                g_printerr("Warning: device not available. Cannot start stream.\n");
                return;
            }
            mPipeline = gst_pipeline_new("camUDPSink_pipeline");
            mAppsrc = gst_element_factory_make("appsrc", "source");
            g_object_set(G_OBJECT(mAppsrc), "do-timestamp", true, NULL);
            g_object_set(G_OBJECT(mAppsrc), "is-live", true, NULL);
            g_object_set(G_OBJECT(mAppsrc), "block", true, NULL);
            g_object_set(G_OBJECT(mAppsrc), "stream-type", 0, NULL);
            gst_util_set_object_arg(G_OBJECT(mAppsrc), "format", "GST_FORMAT_TIME");
            mH264parse = gst_element_factory_make("h264parse", "parser");
            mQueue1 = gst_element_factory_make("queue", "queue1");
            mH264pay = gst_element_factory_make("rtph264pay", "payload");
            g_object_set(G_OBJECT(mH264pay), "pt", 96, NULL);
            mUdpSink = gst_element_factory_make("udpsink", "udp_sink");
            g_object_set(G_OBJECT(mUdpSink), "host", "127.0.0.1", NULL);
            g_object_set(G_OBJECT(mUdpSink), "port", 5600, NULL);
            g_object_set(G_OBJECT(mAppsrc), "caps",
                gst_caps_new_simple("video/x-h264",
                    "width", G_TYPE_INT, 1280,
                    "height", G_TYPE_INT, 720,
                    "framerate", GST_TYPE_FRACTION, 25, 1,
                    NULL), NULL);

            mBus = gst_pipeline_get_bus(GST_PIPELINE(mPipeline));
            mBusWatchId = gst_bus_add_watch(mBus, StreamEventCallBack, this);
            gst_object_unref(mBus);
            mBus = nullptr;
            
            gst_bin_add_many(GST_BIN(mPipeline), mAppsrc, mH264parse, mQueue1, mH264pay, mUdpSink, NULL);
            gst_element_link_many(mAppsrc, mH264parse, mQueue1, mH264pay, mUdpSink, NULL);

            mLoopThread = g_thread_new("GstThread", (GThreadFunc)DepthAIGst::PlayStream, this);

            mNeedDataSignalId = g_signal_connect(mAppsrc, "need-data", G_CALLBACK(NeedDataCallBack), this);
            mIsStreamPlaying = true;
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
            g_thread_exit(0);

            return nullptr;
        }

        static gboolean StreamEventCallBack(GstBus *bus, GstMessage *message, gpointer data)
        {
            (void)bus;
            g_print("%s: Got %s message\n", __FUNCTION__, GST_MESSAGE_TYPE_NAME(message));
 
            DepthAIGst *depthAIGst = (DepthAIGst *)data;
            GstTagList *list = nullptr;

            switch (GST_MESSAGE_TYPE (message)) {
            case GST_MESSAGE_EOS:
                g_print("End of stream\n");
                g_main_loop_quit(depthAIGst->mLoop);
                break;

            case GST_MESSAGE_TAG:
                list = gst_tag_list_new_empty();

                gst_message_parse_tag(message, &list);

                g_printerr("Tag: %s\n", gst_tag_list_to_string(list));
                gst_tag_list_unref(list);
                break;

            case GST_MESSAGE_WARNING:
                gchar  *warnDebug;
                GError *warning;

                gst_message_parse_warning(message, &warning, &warnDebug);
                g_free(warnDebug);

                g_printerr("Warning: %s\n", warning->message);
                g_error_free(warning);
                break;

            case GST_MESSAGE_ERROR:
                gchar  *errDebug;
                GError *error;

                gst_message_parse_error(message, &error, &errDebug);
                g_free(errDebug);

                g_printerr("Error: %s\n", error->message);
                g_error_free(error);

                g_main_loop_quit(depthAIGst->mLoop);
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

            g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
            gst_buffer_unref(buffer);
            if (ret != GST_FLOW_OK) {
                /* something wrong, stop pushing */
                g_main_loop_quit(depthAIGst->mLoop);
            }
        }

    private:
        GMainLoop *mLoop;
        GstElement *mPipeline;
        GstElement *mAppsrc;
        GstElement *mH264parse;
        GstElement *mH264pay;
        GstElement *mUdpSink;
        guint mBusWatchId;
        GstBus *mBus;
        guint mNeedDataSignalId;
        GThread *mLoopThread;
        GstElement *mQueue1;
        bool mIsStreamPlaying;
};


class DepthAICamCtrlSub : public rclcpp::Node
{
    public:
        DepthAICamCtrlSub(DepthAIGst *depthAIGst)
        : Node("depthai_cam_ctrl"), mDepthAIGst(nullptr)
        {
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "videostreamcmd", 10, std::bind(&DepthAICamCtrlSub::depthai_rgb_cam_cmd_cb,
                this, _1));
            if (depthAIGst != nullptr) {
                mDepthAIGst = depthAIGst;
            }
            rcl_interfaces::msg::ParameterDescriptor encoding_desc;
            encoding_desc.name = "encoding";
            encoding_desc.type = rclcpp::PARAMETER_STRING;
            encoding_desc.description = "Encoding format of the video stream.";
            encoding_desc.additional_constraints = "Accepted values are h264 and h265.";
            this->declare_parameter<std::string>("encoding", "h264", encoding_desc);
            this->declare_parameter<int>("width", 1280);
            this->declare_parameter<int>("height", 720);
            this->declare_parameter<int>("fps", 25);
            this->declare_parameter<int>("bitrate", 30000);
            rcl_interfaces::msg::ParameterDescriptor start_stream_on_boot_desc;
            start_stream_on_boot_desc.name = "start_stream_on_boot";
            start_stream_on_boot_desc.type = rclcpp::PARAMETER_BOOL;
            start_stream_on_boot_desc.description = "The node will start the video stream during " \
                                                    "boot if set to true.";
            start_stream_on_boot_desc.additional_constraints = "This parameter has no " \
                                                    "effect after node has started.";
            this->declare_parameter<bool>("start_stream_on_boot", false, start_stream_on_boot_desc);
        }

    private:
        void depthai_rgb_cam_cmd_cb(const std_msgs::msg::String::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), "Command to process: '%s'", msg->data.c_str());
            auto cmd = nlohmann::json::parse(msg->data.c_str());
            if (!cmd["Encoding"].empty()) {
                std::string encoding = cmd["Encoding"];
                std::transform(encoding.begin(), encoding.end(),
                    encoding.begin(), [](unsigned char c){ return std::tolower(c); });
                if (encoding == "H265") {
                    /* H265 */
                } else {
                    /* H264 is default */
                }
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
};


int main(int argc, char * argv[])
{
    std::cout << "Init DepthAI Gstreamer pipeline." << std::endl;
    DepthAIGst depthAIGst(argc, argv);

    rclcpp::init(argc, argv);
    
    std::cout << "Start DepthAI GStreamer video stream." << std::endl;
    depthAIGst.CreatePipeLine();

    std::cout << "Start ROS2 DepthAI subscriber." << std::endl;
    rclcpp::spin(std::make_shared<DepthAICamCtrlSub>(&depthAIGst));

    std::cout << "Stop ROS2 DepthAI subscriber." << std::endl;
    depthAIGst.StopStream();
    rclcpp::shutdown();

    return 0;
}
