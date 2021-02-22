#include <memory>
#include <thread>
#include <iostream>
#include <chrono>
#include <stdlib.h>

#include <depthai/depthai.hpp>

#include <gst/gst.h>
#include <gst/gstbus.h>
#include <gst/gstpipeline.h>
#include <gst/gstelement.h>
#include <gst/gstcaps.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;


class DepthAICtrlSub : public rclcpp::Node
{
    public:
        DepthAICtrlSub()
        : Node("depthai_ctrl_sub")
        {
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "depthai_cam_cmd", 10, std::bind(&DepthAICtrlSub::depthai_rgb_cam_cmd_cb,
                this, _1));
        }

    private:
        void depthai_rgb_cam_cmd_cb(const std_msgs::msg::String::SharedPtr msg) const
        {
            RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

class DepthAICam
{
    public:
        DepthAICam() : device(nullptr)
        {
            auto colorCam = mPipeline.create<dai::node::ColorCamera>();
            auto xlinkOut = mPipeline.create<dai::node::XLinkOut>();
            auto videoEnc = mPipeline.create<dai::node::VideoEncoder>();
            auto videoEncXLinkOut = mPipeline.create<dai::node::XLinkOut>();
            auto monoCam1 = mPipeline.create<dai::node::MonoCamera>();

            xlinkOut->setStreamName("preview");
            colorCam->setPreviewSize(1280, 720);
            colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
            colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
            colorCam->setInterleaved(true);
            colorCam->preview.link(xlinkOut->input);

            videoEncXLinkOut->setStreamName("enc264Left");
            monoCam1->setBoardSocket(dai::CameraBoardSocket::LEFT);

            videoEnc->setDefaultProfilePreset(1280, 720, 25, dai::VideoEncoderProperties::Profile::H264_MAIN);
            monoCam1->out.link(videoEnc->input);
            videoEnc->bitstream.link(videoEncXLinkOut->input);

            try {
                device = new dai::Device(mPipeline, false);
                previewOutput = device->getOutputQueue("preview");
                encLeftOutput = device->getOutputQueue("enc264Left", 30, true);
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
            }
        }

        std::shared_ptr<dai::ImgFrame> GetFrame(void)
        {
            return previewOutput->get<dai::ImgFrame>();
        }

        bool IsDeviceAvailable(void)
        {
            return mIsDeviceAvailable;
        }

        std::shared_ptr<dai::DataOutputQueue> encLeftOutput;

    private:
        dai::Device *device;
        std::shared_ptr<dai::DataOutputQueue> previewOutput;
        dai::Pipeline mPipeline;
        dai::CameraControl mCameraCtrl;
        bool mIsDeviceAvailable = true;
};


class DepthAIGst
{
    public:
        DepthAIGst(int argc, char *argv[]) : depthAICam(nullptr), mLoop(nullptr), mPipeline(nullptr), mAppsrc(nullptr),
                                            mVideoconvert(nullptr), mH264enc(nullptr), mH264pay(nullptr),
                                            mUdpSink(nullptr), mBus(nullptr), mEncQueue(nullptr), mNeedDataSignalId(0),
                                            mPipeline2(nullptr), mAppsrc2(nullptr), mH264parse2(nullptr), mH264pay2(nullptr),
                                            mUdpSink2(nullptr), mBus2(nullptr), mEncQueue2(nullptr), mNeedDataSignalId2(0),
                                            mEnoughDataSignalId2(0)
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
            g_source_remove(mBusWatchId);
        }

        void CreatePipeLine(void) {
            mPipeline = gst_pipeline_new("camUDPSink_pipeline");
            mAppsrc = gst_element_factory_make("appsrc", "source");
            g_object_set(G_OBJECT(mAppsrc), "do-timestamp", true, NULL);
            g_object_set(G_OBJECT(mAppsrc), "is-live", true, NULL);
            g_object_set(G_OBJECT(mAppsrc), "block", true, NULL);
            g_object_set(G_OBJECT(mAppsrc), "stream-type", 0, NULL);
            gst_util_set_object_arg(G_OBJECT(mAppsrc), "format", "GST_FORMAT_TIME");
            mVideoconvert = gst_element_factory_make("videoconvert", "video_convert");
            mEncQueue = gst_element_factory_make("queue", "encoder_queue");
            mH264enc = gst_element_factory_make("x264enc", "encoder");
            gst_util_set_object_arg(G_OBJECT(mH264enc), "tune", "zerolatency");
            gst_util_set_object_arg(G_OBJECT(mH264enc), "speed-preset", "ultrafast");
            mH264pay = gst_element_factory_make("rtph264pay", "payload");
            g_object_set(G_OBJECT(mH264pay), "pt", 96, NULL);
            mUdpSink = gst_element_factory_make("udpsink", "udp_sink");
            g_object_set(G_OBJECT(mUdpSink), "host", "127.0.0.1", NULL);
            g_object_set(G_OBJECT(mUdpSink), "port", 5600, NULL);
            g_object_set(G_OBJECT(mAppsrc), "caps",
                gst_caps_new_simple("video/x-raw",
                    "format", G_TYPE_STRING, "BGR",
                    "width", G_TYPE_INT, 1280,
                    "height", G_TYPE_INT, 720,
                    "framerate", GST_TYPE_FRACTION, 30, 1,
                    NULL), NULL);

            mBus = gst_pipeline_get_bus(GST_PIPELINE(mPipeline));
            mBusWatchId = gst_bus_add_watch(mBus, StreamEventCallBack, this);
            gst_object_unref(mBus);
            mBus = nullptr;
            
            gst_bin_add_many(GST_BIN(mPipeline), mAppsrc, mVideoconvert, mEncQueue, mH264enc, mH264pay, mUdpSink, NULL);
            gst_element_link_many (mAppsrc, mVideoconvert, mEncQueue, mH264enc, mH264pay, mUdpSink, NULL);

            /* Second pipeline for encoded h264 video of the left camera. */
            mPipeline2 = gst_pipeline_new("camUDPSink_pipeline2");
            mAppsrc2 = gst_element_factory_make("appsrc", "source2");
            g_object_set(G_OBJECT(mAppsrc2), "do-timestamp", true, NULL);
            g_object_set(G_OBJECT(mAppsrc2), "is-live", true, NULL);
            g_object_set(G_OBJECT(mAppsrc2), "block", true, NULL);
            g_object_set(G_OBJECT(mAppsrc2), "stream-type", 0, NULL);
            gst_util_set_object_arg(G_OBJECT(mAppsrc2), "format", "GST_FORMAT_TIME");
            mH264parse2 = gst_element_factory_make("h264parse", "parser2");
            mEncQueue2 = gst_element_factory_make("queue", "encoder_queue2");
            mH264pay2 = gst_element_factory_make("rtph264pay", "payload2");
            g_object_set(G_OBJECT(mH264pay2), "pt", 96, NULL);
            mUdpSink2 = gst_element_factory_make("udpsink", "udp_sink2");
            g_object_set(G_OBJECT(mUdpSink2), "host", "127.0.0.1", NULL);
            g_object_set(G_OBJECT(mUdpSink2), "port", 5601, NULL);
            g_object_set(G_OBJECT(mAppsrc2), "caps",
                gst_caps_new_simple("video/x-h264",
                    "width", G_TYPE_INT, 1280,
                    "height", G_TYPE_INT, 720,
                    "framerate", GST_TYPE_FRACTION, 25, 1,
                    NULL), NULL);

            mBus2 = gst_pipeline_get_bus(GST_PIPELINE(mPipeline2));
            mBusWatchId2 = gst_bus_add_watch(mBus2, StreamEventCallBack2, this);
            gst_object_unref(mBus2);
            mBus2 = nullptr;
            
            gst_bin_add_many(GST_BIN(mPipeline2), mAppsrc2, mH264parse2, mH264pay2, mUdpSink2, NULL);
            gst_element_link_many (mAppsrc2, mH264parse2, mH264pay2, mUdpSink2, NULL);

            mLoopThread = g_thread_new("GstThread", (GThreadFunc)DepthAIGst::PlayStream, this);

            if (!depthAICam->IsDeviceAvailable()) {
                return;
            }

            mNeedDataSignalId = g_signal_connect(mAppsrc, "need-data", G_CALLBACK(NeedDataCallBack), this);
            mNeedDataSignalId2 = g_signal_connect(mAppsrc2, "need-data", G_CALLBACK(NeedDataCallBack2), this);
            mEnoughDataSignalId2 = g_signal_connect(mAppsrc2, "enough-data", G_CALLBACK(EnoughDataCallBack2), this);
        }

        void StopStream(void)
        {
            GstFlowReturn ret, ret2;
            if (mEnoughDataSignalId2 != 0) {
                g_signal_handler_disconnect(mAppsrc2, mEnoughDataSignalId2);
            }
            if (mNeedDataSignalId2 != 0) {
                g_signal_handler_disconnect(mAppsrc2, mNeedDataSignalId2);
            }
            g_signal_emit_by_name(mAppsrc2, "end-of-stream", &ret2);
            if (ret2 != GST_FLOW_OK) {
                g_printerr("Error: Emit end-of-stream failed\n");
            }
            gst_element_set_state(mPipeline2, GST_STATE_NULL);
            
            if (mNeedDataSignalId != 0) {
                g_signal_handler_disconnect(mAppsrc, mNeedDataSignalId);
            }
            g_signal_emit_by_name(mAppsrc, "end-of-stream", &ret);
            if (ret != GST_FLOW_OK) {
                g_printerr("Error: Emit end-of-stream failed\n");
            }
            gst_element_set_state(mPipeline, GST_STATE_NULL);
            g_main_loop_quit(mLoop);
            g_thread_join(mLoopThread);
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
            gst_element_set_state(depthAIGst->mPipeline2, GST_STATE_PLAYING);
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

            switch (GST_MESSAGE_TYPE (message)) {
            case GST_MESSAGE_EOS:
                g_print("End of stream\n");
                g_main_loop_quit(depthAIGst->mLoop);
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
            std::shared_ptr<dai::ImgFrame> frame;
            
            try {
                frame = depthAIGst->depthAICam->GetFrame();
            } catch (const std::runtime_error& err) {
                std::cout << "DepthAICam::GetFrame runtime error: " << err.what() << std::endl;
            }

            if (frame == nullptr) {
                return;
            }

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

        static gboolean StreamEventCallBack2(GstBus *bus, GstMessage *message, gpointer data)
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

        static void NeedDataCallBack2(GstElement *appsrc, guint unused_size, gpointer user_data)
        {
            (void)unused_size;
            GstFlowReturn ret;
            GstBuffer *buffer;
            DepthAIGst *depthAIGst = (DepthAIGst *)user_data;
            DepthAICam *depthAICam = depthAIGst->depthAICam;
            
            auto packet = depthAICam->encLeftOutput->get<dai::Buffer>();

            guint size = packet->getData().size();
            buffer = gst_buffer_new_allocate(NULL, size, NULL);
            gst_buffer_fill(buffer, 0, (gconstpointer)(packet->getData().data()), size);

            g_signal_emit_by_name(appsrc, "push-buffer", buffer, &ret);
            gst_buffer_unref(buffer);
            if (ret != GST_FLOW_OK) {
                /* something wrong, stop pushing */
                g_main_loop_quit(depthAIGst->mLoop);
            }
        }

        static void EnoughDataCallBack2(GstElement *appsrc, guint unused_size, gpointer user_data)
        {
            (void)appsrc;
            (void)unused_size;
            (void)user_data;
            g_print("%s\n", __FUNCTION__);
        }

        GThread *mLoopThread;

    private:
        GMainLoop *mLoop;
        GstElement *mPipeline;
        GstElement *mAppsrc;
        GstElement *mVideoconvert;
        GstElement *mH264enc;
        GstElement *mH264pay;
        GstElement *mUdpSink;
        guint mBusWatchId;
        GstBus *mBus;
        GstElement *mEncQueue;
        guint mNeedDataSignalId;
        GstElement *mPipeline2;
        GstElement *mAppsrc2;
        GstElement *mH264parse2;
        GstElement *mH264pay2;
        GstElement *mUdpSink2;
        guint mBusWatchId2;
        GstBus *mBus2;
        GstElement *mEncQueue2;
        guint mNeedDataSignalId2;
        guint mEnoughDataSignalId2;
};


int main(int argc, char * argv[])
{
    std::cout << "Init DepthAI Gstreamer pipeline." << std::endl;
    DepthAIGst depthAIGst(argc, argv);

    depthAIGst.CreatePipeLine();

    rclcpp::init(argc, argv);
    
    std::cout << "Start ROS2 DepthAI subscriber." << std::endl;
    rclcpp::spin(std::make_shared<DepthAICtrlSub>());

    std::cout << "Stop ROS2 DepthAI subscriber." << std::endl;
    depthAIGst.StopStream();
    rclcpp::shutdown();

    return 0;
}
