
#ifndef FOG_SW_DEPTHAI_GSTREAMER_H
#define FOG_SW_DEPTHAI_GSTREAMER_H

#include <gst/gst.h>
#include <gst/gstbus.h>
#include <gst/gstcaps.h>
#include <gst/gstelement.h>
#include <gst/gstpipeline.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <queue>
#include <mutex>
#include <arpa/inet.h>
#include <nlohmann/json.hpp>

namespace depthai_ctrl
{

/// ROS2 Node, taking video stream coming from ROS2 messages and transmitting it to the specified RSTP/UDP address
class DepthAiGStreamer : public rclcpp::Node
{
  public:
    using CompressedImageMsg = sensor_msgs::msg::CompressedImage;

    DepthAiGStreamer(int argc, char* argv[]);
    DepthAiGStreamer(const rclcpp::NodeOptions & options);

    void GrabVideoMsg(const CompressedImageMsg::SharedPtr video_msg);

    ~DepthAiGStreamer();

    void DestroyPipeline();

    void BuildDefaultPipeline();

    void CreatePipeline();

//    void StopStream(void);

    bool IsStreamPlaying() { return mIsStreamPlaying; }

  protected:

    static void* gst_PlayStream(gpointer data);

    static gboolean gst_MissingPluginMessage(GstMessage* msg);

    static const gchar* gst_missing_plugin_message_get_description(GstMessage* msg)
    {
        return gst_structure_get_string(gst_message_get_structure(msg), "name");
    }

    static gboolean gst_StreamEventCallBack(GstBus* bus, GstMessage* message, gpointer data);

    static void gst_NeedDataCallBack(GstElement* appsrc, guint unused_size, gpointer user_data);

    static gboolean gst_StreamRestartCallback(gpointer user_data);

    // Use this function to execute code when timer is removed.
    static void StreamPlayingRestartDone(gpointer user_data) { (void)user_data; }

  private:

    void Initialize();

    bool IsVideoStreamAvailable()
    {
        return _video_subscriber->get_publisher_count() > 0 && !(_message_queue.empty());
    }

    std::string ReadIpAddresFromUdpAddress(void);

    int ReadPortFromUdpAddress(void);

    std::shared_ptr<rclcpp::Subscription<CompressedImageMsg>> _video_subscriber;
    std::queue<CompressedImageMsg::SharedPtr> _message_queue;
    std::mutex _message_queue_mutex;

    bool mIsStreamPlaying;
    int mEncoderWidth;
    int mEncoderHeight;
    int mEncoderFps;
    int mEncoderBitrate;
    std::string mEncoderProfile;
    std::string mStreamAddress;

    GMainLoop* mLoop;
    GstElement* mPipeline;
    GstElement* mAppsrc;
    GstElement* mH26xparse;
    GstElement* mH26xpay;
    GstElement* mUdpSink;
    guint mBusWatchId;
    GstBus* mBus;
    guint mNeedDataSignalId;
    GThread* mLoopThread;
    GstElement* mQueue1;
    GstElement* mRtspSink;

    GstClockTime mGstTimestamp;
    GstElement* mTestSrc;
    GstElement* mTextOverlay;
    GstElement* mH26xEnc;
    GstElement* mTestSrcFilter;
    GstElement* mH26xEncFilter;
    guint mStreamPlayingCheckTimerId;
    GMainContext* mLoopContext;


    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _video_stream_command_subscriber;
    OnSetParametersCallbackHandle::SharedPtr _parameter_setter;


    void SetRclCppError(rcl_interfaces::msg::SetParametersResult& res, std::string msg)
    {
        res.successful = false;
        res.reason = msg;
        RCLCPP_ERROR(this->get_logger(), "%s", res.reason.c_str());
    }

    bool IsIpAddressValid(const std::string& ip_address)
    {
        struct sockaddr_in sa;
        int result = inet_pton(AF_INET, ip_address.c_str(), &(sa.sin_addr));

        return result != 0;
    }

    void ValidateAddressParameters(const std::string address, rcl_interfaces::msg::SetParametersResult& res);

    rcl_interfaces::msg::SetParametersResult SetParameters(const std::vector<rclcpp::Parameter>& parameters);

    void VideoStreamCommand(const std_msgs::msg::String::SharedPtr msg);

};

} // namespace depthai_ctrl

#endif  // FOG_SW_DEPTHAI_GSTREAMER_H
