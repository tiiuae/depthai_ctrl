
#ifndef FOG_SW_DEPTHAI_GSTREAMER_H
#define FOG_SW_DEPTHAI_GSTREAMER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>

namespace depthai_ctrl
{

/// ROS2 Node, taking video stream coming from ROS2 messages and transmitting it to the specified RSTP/UDP address
class DepthAIGStreamer : public rclcpp::Node
{
  public:
    using CompressedImageMsg = sensor_msgs::msg::CompressedImage;

    DepthAIGStreamer(int argc, char* argv[]);
    DepthAIGStreamer(const rclcpp::NodeOptions & options);
    ~DepthAIGStreamer();

    bool IsStreamPlaying();
    bool IsStreamDefault();
    bool IsErrorDetected();

  private:

    struct Impl;
    std::unique_ptr<Impl> _impl;
    rclcpp::Subscription<CompressedImageMsg>::SharedPtr _video_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _stream_command_subscriber;

    void Initialize();
    void GrabVideoMsg(CompressedImageMsg::SharedPtr video_msg);
    void VideoStreamCommand(const std_msgs::msg::String::SharedPtr msg);

};

} // namespace depthai_ctrl

#endif  // FOG_SW_DEPTHAI_GSTREAMER_H
