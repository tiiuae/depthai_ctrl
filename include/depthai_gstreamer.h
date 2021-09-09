
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

//    void DestroyPipeline();
//
//    void BuildDefaultPipeline();
//
//    void CreatePipeline();

//    void StopStream();

//    bool IsStreamPlaying();

  protected:



  private:

    rclcpp::Subscription<CompressedImageMsg>::SharedPtr _video_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _stream_command_subscriber;
    OnSetParametersCallbackHandle::SharedPtr _parameter_setter;

    struct Impl;
    std::unique_ptr<Impl> _impl;


    void Initialize();

    void GrabVideoMsg(CompressedImageMsg::SharedPtr video_msg);

    //bool IsVideoStreamAvailable();


    void SetRclCppError(rcl_interfaces::msg::SetParametersResult& res, std::string msg)
    {
        res.successful = false;
        res.reason = msg;
        RCLCPP_ERROR(this->get_logger(), "%s", res.reason.c_str());
    }

    bool IsIpAddressValid(const std::string& ip_address);

    void ValidateAddressParameters(const std::string address, rcl_interfaces::msg::SetParametersResult& res);

    rcl_interfaces::msg::SetParametersResult SetParameters(const std::vector<rclcpp::Parameter>& parameters);

    void VideoStreamCommand(const std_msgs::msg::String::SharedPtr msg);

};

} // namespace depthai_ctrl

#endif  // FOG_SW_DEPTHAI_GSTREAMER_H
