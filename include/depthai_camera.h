#ifndef FOG_SW_DEPTHAI_CAMERA_H
#define FOG_SW_DEPTHAI_CAMERA_H
#include <depthai/device/Device.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/node/ColorCamera.hpp>
#include <depthai/pipeline/node/MonoCamera.hpp>
#include <depthai/pipeline/node/VideoEncoder.hpp>
#include <depthai/pipeline/node/XLinkIn.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>
#include <depthai/utility/Initialization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>

namespace depthai_ctrl
{

/// ROS2 Node which works as DepthAI Camera driver
/// Reads camera frames directly fromn the camera and publishes corresponding ROS2 messages
/// It also publishes color camera video stream
class DepthAICamera : public rclcpp::Node
{
  public:
    using ImageMsg = sensor_msgs::msg::Image;
    using CompressedImageMsg = sensor_msgs::msg::CompressedImage;

    DepthAICamera()
        : Node("depthai_camera"),
          _videoWidth(1280),
          _videoHeight(720),
          _videoFps(25),
          _videoBitrate(3000000),
          _videoH265(false),
          _thread_running(false),
          _left_camera_frame("left_camera_frame"),
          _right_camera_frame("right_camera_frame"),
          _color_camera_frame("color_camera_frame")
    {
        Initialize();
        TryRestarting();
    }

    DepthAICamera(const rclcpp::NodeOptions& options) : Node("depthai_camera", options),
                                                _videoWidth(1280),
                                                _videoHeight(720),
                                                _videoFps(25),
                                                _videoBitrate(3000000),
                                                _videoH265(false),
                                                _thread_running(false),
                                                _left_camera_frame("left_camera_frame"),
                                                _right_camera_frame("right_camera_frame"),
                                                _color_camera_frame("color_camera_frame")
    {

        Initialize();
        TryRestarting();
    }

    ~DepthAICamera()
    {
        Stop();
    };

    bool IsNodeRunning() { return bool(_device) && !_device->isClosed() && _thread_running; }

    void Stop()
    {
        if (_thread_running)
        {
            _thread_running = false;
            if(_processing_thread.joinable())
            {
                _processing_thread.join();
            }

        }
        if (bool(_device))
        {
            _device->close();
        }
    }

    void TryRestarting();


    bool ValidateParameters(int width, int height, int fps, int bitrate, std::string encoding)
    {
        if (width > 3840 || width % 8 != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "[%s]: Required video stream parameter 'width' is incorrect.", this->get_name());
            return false;
        }
        if (height > 2160 || height % 8 != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "[%s]: Required video stream parameter 'height' is incorrect.", this->get_name());
            return false;
        }

        if (fps < 5 || fps > 60)
        {
            RCLCPP_ERROR(this->get_logger(), "[%s]: Required video stream parameter 'height' is incorrect.", this->get_name());
            return false;
        }

        if (bitrate < 400000)
        {
            RCLCPP_ERROR(this->get_logger(), "[%s]: Required video stream parameter 'bitrate' is incorrect.", this->get_name());
            return false;
        }

        std::transform(encoding.begin(), encoding.end(), encoding.begin(), ::toupper);
        if ((encoding != "H264") && (encoding != "H265"))
        {
            RCLCPP_ERROR(this->get_logger(), "[%s]: Required video stream parameter 'encoding' is incorrect.", this->get_name());
            return false;
        }
        return true;
    }


  private:
    void ProcessingThread();
    std::shared_ptr<ImageMsg> ConvertImage(std::shared_ptr<dai::ImgFrame>, std::string);
    void Initialize();
    void VideoStreamCommand(std_msgs::msg::String::SharedPtr);
    rcl_interfaces::msg::SetParametersResult SetParameters(const std::vector<rclcpp::Parameter>&);

    std::shared_ptr<dai::Device> _device;
    std::shared_ptr<dai::Pipeline> _pipeline;
    std::shared_ptr<dai::DataOutputQueue> _videoQueue;
    std::shared_ptr<dai::DataOutputQueue> _leftQueue;
    std::shared_ptr<dai::DataOutputQueue> _rightQueue;
    std::shared_ptr<dai::DataOutputQueue> _colorQueue;

    int _videoWidth;
    int _videoHeight;
    int _videoFps;
    int _videoBitrate;
    bool _videoH265;

    std::shared_ptr<rclcpp::Publisher<ImageMsg>> _left_publisher;
    std::shared_ptr<rclcpp::Publisher<ImageMsg>> _right_publisher;
    std::shared_ptr<rclcpp::Publisher<ImageMsg>> _color_publisher;
    std::shared_ptr<rclcpp::Publisher<CompressedImageMsg>> _video_publisher;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _video_stream_command_subscriber;
    OnSetParametersCallbackHandle::SharedPtr _parameters_setter;

    std::thread _processing_thread;
    std::atomic<bool> _thread_running;
    std::string _left_camera_frame, _right_camera_frame, _color_camera_frame;

    std::unordered_map<dai::RawImgFrame::Type, std::string> encodingEnumMap = {
        {dai::RawImgFrame::Type::YUV422i, "yuv422"},
        {dai::RawImgFrame::Type::RGBA8888, "rgba8"},
        {dai::RawImgFrame::Type::RGB888i, "rgb8"},
        {dai::RawImgFrame::Type::BGR888i, "bgr8"},
        {dai::RawImgFrame::Type::GRAY8, "mono8"},
        {dai::RawImgFrame::Type::RAW8, "8UC1"},
        {dai::RawImgFrame::Type::RAW16, "16UC1"}};

};

}  // namespace depthai_ctrl

#endif  // FOG_SW_DEPTHAI_CAMERA_H
