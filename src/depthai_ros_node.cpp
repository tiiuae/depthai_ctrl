#include "ImageConverter.hpp"
#include <depthai/device/Device.hpp>
#include <depthai/pipeline/node/ColorCamera.hpp>
#include <depthai/pipeline/node/MonoCamera.hpp>
#include <depthai/pipeline/node/XLinkOut.hpp>
#include <depthai/utility/Initialization.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

class DepthAiCameraNode : public rclcpp::Node
{
  public:
    using ImageMsg = sensor_msgs::msg::Image;

    const std::array<std::string, 3> StreamNames{"left", "right", "color"};

    DepthAiCameraNode() : Node("depthai_node"), _converter(true)
    {
        declare_parameter<std::string>("parameters_uri", "package://depthai_ctrl/params");

        const std::string parameters_uri = get_parameter("parameters_uri").as_string();

        const std::string left_camera_topic = "~/camera/" + StreamNames[0] + "/image";
        const std::string right_camera_topic = "~/camera/" + StreamNames[1] + "/image";
        const std::string color_camera_topic = "~/camera/" + StreamNames[2] + "/image";

        const std::string left_camera_frame = StreamNames[0] + "_camera";
        const std::string right_camera_frame = StreamNames[1] + "_camera";
        const std::string color_camera_frame = StreamNames[2] + "_camera";

        auto monoLeft = _p.create<dai::node::MonoCamera>();
        auto monoRight = _p.create<dai::node::MonoCamera>();
        auto colorCam = _p.create<dai::node::ColorCamera>();
        auto xoutLeft = _p.create<dai::node::XLinkOut>();
        auto xoutRight = _p.create<dai::node::XLinkOut>();
        auto xoutColor = _p.create<dai::node::XLinkOut>();

        // MonoCamera
        monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
        // monoLeft->setFps(5.0);
        monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
        monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
        // monoRight->setFps(5.0);

        colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        colorCam->setInterleaved(true);
        colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
        colorCam->setPreviewSize(1920, 1080);

        // XLinkOut
        xoutLeft->setStreamName(StreamNames[0]);
        xoutRight->setStreamName(StreamNames[1]);
        xoutColor->setStreamName(StreamNames[2]);

        monoLeft->out.link(xoutLeft->input);
        monoRight->out.link(xoutRight->input);
        colorCam->preview.link(xoutColor->input);

        // CONNECT TO DEVICE
        _dev = std::make_unique<dai::Device>(_p);

        _leftQueue = _dev->getOutputQueue(StreamNames[0], 30, false);
        _rightQueue = _dev->getOutputQueue(StreamNames[1], 30, false);
        _colorQueue = _dev->getOutputQueue(StreamNames[2], 30, false);

        _left_publisher = create_publisher<ImageMsg>(left_camera_topic, rclcpp::SensorDataQoS());
        _right_publisher = create_publisher<ImageMsg>(right_camera_topic, rclcpp::SensorDataQoS());
        _color_publisher = create_publisher<ImageMsg>(color_camera_topic, rclcpp::SensorDataQoS());

        RCLCPP_INFO(get_logger(), "DepthAi successfully initialized");

        _thread_running = true;
        _reading_thread = std::thread([&]() {
            while (rclcpp::ok() && _thread_running)
            {
                auto leftPtr = _leftQueue->tryGet<dai::ImgFrame>();
                auto rightPtr = _rightQueue->tryGet<dai::ImgFrame>();
                auto colorPtr = _colorQueue->tryGet<dai::ImgFrame>();

                if (leftPtr != nullptr)
                {
                    auto imgPtr = _converter.toRosMsgPtr(leftPtr);
                    publish(imgPtr, left_camera_frame, _left_publisher);
                    RCLCPP_INFO(get_logger(), "LEFT");
                }
                if (rightPtr != nullptr)
                {
                    auto imgPtr = _converter.toRosMsgPtr(rightPtr);
                    publish(imgPtr, right_camera_topic, _right_publisher);
                    RCLCPP_INFO(get_logger(), "RIGHT");
                }
                if (colorPtr != nullptr)
                {
                    auto imgPtr = _converter.toRosMsgPtr(colorPtr);
                    publish(imgPtr, color_camera_frame, _color_publisher);
                    RCLCPP_INFO(get_logger(), "COLOR");
                }
            }
        });
    }

    ~DepthAiCameraNode()
    {
        _thread_running = false;
        _reading_thread.join();
        _dev->close();
    };

  private:
    void publish(const ImageMsg::Ptr image,
                 const std::string& frame_id,
                 std::shared_ptr<rclcpp::Publisher<ImageMsg>> publisher)
    {
        image->header.frame_id = frame_id;
        publisher->publish(*image);
    }

    std::shared_ptr<dai::DataOutputQueue> _leftQueue;
    std::shared_ptr<dai::DataOutputQueue> _rightQueue;
    std::shared_ptr<dai::DataOutputQueue> _colorQueue;
    std::unique_ptr<dai::Device> _dev;
    dai::Pipeline _p;
    dai::rosBridge::ImageConverter _converter;
    std::shared_ptr<rclcpp::Publisher<ImageMsg>> _left_publisher;
    std::shared_ptr<rclcpp::Publisher<ImageMsg>> _right_publisher;
    std::shared_ptr<rclcpp::Publisher<ImageMsg>> _color_publisher;
    std::thread _reading_thread;
    std::atomic<bool> _thread_running;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DepthAiCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
