#include "depthai_gstreamer.h"

using namespace depthai_ctrl;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::cout << "DepthAI GStreamer Node." << std::endl;
    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto cameraNode = std::make_shared<DepthAIGStreamer>(argc, argv);
    exec.add_node(cameraNode);
    exec.spin();

    rclcpp::shutdown();

    return 0;
}

