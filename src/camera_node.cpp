#include "depthai_camera.h"
#include <rclcpp/rclcpp.hpp>

using namespace depthai_ctrl;


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::cout << "DepthAI Camera Node." << std::endl;
    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions options;
    auto cameraNode = std::make_shared<DepthAICamera>();
    exec.add_node(cameraNode);
    exec.spin();

    rclcpp::shutdown();

    return 0;
}