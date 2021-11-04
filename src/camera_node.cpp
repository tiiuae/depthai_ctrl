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
#ifdef CONTINUE_TRYING_TO_CONNECT
    while(rclcpp::ok())
    {
        try
        {
            //rclcpp::spin_some(cameraNode);
        }
        catch(...)
        {
            cameraNode->Stop();
        }
        if(!cameraNode->IsNodeRunning())
        {
            std::cout << "DepthAI Camera Node is not running!" << std::endl;
            cameraNode->Stop();
            cameraNode->TryRestarting();
        }
    }
#else
    exec.spin();
#endif

    rclcpp::shutdown();

    return 0;
}