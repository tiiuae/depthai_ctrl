#include "depthai_camera.h"

using namespace depthai_ctrl;


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::cout << "DepthAI Camera Node." << std::endl;
    auto cameraNode = std::make_shared<DepthAICamera>();
#ifdef CONTINUE_TRYING_TO_CONNECT
    while(rclcpp::ok())
    {
        try
        {
            rclcpp::spin_some(cameraNode);
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
    rclcpp::spin(cameraNode);
#endif

    rclcpp::shutdown();

    return 0;
}