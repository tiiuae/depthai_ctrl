#include "depthai_gstreamer.h"

using namespace depthai_ctrl;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::cout << "DepthAI GStreamer Node." << std::endl;
    auto gstreamer = std::make_shared<DepthAIGStreamer>(argc, argv);
    rclcpp::spin(gstreamer);
    rclcpp::shutdown();

    return 0;
}

