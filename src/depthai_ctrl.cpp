#include <memory>
#include "depthai_camera.h"
#include "depthai_gstreamer.h"
#include <rclcpp/rclcpp.hpp>

using namespace depthai_ctrl;


int main(int argc, char * argv[])
{
    // Force flush of the stdout buffer.
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Initialize any global resources needed by the middleware and the client library.
    // This will also parse command line arguments one day (as of Beta 1 they are not used).
    // You must call this before using any other part of the ROS system.
    // This should be called once per process.
    rclcpp::init(argc, argv);

    // Create an executor that will be responsible for execution of callbacks for a set of nodes.
    // With this version, all callbacks will be called from within this thread (the main one).
    rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::NodeOptions options;

    // Add some nodes to the executor which provide work for the executor during its "spin" function.
    // An example of available work is executing a subscription callback, or a timer callback.
    auto camera = std::make_shared<DepthAICamera>(options);
    exec.add_node(camera);
    auto gstreamer = std::make_shared<DepthAIGStreamer>(argc, argv);
    exec.add_node(gstreamer);

    // spin will block until work comes in, execute work as it becomes available, and keep blocking.
    // It will only be interrupted by Ctrl-C.
    exec.spin();

    rclcpp::shutdown();

    return 0;
}

