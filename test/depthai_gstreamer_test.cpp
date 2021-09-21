#include "gtest/gtest.h"
#include "depthai_gstreamer.h"

using CompressedImageMsg = depthai_ctrl::DepthAIGStreamer::CompressedImageMsg;

GTEST_API_ int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    printf("Running main() from %s\n", __FILE__);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/// Test Gstreamer Node -> it's not connected to anywhere
TEST(DepthAIGStreamerTest, BasicTest)
{
    rclcpp::NodeOptions options{};
    std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

    EXPECT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

    auto video_publisher = rclcpp::create_publisher<CompressedImageMsg>(*gstreamer_node,
        "camera/color/video", rclcpp::SystemDefaultsQoS());

    EXPECT_EQ(1UL, video_publisher->get_subscription_count());

    EXPECT_FALSE(gstreamer_node->isStreamPlaying());
    EXPECT_NO_THROW(gstreamer_node.reset());
}


