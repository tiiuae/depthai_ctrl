#include "depthai_gstreamer.h"
#include "gtest/gtest.h"
#include <gst/gst.h>
#include <gst/gstcaps.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <rclcpp/rclcpp.hpp>

using CompressedImageMsg = depthai_ctrl::DepthAIGStreamer::CompressedImageMsg;

GTEST_API_ int main(int argc, char* argv[])
{
    gst_init(&argc, &argv);
    rclcpp::init(argc, argv);
    printf("Running main() from %s\n", __FILE__);
    testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();
    gst_deinit();
    return res;
}

static std::atomic<bool> connection_detected{false};

class DepthAIGStreamerTest : public ::testing::Test
{
  public:
    DepthAIGStreamerTest() = default;

  protected:
    static void client_connected_handler(GstRTSPServer* server, GstRTSPClient* client, gpointer user_data)
    {
        std::cout << "Client Connection Detected!" << std::endl;
        connection_detected = true;
    }

    void StartServer()
    {
        if (_loop != nullptr)
            return;

        _rtsp_server_thread = std::thread([&] {
            _loop = g_main_loop_new(NULL, FALSE);
            GstRTSPServer* server = gst_rtsp_server_new();

            g_signal_connect(server, "client-connected", G_CALLBACK(client_connected_handler), NULL);

            GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points(server);
            GstRTSPMediaFactoryURI* factory = gst_rtsp_media_factory_uri_new();
            gst_rtsp_mount_points_add_factory(mounts, "/test", GST_RTSP_MEDIA_FACTORY(factory));

            // attaching server to g_main_loop
            gst_rtsp_server_attach(server, NULL);
            std::cout << "RTSP server is running  ..." << std::endl;
            g_main_loop_run(_loop);
            std::cout << "RTSP Server is stopping ..." << std::endl;
            gst_rtsp_mount_points_remove_factory(mounts, "/test");
            g_object_unref(mounts);
            g_object_unref(server);
        });

        // give it a time to initialize a server
        std::this_thread::sleep_for(std::chrono::seconds(1));
        connection_detected = false;
    }

    void StopServer()
    {
        if (_loop != nullptr)
        {
            g_main_loop_quit(_loop);
        }

        if (_rtsp_server_thread.joinable())
        {
            _rtsp_server_thread.join();
        }

        if (_loop != nullptr)
        {
            g_main_loop_unref(_loop);
            _loop = nullptr;
        }
    }

    std::thread _rtsp_server_thread{};
    GMainLoop* _loop{nullptr};
};

/// Creates GStreamerNode, stream is not started / not connected
TEST_F(DepthAIGStreamerTest, BasicTest)
{
    rclcpp::NodeOptions options{};
    std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;
    EXPECT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

    auto video_publisher = rclcpp::create_publisher<CompressedImageMsg>(
        *gstreamer_node, "camera/color/video", rclcpp::SystemDefaultsQoS());
    EXPECT_EQ(1UL, video_publisher->get_subscription_count());

    EXPECT_FALSE(gstreamer_node->isStreamPlaying());
    EXPECT_NO_THROW(gstreamer_node.reset());
}

/// StartOnBoot enabled, but still not connected -> expect no gst pipeline crush, isStreamPlaying == true
TEST_F(DepthAIGStreamerTest, BasicStartOnBootTest)
{
    rclcpp::NodeOptions options{};
    std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

    std::vector<rclcpp::Parameter>& parameters = options.parameter_overrides();
    parameters.push_back({"start_stream_on_boot", true});
    EXPECT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

    auto video_publisher = rclcpp::create_publisher<CompressedImageMsg>(
        *gstreamer_node, "camera/color/video", rclcpp::SystemDefaultsQoS());
    EXPECT_EQ(1UL, video_publisher->get_subscription_count());

    // stream supposed to be playing, even if server is off
    EXPECT_TRUE(gstreamer_node->isStreamPlaying());

    // we play "default" stream, as no real camera connected
    std::this_thread::sleep_for(std::chrono::seconds(3));
    EXPECT_TRUE(gstreamer_node->isStreamDefault());
    EXPECT_NO_THROW(gstreamer_node.reset());
}

/// Check if test RTSP server starts and stops without problems
TEST_F(DepthAIGStreamerTest, InternalTest)
{
    EXPECT_NO_THROW(StartServer());
    EXPECT_NO_THROW(StopServer());
    EXPECT_FALSE(connection_detected);
}

/// Check if we may connect GStreamerNode to local test rtsp server
/// it will play "default" stream
 TEST_F(DepthAIGStreamerTest, StartOnBootDefaultStreamTest)
{
    EXPECT_NO_THROW(StartServer());

    rclcpp::NodeOptions options{};
    std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

    std::vector<rclcpp::Parameter>& parameters = options.parameter_overrides();
    parameters.push_back({"start_stream_on_boot", true});
    parameters.push_back({"address", "rtsp://127.0.0.1:8554/test"});

    EXPECT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

    EXPECT_TRUE(gstreamer_node->isStreamPlaying());

    // give it a time to connect and initialize a stream
    rclcpp::spin_some(gstreamer_node);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // make sure stream is "default"
    EXPECT_TRUE(gstreamer_node->isStreamDefault());
    EXPECT_NO_THROW(StopServer());

    EXPECT_TRUE(connection_detected);
    EXPECT_NO_THROW(gstreamer_node.reset());
}

/// Start without start_on_boot, then send a "start" command through ROS message
TEST_F(DepthAIGStreamerTest, DelayedStartTest)
{
    EXPECT_NO_THROW(StartServer());

    rclcpp::NodeOptions options{};
    std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

    std::vector<rclcpp::Parameter>& parameters = options.parameter_overrides();
    parameters.push_back({"start_stream_on_boot", false});
    parameters.push_back({"address", "rtsp://127.0.0.1:8554/test"});

    EXPECT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

    // give it a time to initialize
    rclcpp::spin_some(gstreamer_node);

    EXPECT_FALSE(connection_detected); // GStreamer have not tried to connected
    EXPECT_FALSE(gstreamer_node->isStreamPlaying());

    // prepare and send a command
    auto command_publisher = rclcpp::create_publisher<std_msgs::msg::String>(
        *gstreamer_node, "/videostreamcmd", rclcpp::SystemDefaultsQoS());

    ASSERT_EQ(command_publisher->get_subscription_count(), 1);
    std_msgs::msg::String command{};
    command.data = R"({"Command": "start"})";
    command_publisher->publish(command);

    // let GFStreamer to process command
    rclcpp::spin_some(gstreamer_node);

    // give it a time to initialize a stream
    std::this_thread::sleep_for(std::chrono::seconds(3));
    EXPECT_TRUE(connection_detected);
    EXPECT_TRUE(gstreamer_node->isStreamPlaying());

    // make sure stream is "default"
    EXPECT_TRUE(gstreamer_node->isStreamDefault());
    EXPECT_NO_THROW(StopServer());

    EXPECT_NO_THROW(gstreamer_node.reset());
}
