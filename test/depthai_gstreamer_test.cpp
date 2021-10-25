#include "depthai_gstreamer.h"
#include "gtest/gtest.h"
#include <gst/gst.h>
//#include <gst/app/gstappsink.h>
#include <gst/gstcaps.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>

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


class DepthAIGStreamerTest : public ::testing::Test
{
  public:
    DepthAIGStreamerTest()
    {
        connection_detected = false;
    }

    static std::atomic<bool> connection_detected;

  protected:

    static void client_connected_handler(GstRTSPServer*, GstRTSPClient*, gpointer)
    {
        std::cout << "Client Connection Detected!" << std::endl;
//        auto mounts = gst_rtsp_client_get_mount_points(client);
//        if(mounts != nullptr)
//        {
//
//        }
        DepthAIGStreamerTest::connection_detected = true;
//        gst_rtsp_client_close(client);
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
            std::cout << "RTSP Server has started" << std::endl;
            g_main_loop_run(_loop);
            std::cout << "RTSP Server has stopped" << std::endl;
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

std::atomic<bool> DepthAIGStreamerTest::connection_detected{false};

/// Check if our "test environment" (which could be inside Docker) works with the GStreamer.
/// It fails if some essential GST plugins or ENV variables are missing.
TEST_F(DepthAIGStreamerTest, InternalTest_GstPipelineTest)
{
    const std::string pipeline_string =
        "videotestsrc name=source pattern=ball ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! videoconvert ! x264enc ! fakesink name=sink";
    GError* parse_error = nullptr;
    auto pipeline = gst_parse_launch(pipeline_string.c_str(), &parse_error);
    ASSERT_EQ(parse_error, nullptr);
    ASSERT_NE(pipeline, nullptr);
    const auto status = gst_element_set_state(pipeline, GST_STATE_READY);
    EXPECT_EQ(status , GST_STATE_CHANGE_SUCCESS);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    ASSERT_NO_THROW(gst_object_unref(pipeline));
}


/// Check if test RTSP server starts and stops without problems
TEST_F(DepthAIGStreamerTest, InternalTest_RtspServerTest)
{
    ASSERT_NO_THROW(StartServer());
    ASSERT_NO_THROW(StopServer());
    EXPECT_FALSE(connection_detected);
}

/// Creates GStreamerNode, stream is not started / not connected
TEST_F(DepthAIGStreamerTest, BasicTest_CreateDeleteTest)
{
    rclcpp::NodeOptions options{};
    std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;
    ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

    auto video_publisher = rclcpp::create_publisher<CompressedImageMsg>(
        *gstreamer_node, "camera/color/video", rclcpp::SystemDefaultsQoS());
    EXPECT_EQ(1UL, video_publisher->get_subscription_count());

    EXPECT_FALSE(gstreamer_node->IsStreamPlaying());
    ASSERT_NO_THROW(gstreamer_node.reset());
}

/// Creates GStreamerNode with argc/argv constructor
TEST_F(DepthAIGStreamerTest, BasicTest_ArgcArgvTest)
{
    int argc = 1;
    char* argv[] = {(char*)"/executable"};
    std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;
    ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(argc, argv));

    auto video_publisher = rclcpp::create_publisher<CompressedImageMsg>(
        *gstreamer_node, "camera/color/video", rclcpp::SystemDefaultsQoS());
    EXPECT_EQ(1UL, video_publisher->get_subscription_count());

    EXPECT_FALSE(gstreamer_node->IsStreamPlaying());
    ASSERT_NO_THROW(gstreamer_node.reset());
}

/// Node shall not crush when wrong command submitted
TEST_F(DepthAIGStreamerTest, BasicTest_WrongCommandTest)
{
    rclcpp::NodeOptions options{};
    std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;
    gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options);

    // prepare and send a command
    auto command_publisher = rclcpp::create_publisher<std_msgs::msg::String>(
        *gstreamer_node, "/videostreamcmd", rclcpp::SystemDefaultsQoS());

    ASSERT_EQ(command_publisher->get_subscription_count(), 1);
    std_msgs::msg::String command{};
    command.data = R"(dfghj!#$)";
    command_publisher->publish(command);

    // this delay is essential for message passing (o_O)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // let GStreamerNode to process command
    ASSERT_NO_THROW(rclcpp::spin_some(gstreamer_node));
    gstreamer_node.reset();
}

/// Try to break GSTreamer pipeline by sending hackish string
TEST_F(DepthAIGStreamerTest, BasicTest_AddressPipelineHackTest)
{
    rclcpp::NodeOptions options{};
    std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

    std::vector<rclcpp::Parameter>& parameters = options.parameter_overrides();
    parameters.push_back({"start_stream_on_boot", true});
    parameters.push_back({"address", "zzzzz://something ! dsadsdsdfd"});

    ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    EXPECT_FALSE(gstreamer_node->IsStreamPlaying());
    ASSERT_NO_THROW(gstreamer_node.reset());
}

/// StartOnBoot enabled, but still not connected -> expect no gst pipeline crush,
/// EXPECT isStreamPlaying = true ,  isStreamDefault = true
TEST_F(DepthAIGStreamerTest, StartOnBoot_BasicTest)
{
    rclcpp::NodeOptions options{};
    std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

    std::vector<rclcpp::Parameter>& parameters = options.parameter_overrides();
    parameters.push_back({"start_stream_on_boot", true});
    parameters.push_back({"address", "rtsp://127.0.0.1:8554/test"});
    ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

    std::this_thread::sleep_for(std::chrono::seconds(2));
    // stream supposed to be playing, even if server is off
    EXPECT_TRUE(gstreamer_node->IsStreamPlaying());
    // Connectivity error shall be detected
    EXPECT_TRUE(gstreamer_node->IsErrorDetected());
    // we play "default" stream, as no real camera connected
    EXPECT_TRUE(gstreamer_node->IsStreamDefault());
    ASSERT_NO_THROW(gstreamer_node.reset());
}

/// StartOnBoot enabled, but still not connected -> expect no gst pipeline crush, isStreamPlaying == true
TEST_F(DepthAIGStreamerTest, StartOnBoot_UDPTest)
{
    rclcpp::NodeOptions options{};
    std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

    std::vector<rclcpp::Parameter>& parameters = options.parameter_overrides();
    parameters.push_back({"start_stream_on_boot", true});
    parameters.push_back({"address", "udp://127.0.0.1:8554"});
    ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

    std::this_thread::sleep_for(std::chrono::seconds(3));
    // stream supposed to be playing, even if server is off
    EXPECT_TRUE(gstreamer_node->IsStreamPlaying());
    // Connectivity error shall be detected
    EXPECT_TRUE(gstreamer_node->IsErrorDetected());
    // we play "default" stream, as no real camera connected
    EXPECT_TRUE(gstreamer_node->IsStreamDefault());
    ASSERT_NO_THROW(gstreamer_node.reset());
}

/// StartOnBoot enabled, but still not connected -> expect no gst pipeline crush, isStreamPlaying == true
TEST_F(DepthAIGStreamerTest, StartOnBoot_H265Test)
{
    rclcpp::NodeOptions options{};
    std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

    std::vector<rclcpp::Parameter>& parameters = options.parameter_overrides();
    parameters.push_back({"start_stream_on_boot", true});
    parameters.push_back({"address", "rtsp://127.0.0.1:8554/test"});
    parameters.push_back({"encoding", "H265"});

    ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

    auto video_publisher = rclcpp::create_publisher<CompressedImageMsg>(
        *gstreamer_node, "camera/color/video", rclcpp::SystemDefaultsQoS());
    EXPECT_EQ(1UL, video_publisher->get_subscription_count());

    std::this_thread::sleep_for(std::chrono::seconds(2));
    // stream supposed to be playing, even if server is off
    EXPECT_TRUE(gstreamer_node->IsStreamPlaying());
    // we play "default" stream, as no real camera connected
    EXPECT_TRUE(gstreamer_node->IsStreamDefault());
    ASSERT_NO_THROW(gstreamer_node.reset());
}

/// Check if we may connect GStreamerNode to local test rtsp server
/// It must come to rtsp://127.0.0.1:8554, even if we not explicitly set it
TEST_F(DepthAIGStreamerTest, LocalServer_DefaultAddressTest)
{
    ASSERT_NO_THROW(StartServer());

    rclcpp::NodeOptions options{};
    std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

    std::vector<rclcpp::Parameter>& parameters = options.parameter_overrides();
    parameters.push_back({"start_stream_on_boot", true});

    ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

    EXPECT_TRUE(gstreamer_node->IsStreamPlaying());

    // give it a time to connect and initialize a stream
    rclcpp::spin_some(gstreamer_node);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // make sure the stream is "default"
    EXPECT_TRUE(gstreamer_node->IsStreamDefault());
    EXPECT_NO_THROW(StopServer());

    EXPECT_TRUE(connection_detected);
    ASSERT_NO_THROW(gstreamer_node.reset());
}

/// Check if we may connect GStreamerNode to local test rtsp server
/// it will play "default" stream
TEST_F(DepthAIGStreamerTest, LocalServer_DefaultStreamTest)
{
    ASSERT_NO_THROW(StartServer());

    rclcpp::NodeOptions options{};
    std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

    std::vector<rclcpp::Parameter>& parameters = options.parameter_overrides();
    parameters.push_back({"start_stream_on_boot", true});
    parameters.push_back({"address", "rtsp://127.0.0.1:8554/test"});

    ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

    // give it a time to connect and initialize a stream
    rclcpp::spin_some(gstreamer_node);
    std::this_thread::sleep_for(std::chrono::seconds(3));

    EXPECT_TRUE(gstreamer_node->IsStreamPlaying());
    // make sure the stream is "default"
    EXPECT_TRUE(gstreamer_node->IsStreamDefault());
    EXPECT_NO_THROW(StopServer());

    EXPECT_TRUE(connection_detected);
    ASSERT_NO_THROW(gstreamer_node.reset());
}

///// What if RTSP Server wasn't available at the beginning?
//TEST_F(DepthAIGStreamerTest, LocalServer_ServerReconnectTest)
//{
//    rclcpp::NodeOptions options{};
//    std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;
//
//    std::vector<rclcpp::Parameter>& parameters = options.parameter_overrides();
//    parameters.push_back({"start_stream_on_boot", true});
//    parameters.push_back({"address", "rtsp://127.0.0.1:8554/test"});
//
//    ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));
//
//    // give it a time to connect and initialize a stream
//    rclcpp::spin_some(gstreamer_node);
//    std::this_thread::sleep_for(std::chrono::seconds(3));
//
//    EXPECT_TRUE(gstreamer_node->IsStreamPlaying());
//    EXPECT_TRUE(gstreamer_node->IsStreamDefault());
//    EXPECT_TRUE(gstreamer_node->IsErrorDetected());
//    EXPECT_FALSE(connection_detected);
//
//    ASSERT_NO_THROW(StartServer());
//    // give it a chance to re-initialize a stream
//    rclcpp::spin_some(gstreamer_node);
//    std::this_thread::sleep_for(std::chrono::seconds(3));
//    EXPECT_TRUE(gstreamer_node->IsStreamPlaying());
//    EXPECT_TRUE(gstreamer_node->IsStreamDefault());
//    EXPECT_TRUE(gstreamer_node->IsErrorDetected());
//    // This time we have to detect connection!
//    EXPECT_TRUE(connection_detected);
//    ASSERT_NO_THROW(gstreamer_node.reset());
//    EXPECT_NO_THROW(StopServer());
//}

/// Start without start_on_boot, then send a "start" command through ROS message
TEST_F(DepthAIGStreamerTest, LocalServer_DelayedStartTest)
{
    ASSERT_NO_THROW(StartServer());

    rclcpp::NodeOptions options{};
    std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

    std::vector<rclcpp::Parameter>& parameters = options.parameter_overrides();
    parameters.push_back({"start_stream_on_boot", false});
    parameters.push_back({"address", "rtsp://127.0.0.1:8554/test"});

    ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

    EXPECT_FALSE(connection_detected);  // GStreamer have not tried to connected
    EXPECT_FALSE(gstreamer_node->IsStreamPlaying());

    // prepare and send a command
    auto command_publisher = rclcpp::create_publisher<std_msgs::msg::String>(
        *gstreamer_node, "/videostreamcmd", rclcpp::SystemDefaultsQoS());

    ASSERT_EQ(command_publisher->get_subscription_count(), 1);
    std_msgs::msg::String command{};
    command.data = R"({"Command": "start"})";
    command_publisher->publish(command);

    // this delay is essential for message passing (o_O)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // let GStreamerNode to process command
    rclcpp::spin_some(gstreamer_node);

    // give it a time to initialize a stream
    std::this_thread::sleep_for(std::chrono::seconds(3));
    EXPECT_TRUE(connection_detected);
    EXPECT_TRUE(gstreamer_node->IsStreamPlaying());

    // make sure the stream is "default"
    EXPECT_TRUE(gstreamer_node->IsStreamDefault());
    ASSERT_NO_THROW(gstreamer_node.reset());
    EXPECT_NO_THROW(StopServer());
}
//
///// Send several empty ROS messages, pretending we have a camera
//TEST_F(DepthAIGStreamerTest, LocalServer_FakeCameraTest)
//{
//    rclcpp::NodeOptions options{};
//    std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;
//
//    std::vector<rclcpp::Parameter>& parameters = options.parameter_overrides();
//    parameters.push_back({"start_stream_on_boot", false});
//    parameters.push_back({"address", "rtsp://127.0.0.1:8554/test"});
//
//    ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));
//
//    //EXPECT_FALSE(connection_detected);  // GStreamer have not tried to connected
//    EXPECT_FALSE(gstreamer_node->IsStreamPlaying());
//
//    // prepare and send several video messages
//    auto stream_publisher = rclcpp::create_publisher<CompressedImageMsg>(
//        *gstreamer_node, "camera/color/video", rclcpp::SystemDefaultsQoS());
//
//    CompressedImageMsg msg {};
//    msg.header.stamp.sec = 1;
//    msg.header.stamp.nanosec = 1;
//    msg.header.frame_id = "camera";
//    msg.data = {'\0'};
//    msg.format = "H264";
//    stream_publisher->publish(msg);
//    std::this_thread::sleep_for(std::chrono::milliseconds(1));
//    rclcpp::spin_some(gstreamer_node);
//
//    // prepare and send a command
//    auto command_publisher = rclcpp::create_publisher<std_msgs::msg::String>(
//        *gstreamer_node, "/videostreamcmd", rclcpp::SystemDefaultsQoS());
//
//    ASSERT_EQ(command_publisher->get_subscription_count(), 1);
//    std_msgs::msg::String command{};
//    command.data = R"({"Command": "start"})";
//    command_publisher->publish(command);
//
//    // this delay is essential for message passing (o_O)
//    std::this_thread::sleep_for(std::chrono::milliseconds(1));
//    // let GStreamerNode to process command
//    rclcpp::spin_some(gstreamer_node);
//
//    // give it a time to initialize a stream
//    std::this_thread::sleep_for(std::chrono::seconds(3));
//    EXPECT_FALSE(connection_detected);
//    EXPECT_TRUE(gstreamer_node->IsStreamPlaying());
//
//    // make sure the stream is "default"
//    EXPECT_FALSE(gstreamer_node->IsStreamDefault());
//    //EXPECT_NO_THROW(StopServer());
//
//    ASSERT_NO_THROW(gstreamer_node.reset());
//}





#ifdef NOT_READY_YET
/// Simulate presence of the real camera, by generating video bitstream and sending it as ROS messages
TEST_F(DepthAIGStreamerTest, SimulateCamera)
{
    std::atomic<bool> gst_running{true};
    std::thread gst_thread = std::thread([&] {
      const std::string pipeline_string =
          "videotestsrc name=source pattern=ball ! videoconvert ! x264enc ! appsink name=sink";

      GError* parse_error = nullptr;
      auto pipeline = gst_parse_launch(pipeline_string.c_str(), &parse_error);

      if (parse_error != nullptr)
      {
          std::cerr << "Gst Parse Error " << parse_error->code << ": " << parse_error->message << std::endl;
          g_clear_error(&parse_error);
          parse_error = nullptr;
      }

      ASSERT_EQ(parse_error, nullptr);
      ASSERT_NE(pipeline, nullptr);
      auto appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
      ASSERT_NE(appsink, nullptr);
      ASSERT_TRUE(GST_IS_APP_SINK(appsink));



      const auto status = gst_element_set_state(pipeline, GST_STATE_PLAYING);
      ASSERT_EQ(status, 1);
      if (status != 1)
      {
          std::cerr << "Leaving the thread #2" << std::endl;
          gst_element_set_state(pipeline, GST_STATE_NULL);
          gst_object_unref(pipeline);
          return;
      }

      while (gst_running)
      {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      std::cerr << "Leaving the thread #3" << std::endl;
      gst_element_set_state(pipeline, GST_STATE_NULL);
      gst_object_unref(pipeline);
    });

    std::this_thread::sleep_for(std::chrono::seconds(1));

    gst_running = false;
    if (gst_thread.joinable())
    {
        std::cerr << "Try to join GST thread" << std::endl;
        ASSERT_NO_THROW(gst_thread.join());
    }
}
#endif

