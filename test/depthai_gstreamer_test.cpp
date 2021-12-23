/*******************************************************************************
* Copyright 2021 Unikie Oy, Technology Innovation Institute
* All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors(Unikie Oy): Mehmet Killioglu, Manuel Segarra-Abad, Sergey */

#include "depthai_ctrl/depthai_gstreamer.h"
#include "gtest/gtest.h"
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/gstcaps.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>

using CompressedImageMsg = depthai_ctrl::DepthAIGStreamer::CompressedImageMsg;
//#define MULTI_THREADING_FIXED
/// Unfortunately, GST RTSP server has some kind of state (hidden static variables),
/// so, its impossible to make all the tests completely independent.
/// Therefore, we have to run one (singleton) RTSP server for all tests, which need it
class StaticRTSPServer
{
public:
  std::atomic<bool> connection_detected{false};

  static StaticRTSPServer & getInstance()
  {
    static StaticRTSPServer instance;
    return instance;
  }

  StaticRTSPServer(StaticRTSPServer const &) = delete;
  void operator=(StaticRTSPServer const &) = delete;

  ~StaticRTSPServer()
  {
    if (_loop != nullptr) {
      g_main_loop_quit(_loop);
    }

    if (_rtsp_server_thread.joinable()) {
      _rtsp_server_thread.join();
    }

    if (_loop != nullptr) {
      g_main_loop_unref(_loop);
      _loop = nullptr;
      std::cout << "RTSP Server has REALLY stopped" << std::endl;
    }
  }

private:
  StaticRTSPServer()
  {
    connection_detected = false;

    _rtsp_server_thread = std::thread(
      [&] {
        _loop = g_main_loop_new(NULL, FALSE);

        GstRTSPServer * server = gst_rtsp_server_new();
        g_signal_connect(server, "client-connected", G_CALLBACK(client_connected_handler), this);

        GstRTSPMediaFactoryURI * factory = gst_rtsp_media_factory_uri_new();
        GstRTSPMountPoints * mounts = gst_rtsp_server_get_mount_points(server);

        gst_rtsp_mount_points_add_factory(mounts, "/test", GST_RTSP_MEDIA_FACTORY(factory));

        // attaching server to g_main_loop
        auto res = gst_rtsp_server_attach(server, NULL);
        std::cout << "RTSP Server has started (result= " << res << ")" << std::endl;
        std::cout << "RTSP Server is serving at " << gst_rtsp_server_get_address(
          server) << std::endl;
        g_main_loop_run(_loop);

        // disconnect "client-connected" signal handler, otherwise its gonna float around
        auto handlers = g_signal_handlers_disconnect_by_data(
          server,
          this);
        std::cout << "RTSP Server has stopped (disconnected " << handlers << " handler(s))" << std::endl;

        gst_rtsp_mount_points_remove_factory(mounts, "/test");
        g_object_unref(mounts);
        g_object_unref(server);
        if (_loop != nullptr) {
          g_main_loop_unref(_loop);
          _loop = nullptr;
          std::cout << "RTSP Server has REALLY stopped" << std::endl;
        }
      });

    // give it a time to initialize a server thread
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  /// Handler for "client_connected" signal
  /// Transmitting instance of class as "gpointer obj" allows to distinguish connections coming from different tests
  static void client_connected_handler(GstRTSPServer *, GstRTSPClient *, gpointer obj)
  {
    std::cout << "Client Connection Detected!" << std::endl;
    auto instance = static_cast<StaticRTSPServer *>(obj);
    instance->connection_detected = true;
  }

  std::thread _rtsp_server_thread{};
  GMainLoop * _loop{nullptr};
};

GTEST_API_ int main(int argc, char * argv[])
{
  gst_init(&argc, &argv);
  rclcpp::init(argc, argv);
  printf("Running main() from %s\n", __FILE__);
  testing::InitGoogleTest(&argc, argv);
  StaticRTSPServer::getInstance();
  printf("Running tests\n");
  auto res = RUN_ALL_TESTS();
  printf("Tests finished, running gst_deinit\n");
  gst_deinit();
  printf("Tests finished and gst_deinit called, exiting\n");
  return res;
}

class DepthAIGStreamerTest : public ::testing::Test
{
public:
  /// reference to RTSP server
  StaticRTSPServer & rtspServer;

  DepthAIGStreamerTest()
  : rtspServer(StaticRTSPServer::getInstance()) {}
  //DepthAIGStreamerTest(void);
  /*virtual ~DepthAIGStreamerTest();
  static void TearDownTestCase(){
      printf("Tests finished, running gst_deinit\n");
      rtspServer.~StaticRTSPServer();
  }*/
};

/// Check if our "test environment" (which could be inside Docker) works with the GStreamer.
/// It fails if some essential GST plugins or ENV variables are missing.
TEST_F(DepthAIGStreamerTest, InternalTest_GstPipelineTest)
{
  const std::string pipeline_string =
    "videotestsrc name=source pattern=ball ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! "
    "videoconvert ! x264enc ! fakesink name=sink";
  GError * parse_error = nullptr;
  auto pipeline = gst_parse_launch(pipeline_string.c_str(), &parse_error);
  ASSERT_EQ(parse_error, nullptr);
  ASSERT_NE(pipeline, nullptr);
  const auto status = gst_element_set_state(pipeline, GST_STATE_READY);
  EXPECT_EQ(status, GST_STATE_CHANGE_SUCCESS);
  gst_element_set_state(pipeline, GST_STATE_NULL);
  ASSERT_NO_THROW(gst_object_unref(pipeline));
}

/// Check if test RTSP server starts and stops without problems
/// Also check if networking is OK, rstp
TEST_F(DepthAIGStreamerTest, InternalTest_RtspServerTest)
{
  rtspServer.connection_detected = false;
  EXPECT_FALSE(rtspServer.connection_detected);

  std::cout << "Trying to catch client connection ..." << std::endl;
  const std::string pipeline_string =
    "videotestsrc name=source pattern=ball ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! "
    "videoconvert ! x264enc ! rtspclientsink location=rtsp://127.0.0.1:8554/test";
  GError * parse_error = nullptr;
  auto pipeline = gst_parse_launch(pipeline_string.c_str(), &parse_error);
  const auto status = gst_element_set_state(pipeline, GST_STATE_PLAYING);
  EXPECT_EQ(status, GST_STATE_CHANGE_SUCCESS);
  std::this_thread::sleep_for(std::chrono::seconds(1));
  gst_element_set_state(pipeline, GST_STATE_NULL);
  ASSERT_NO_THROW(gst_object_unref(pipeline));
  EXPECT_TRUE(rtspServer.connection_detected);
}

/// Creates GStreamerNode, stream is not started / not connected
TEST_F(DepthAIGStreamerTest, BasicTest_CreateDeleteTest)
{
  rclcpp::NodeOptions options{};
  std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

  std::vector<rclcpp::Parameter> & parameters = options.parameter_overrides();
  parameters.push_back({"start_stream_on_boot", false});

  ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

  auto video_publisher = rclcpp::create_publisher<CompressedImageMsg>(
    *gstreamer_node, "camera/color/video", rclcpp::SystemDefaultsQoS());
  EXPECT_EQ(1UL, video_publisher->get_subscription_count());

  EXPECT_FALSE(gstreamer_node->IsStreamPlaying());
  ASSERT_NO_THROW(gstreamer_node.reset());
}

/*
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
*/

/// Node shall not crush when wrong command submitted
TEST_F(DepthAIGStreamerTest, BasicTest_WrongCommandTest)
{
  rclcpp::NodeOptions options{};
  std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

  std::vector<rclcpp::Parameter> & parameters = options.parameter_overrides();
  parameters.push_back({"start_stream_on_boot", false});

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

  std::vector<rclcpp::Parameter> & parameters = options.parameter_overrides();
  parameters.push_back({"start_stream_on_boot", true});
  parameters.push_back({"address", "zzzzz://something ! dsadsdsdfd"});

  ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));
  std::this_thread::sleep_for(std::chrono::seconds(4));
  EXPECT_FALSE(gstreamer_node->IsStreamPlaying());
  ASSERT_NO_THROW(gstreamer_node.reset());
}

/// StartOnBoot enabled, but address is wrong -> expect no gst pipeline crush,
TEST_F(DepthAIGStreamerTest, StartOnBoot_BasicTest)
{
  rclcpp::NodeOptions options{};
  std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

  std::vector<rclcpp::Parameter> & parameters = options.parameter_overrides();
  parameters.push_back({"start_stream_on_boot", true});
  parameters.push_back({"address", "rtsp://UNKNOWN_ADDRESS:11111/test"});
  ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

  std::this_thread::sleep_for(std::chrono::seconds(4));
  // Cannot go into GST_STATE_PLAYING, because address is wrong
  EXPECT_FALSE(gstreamer_node->IsStreamPlaying());
  EXPECT_TRUE(gstreamer_node->IsErrorDetected());
  // we play "default" stream, as no real camera connected
  EXPECT_TRUE(gstreamer_node->IsStreamDefault());

  ASSERT_NO_THROW(gstreamer_node.reset());
}

#ifdef MULTI_THREADING_FIXED
/// Same as before, but UDP address is set
TEST_F(DepthAIGStreamerTest, StartOnBoot_UDPTest)
{
  rclcpp::NodeOptions options{};
  std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

  std::vector<rclcpp::Parameter> & parameters = options.parameter_overrides();
  parameters.push_back({"start_stream_on_boot", true});
  parameters.push_back({"address", "udp://127.0.0.1:8554"});
  ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

  std::this_thread::sleep_for(std::chrono::seconds(5));
  
  EXPECT_TRUE(gstreamer_node->IsStreamPlaying());
  // UDP transfer will start even if there is no server, so no error
  EXPECT_FALSE(gstreamer_node->IsErrorDetected());
  // we play "default" stream, as no real camera connected
  EXPECT_TRUE(gstreamer_node->IsStreamDefault());
  ASSERT_NO_THROW(gstreamer_node.reset());
}

/// Same as before, but H265 encoding is set
TEST_F(DepthAIGStreamerTest, StartOnBoot_H265Test)
{
  rclcpp::NodeOptions options{};
  std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

  std::vector<rclcpp::Parameter> & parameters = options.parameter_overrides();
  parameters.push_back({"start_stream_on_boot", true});
  parameters.push_back({"address", "rtsp://127.0.0.1:8554/test"});
  parameters.push_back({"encoding", "H265"});

  ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

  auto video_publisher = rclcpp::create_publisher<CompressedImageMsg>(
    *gstreamer_node, "camera/color/video", rclcpp::SystemDefaultsQoS());
  EXPECT_EQ(1UL, video_publisher->get_subscription_count());

  std::this_thread::sleep_for(std::chrono::seconds(10));
  
  EXPECT_TRUE(gstreamer_node->IsStreamPlaying());
  EXPECT_TRUE(gstreamer_node->IsErrorDetected());
  // we play "default" stream, as no real camera connected
  EXPECT_TRUE(gstreamer_node->IsStreamDefault());
  ASSERT_NO_THROW(gstreamer_node.reset());
}

/// Check if we may connect GStreamerNode to local test rtsp server
/// It must come to rtsp://127.0.0.1:8554, even if we not explicitly set it
TEST_F(DepthAIGStreamerTest, LocalServer_DefaultAddressTest)
{
  rtspServer.connection_detected = false;

  rclcpp::NodeOptions options{};
  std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

  std::vector<rclcpp::Parameter> & parameters = options.parameter_overrides();
  parameters.push_back({"start_stream_on_boot", true});

  ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));
  std::this_thread::sleep_for(std::chrono::seconds(2));
  EXPECT_TRUE(rtspServer.connection_detected);

  EXPECT_TRUE(gstreamer_node->IsStreamPlaying());
  EXPECT_TRUE(gstreamer_node->IsStreamDefault());
  ASSERT_NO_THROW(gstreamer_node.reset());
}

/// Start without start_on_boot, then send a "start" command through ROS message
TEST_F(DepthAIGStreamerTest, LocalServer_DelayedStartTest)
{
  rtspServer.connection_detected = false;
  rclcpp::NodeOptions options{};
  std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

  std::vector<rclcpp::Parameter> & parameters = options.parameter_overrides();
  parameters.push_back({"start_stream_on_boot", false});
  parameters.push_back({"address", "rtsp://127.0.0.1:8554/test"});

  ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

  // give it a time to initialize a stream -> it must not run though
  std::this_thread::sleep_for(std::chrono::seconds(2));
  EXPECT_FALSE(rtspServer.connection_detected);    // GStreamer have not tried to connected
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
  std::this_thread::sleep_for(std::chrono::seconds(2));
  EXPECT_TRUE(rtspServer.connection_detected);
  EXPECT_TRUE(gstreamer_node->IsStreamPlaying());

  // make sure the stream is "default"
  EXPECT_TRUE(gstreamer_node->IsStreamDefault());
  ASSERT_NO_THROW(gstreamer_node.reset());
}


/// Send several empty ROS messages, pretending we have a camera
TEST_F(DepthAIGStreamerTest, LocalServer_FakeCameraTest)
{
  rclcpp::NodeOptions options{};
  std::shared_ptr<depthai_ctrl::DepthAIGStreamer> gstreamer_node;

  std::vector<rclcpp::Parameter> & parameters = options.parameter_overrides();
  parameters.push_back({"start_stream_on_boot", false});
  parameters.push_back({"address", "rtsp://127.0.0.1:8554/test"});

  rtspServer.connection_detected = false;
  ASSERT_NO_THROW(gstreamer_node = std::make_shared<depthai_ctrl::DepthAIGStreamer>(options));

  // EXPECT_FALSE(connection_detected);  // GStreamer have not tried to connected
  EXPECT_FALSE(gstreamer_node->IsStreamPlaying());

  // prepare and send several video messages
  auto stream_publisher = rclcpp::create_publisher<CompressedImageMsg>(
    *gstreamer_node, "camera/color/video", rclcpp::SystemDefaultsQoS());

  const std::string pipeline_string =
    "videotestsrc num-buffers=1 pattern=ball ! video/x-raw,format=YUY2,width=640,height=480,framerate=30/1 ! "
    "videoconvert ! x264enc ! appsink name=sink";
  GError * parse_error = nullptr;
  auto pipeline = gst_parse_launch(pipeline_string.c_str(), &parse_error);
  ASSERT_EQ(parse_error, nullptr);
  ASSERT_NE(pipeline, nullptr);

  auto appSink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");
  ASSERT_NE(appSink, nullptr);
  ASSERT_TRUE(GST_IS_APP_SINK(appSink));

  gst_element_set_state(pipeline, GST_STATE_PLAYING);

  CompressedImageMsg msg{};
  msg.header.stamp.sec = 1;
  msg.header.stamp.nanosec = 1;
  msg.header.frame_id = "camera";
  msg.data = {'\0'};
  msg.format = "H264";

  GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(appSink));

  ASSERT_NE(sample, nullptr);

  GstBuffer * buffer = gst_sample_get_buffer(sample);

  ASSERT_NE(buffer, nullptr);

  size_t length = GST_BUFFER_OFFSET_END(buffer) - GST_BUFFER_OFFSET(buffer);
  msg.data.resize(length);
  gst_buffer_extract(buffer, GST_BUFFER_OFFSET(buffer), &msg.data[0], length);

  stream_publisher->publish(msg);

  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  rclcpp::spin_some(gstreamer_node);

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
  std::this_thread::sleep_for(std::chrono::seconds(2));
  EXPECT_TRUE(rtspServer.connection_detected);
  EXPECT_TRUE(gstreamer_node->IsStreamPlaying());

  // make sure the stream is "default"
  EXPECT_FALSE(gstreamer_node->IsStreamDefault());

  ASSERT_NO_THROW(gstreamer_node.reset());

  gst_element_set_state(pipeline, GST_STATE_NULL);
  ASSERT_NO_THROW(gst_object_unref(pipeline));
}
#endif
