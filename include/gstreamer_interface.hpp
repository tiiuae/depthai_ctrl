#ifndef FOG_SW_DEPTHAI_GSTREAMER_INTERFACE_H
#define FOG_SW_DEPTHAI_GSTREAMER_INTERFACE_H
#include <memory>
#include <thread>
#include <iostream>
#include <sstream>
#include <chrono>
#include <stdlib.h>
#include <functional>
#include <algorithm>
#include <string>
#include <cctype>
#include <arpa/inet.h>

#include "depthai_utils.h"
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <gst/gstbus.h>
#include <gst/gstcaps.h>
#include <gst/gstelement.h>
#include <gst/gstpipeline.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <queue>


namespace depthai_ctrl
{

using std::placeholders::_1;

using namespace std::chrono_literals;
using CompressedImageMsg = sensor_msgs::msg::CompressedImage;
class GstInterface
{
public:
  //! @brief Constructor
  //! Constructor for GstInterface class
  //! @return void
  //!
  GstInterface(int argc, char * argv[]);

  //! @brief Destructor
  //! Destructor for GstInterface class
  //! Destroy the pipeline and free the memory
  //! @return void
  //!
  ~GstInterface();

  //! @brief Initialize the class
  //! @return void
  //!
  void Init();

  //! @brief Start the stream
  //! Invoked when there is at least one message in the queue.
  //! Calls PlayStream method and connects need-data signal to the callback
  //! @return void
  //!
  void StartStream();

  //! @brief Stop the stream
  //! Disconnect signal handler, set state to GST_STATE_NULL,
  //! Quit the gstreamer main loop and set isStreamPlaying to false.
  //! @return void
  //!
  void StopStream();

  //! @brief Destroy the pipeline
  //! @return void
  //!
  void DestroyPipeline();

  //! @brief Build default pipeline if camera is not active
  //! This default pipeline creates an empty image with "Camera not detected"
  //! @return void
  //!
  void BuildDefaultPipeline();

  //! @brief Build pipeline in GstInterface
  //! @return void
  //!
  void BuildPipeline();

  //! @brief Return encoder width
  //! @return encoder width
  //!
  int GetEncoderWidth() {return _encoderWidth;}

  //! @brief Set encoder height
  //! @param[in] height - height of the encoder
  //! @return void
  //!
  void SetEncoderHeight(int height)
  {
    if (height > 4096) {
      g_printerr("Height must be smaller than 4096 for H26x encoder profile.\n");
      return;
    }
    if (height % 8 != 0) {
      g_printerr("Height must be multiple of 8 for H26x encoder profile.\n");
      return;
    }
    _encoderHeight = height;
  }

  //! @brief Return encoder height
  //! @return encoder height
  //!
  int GetEncoderHeight() {return _encoderHeight;}

  //! @brief Set encoder fps
  //! @param[in] fps - fps of the encoder
  //! @return void
  //!
  void SetEncoderFps(int fps)
  {
    if (fps > 60) {
      g_printerr("Too high frames per second.\n");
      return;
    }
    _encoderFps = fps;
  }
  //! @brief Return encoder fps
  //!
  int GetEncoderFps() {return _encoderFps;}

  //! @brief Set encoder bitrate
  //! @param[in] bitrate - bitrate of the encoder
  //! @return void
  //!
  void SetEncoderBitrate(int bitrate)
  {
    _encoderBitrate = bitrate;
  }

  //! @brief Return encoder bitrate
  //! @return encoder bitrate
  //!
  int GetEncoderBitrate() {return _encoderBitrate;}

  //! @brief Set encoder profile
  //! @param[in] profile - profile of the encoder
  //! @return void
  //!
  void SetEncoderProfile(std::string profile)
  {
    std::transform(profile.begin(), profile.end(), profile.begin(), ::toupper);
    if (profile != "H264" && profile != "H265") {
      g_printerr("Not valid H26x profile.\n");
      return;
    }
    _encoderProfile = profile;
  }

  //! @brief Return encoder profile
  //! @return encoder profile
  //!
  const std::string & GetEncoderProfile() {return _encoderProfile;}

  //! @brief Set stream address for GST sink
  //! @param[in] address - address of the stream
  //! If GST uses RTSP sink, it will be set to gst sink object
  //! If GST uses UDP sink, it will be set to stream address variable
  //! @return void
  //!
  void SetStreamAddress(const std::string address)
  {
    _streamAddress = address;
    if (_rtspSink) {
      g_object_set(
        G_OBJECT(_rtspSink),
        "protocols", 4,                 // 4 = tcp
        "location", _streamAddress.c_str(),
        NULL);
    }
  }

  //! @brief Return encoder profile
  //! @return encoder profile
  //!
  const std::string & GetStreamAddress() {return _streamAddress;}

  //! @brief Return is stream playing private boolean.
  //! @return true if stream is playing, false otherwise.
  //!
  bool IsStreamPlaying() {return _isStreamPlaying;}

  //! @brief Return is stream default private boolean.
  //! @return true if stream is default, false otherwise.
  //!
  bool IsStreamDefault() {return _isStreamDefault;}

  //! @brief Incoming message queue, shared with ROS2 node.
  std::queue<CompressedImageMsg::SharedPtr> queue {};
  //! @brief Mutex for the incoming message queue.
  std::mutex queueMutex {};

  GCond haveDataCond {};
  GMutex haveDataCondMutex {};

  GCond startStreamingCond {};
  GMutex startStreamingCondMutex {};

protected:
  //! @brief GThreadFunc for gstreamer main loop
  //! @param[in] data - GstInterface object
  //! @return void
  //!
  static void * PlayStream(gpointer data);

static void * CreatePipeline(gpointer data);

  //! @brief Missing plugin message, used by StreamEventCallback
  //! @param[in] msg - GstMessage object
  //! @return true if plugin is missing
  //!
  static gboolean gst_is_missing_plugin_message(GstMessage * msg)
  {
    if (GST_MESSAGE_TYPE(msg) != GST_MESSAGE_ELEMENT ||
      gst_message_get_structure(msg) == NULL)
    {
      return FALSE;
    }

    return gst_structure_has_name(gst_message_get_structure(msg), "missing-plugin");
  }

  //! @brief Missing plugin message get description, used by StreamEventCallback
  //! @param[in] msg - GstMessage object
  //! @return description of the missing plugin
  //!
  static const gchar * gst_missing_plugin_message_get_description(GstMessage * msg)
  {
    return gst_structure_get_string(gst_message_get_structure(msg), "name");
  }

  //! @brief GstBusSyncHandler for gstreamer main loop

  //! @param[in] bus - GstBus object
  //! @param[in] message - GstMessage object
  //! @param[in] data - GstInterface object
  //! @return true
  //!
  static gboolean StreamEventCallBack(GstBus * bus, GstMessage * message, gpointer data)
  {
    (void)bus;
    g_debug("%s: Got %s message\n", __FUNCTION__, GST_MESSAGE_TYPE_NAME(message));

    GstInterface * depthAIGst = (GstInterface *)data;
    GstTagList * list = nullptr;

    switch (GST_MESSAGE_TYPE(message)) {
      case GST_MESSAGE_STREAM_STATUS:
        GstStreamStatusType statusType;
        GstElement * element;

        gst_message_parse_stream_status(message, &statusType, &element);
        g_print(
          "Element %s stream status type %d.\n",
          GST_OBJECT_NAME(message->src),
          statusType);
        break;

      case GST_MESSAGE_PROGRESS:
        GstProgressType progressType;
        gchar * code, * text;

        gst_message_parse_progress(message, &progressType, &code, &text);
        switch (progressType) {
          case GST_PROGRESS_TYPE_START:
            g_print("Progress: (%s) %s (Start)\n", code, text);
            break;
          case GST_PROGRESS_TYPE_CONTINUE:
            g_print("Progress: (%s) %s (Continue)\n", code, text);
            break;
          case GST_PROGRESS_TYPE_COMPLETE:
            g_print("Progress: (%s) %s (Complete)\n", code, text);
            break;
          case GST_PROGRESS_TYPE_CANCELED:
            g_print("Progress: (%s) %s (Canceled)\n", code, text);
            break;
          case GST_PROGRESS_TYPE_ERROR:
            g_print("Progress: (%s) %s (Error)\n", code, text);
            break;
          default:
            break;
        }
        g_free(code);
        g_free(text);
        break;

      case GST_MESSAGE_NEW_CLOCK:
        GstClock * clock;

        gst_message_parse_new_clock(message, &clock);
        g_print("New clock: %s\n", (clock ? GST_OBJECT_NAME(clock) : "NULL"));
        break;

      case GST_MESSAGE_LATENCY:
        g_print("Redistribute latency...\n");
        gst_bin_recalculate_latency(GST_BIN(depthAIGst->_pipeline));
        break;

      case GST_MESSAGE_ELEMENT:
        if (gst_is_missing_plugin_message(message)) {
          const gchar * desc;

          desc = gst_missing_plugin_message_get_description(message);
          g_print("Missing element: %s\n", desc ? desc : "(no description)");
        }
        break;

      case GST_MESSAGE_STATE_CHANGED:
        GstState old_state, new_state;

        gst_message_parse_state_changed(message, &old_state, &new_state, NULL);
        g_print(
          "Element %s changed state from %s to %s.\n",
          GST_OBJECT_NAME(message->src),
          gst_element_state_get_name(old_state),
          gst_element_state_get_name(new_state));
        if (g_strrstr(GST_OBJECT_NAME(message->src), "rtspbin") &&
          new_state == GST_STATE_PLAYING)
        {
          depthAIGst->_isStreamPlaying = true;
        }/*
        if (g_strrstr(GST_OBJECT_NAME(message->src), "appsrc") &&
          new_state == GST_STATE_NULL)
        {
          depthAIGst->_isStreamPlaying = false;
          if (depthAIGst->_pipeline != nullptr) {
          std::cout << "Unreferencing pipeline element!" << std::endl;
            gst_object_unref (GST_OBJECT(depthAIGst->_pipeline));
          }
          std::cout << "Quitting main gst loop!" << std::endl;  
          if (depthAIGst->_mLoop != nullptr) {
            g_main_loop_quit(depthAIGst->_mLoop);
          }
          std::cout << "Quitting main loop thread!" << std::endl;
          if (depthAIGst->_mLoopThread != nullptr) {
            g_thread_join(depthAIGst->_mLoopThread);
          }
        }*/
        break;

      case GST_MESSAGE_EOS:
        g_print("End of stream.\n");
        g_main_loop_quit(depthAIGst->_mLoop);
        break;

      case GST_MESSAGE_TAG:
        list = gst_tag_list_new_empty();

        gst_message_parse_tag(message, &list);

        g_print("Tag: %s.\n", gst_tag_list_to_string(list));
        gst_tag_list_unref(list);
        break;

      case GST_MESSAGE_WARNING:
        gchar * warnDebug;
        GError * warning;

        gst_message_parse_warning(message, &warning, &warnDebug);
        g_free(warnDebug);

        g_warning("Warning: %s.\n", warning->message);
        g_error_free(warning);
        break;

      case GST_MESSAGE_ERROR:
        gchar * errDebug;
        GError * error;
        GSource * source;

        gst_message_parse_error(message, &error, &errDebug);
        g_printerr(
          "ERROR from element %s: %s\n",
          GST_OBJECT_NAME(message->src), error->message);
        g_printerr("Debugging info: %s\n", (errDebug) ? errDebug : "none");
        if (error->code == G_FILE_ERROR_NODEV &&
          g_strrstr(error->message, "Could not open resource for reading and writing"))
        {
          GstFlowReturn ret;
          if (depthAIGst->_needDataSignalId != 0) {
            g_signal_handler_disconnect(depthAIGst->_appSource, depthAIGst->_needDataSignalId);
          }
          if (depthAIGst->_appSource != nullptr) {
            g_signal_emit_by_name(depthAIGst->_appSource, "end-of-stream", &ret);
            if (ret != GST_FLOW_OK) {
              g_printerr("Error: Emit end-of-stream failed\n");
            }
          }
          if (depthAIGst->_pipeline != nullptr) {
            gst_element_set_state(depthAIGst->_pipeline, GST_STATE_NULL);
          }
          depthAIGst->_isStreamPlaying = false;
          // Restart stream after two seconds.
          source = g_timeout_source_new(2000);
          g_source_set_callback(
            source,
            GstInterface::StreamPlayingRestartCallback,
            depthAIGst,
            GstInterface::StreamPlayingRestartDone);
          g_source_attach(source, depthAIGst->_mLoopContext);
          g_source_unref(source);
        }
        g_error_free(error);
        g_free(errDebug);
        break;

      default:
        break;
    }
    return true;
  }

  //! @brief Callback for the need-data signal.
  //! When the need-data signal is emitted, this callback will be called.
  //! It will push data to the appsrc using the queue of the ROS2 node.
  //! @param[in] appsrc The appsrc element.
  //! @param[in] user_data GstInterface object
  //! @param[in] unused_size The unused size of the buffer.
  //! @return void
  //!
  static void NeedDataCallBack(GstElement * appsrc, guint unused_size, gpointer user_data);

  //! @brief Restart stream callback invoked by the timeout source.
  //! @param[in] data GstInterface object
  //! @return false
  //!
  static gboolean StreamPlayingRestartCallback(gpointer user_data);

  //! @brief Restart stream done callback invoked by the timeout source.
  //!
  static void StreamPlayingRestartDone(gpointer user_data);




private:
  //! @brief The gst pipeline element
  GstElement * _pipeline {};
  //! @brief The gst appsource element
  GstElement * _appSource {};
  //! @brief bus watch id for gst messages
  guint _busWatchId;
  //! @brief gst bus object
  GstBus * _bus;
  //! @brief need-data signal id
  guint _needDataSignalId;
  //! @brief stamp of the first frame received from the camera
  GstClockTime _stamp0 {};
  //! @brief is stream playing private boolean
  bool _isStreamPlaying = false;
  //! @brief is stream default private boolean
  bool _isStreamDefault = false;
  //! @brief is stream shutdown started flag
  bool _isStreamShutdown = false;
  //! @brief The main gst loop context
  GMainContext * _mLoopContext;
  //! @brief Pipeline creating thread, only ran once
  GThread * _mCreatePipelineThread;
  //! @brief The main gst loop thread
  GThread * _mLoopThread;
  //! @brief the main gst loop
  GMainLoop * _mLoop;
  //! @brief encoder profile for the pipeline
  std::string _encoderProfile {};
  //! @brief stream address, either starts with udp:// or rtsps://
  std::string _streamAddress {};



  GstElement *_testSrc;
  GstElement *_textOverlay;
  GstElement *_h26xEnc;
  GstElement *_testSrcFilter;
  GstElement *_h26xEncFilter;
  GstElement *_h26xparse;
  GstElement *_h26xpay;
  GstElement *_udpSink;
  GstElement *_queue1;
  GstElement * _rtspSink;
  int _encoderWidth;
  int _encoderHeight;
  int _encoderFps;
  int _encoderBitrate;
};
} // namespace depthai_ctrl
#endif //FOG_SW_DEPTHAI_GSTREAMER_INTERFACE_H
