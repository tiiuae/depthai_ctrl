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
  //! Creates a g_thread for CreatePipeline method
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

  //! @brief Return stream bitrate info
  //! @return stream bitrate
  //!
  uint GetStreamBitrate() {return _stats_encoder_bitrate;}
  uint GetStreamMinBitrate() {return _stats_encoder_min_bitrate;}
  uint GetStreamMaxBitrate() {return _stats_encoder_max_bitrate;}
  uint GetStreamNominalBitrate() {return _stats_encoder_nominal_bitrate;}

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

  //! @brief Return is stream starting private boolean.
  //! @return true if stream starting command given, false otherwise.
  //!
  bool IsStreamStarting() {return _isStreamStarting;}

  //! @brief Return is error detected private boolean.
  //! Error might be caused by connection error, network failure..
  //! @return true if error detected, false otherwise.
  //!
  bool IsErrorDetected() {return _isErrorDetected;}

  //! @brief Incoming message queue, shared with ROS2 node.
  std::queue<CompressedImageMsg::SharedPtr> queue {};

  //! @brief GCond for queue
  GCond haveDataCond {};
  GMutex haveDataCondMutex {};

protected:
  //! @brief GThreadFunc for gstreamer main loop
  //! @param[in] data - GstInterface object pointer
  //! @return void
  //!
  static void * PlayStream(gpointer data);

  //! @brief Create pipeline for the stream
  //! This function is called from StartStream function in a g_thread
  //! It waits two seconds for the data to be received from the ROS2 node
  //! If no data is received, it will create a default pipeline(Camera Not Found! stream)
  //! otherwise normal stream from camera
  //! @param[in] data - GstInterface object
  //! @return void
  static void * CreatePipeline(gpointer data);

  //! @brief Missing plugin message, used by StreamEventCallback
  //! @param[in] msg - GstMessage object pointer
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
        if (new_state == GST_STATE_PLAYING) {
          if (g_strrstr(GST_OBJECT_NAME(message->src), "rtspbin"))
          {
            depthAIGst->_isStreamPlaying = true;
            depthAIGst->_isErrorDetected = false;
          }
          if ((depthAIGst->_streamAddress.find("udp://") == 0)){
            if (g_strrstr(GST_OBJECT_NAME(message->src), "default_pipeline")||
              g_strrstr(GST_OBJECT_NAME(message->src), "rgbCamSink_pipeline")){
              depthAIGst->_isStreamPlaying = true;
              depthAIGst->_isErrorDetected = false;
            }
          }
        }

        break;

      case GST_MESSAGE_EOS:
        g_print("End of stream.\n");
        g_main_loop_quit(depthAIGst->_mLoop);
        break;

      case GST_MESSAGE_TAG:
        list = gst_tag_list_new_empty();

        gst_message_parse_tag(message, &list);

        g_print("Tag: %s.\n", gst_tag_list_to_string(list));
        
        gst_tag_list_get_uint_index(list, GST_TAG_BITRATE, 0, &depthAIGst->_stats_encoder_bitrate);
        gst_tag_list_get_uint_index(list, GST_TAG_MINIMUM_BITRATE, 0, &depthAIGst->_stats_encoder_min_bitrate);
        gst_tag_list_get_uint_index(list, GST_TAG_MAXIMUM_BITRATE, 0, &depthAIGst->_stats_encoder_max_bitrate);
        gst_tag_list_get_uint_index(list, GST_TAG_NOMINAL_BITRATE, 0, &depthAIGst->_stats_encoder_nominal_bitrate);
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
        //GSource * source;

        gst_message_parse_error(message, &error, &errDebug);
        g_printerr(
          "ERROR from element %s: %s\n",
          GST_OBJECT_NAME(message->src), error->message);
        g_printerr("Debugging info: %s\n", (errDebug) ? errDebug : "none");
        depthAIGst->_isStreamPlaying = false;
        depthAIGst->_isErrorDetected = true;
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

private:
  //! @brief The gst pipeline element
  GstElement * _pipeline {};
  //! @brief The gst appsource element
  GstElement * _appSource {};
  //! @brief bus watch id for gst messages, stream event handling
  guint _busWatchId;
  //! @brief gst bus object, unref after use
  GstBus * _bus;
  //! @brief need-data signal id, connected to appsrc
  guint _needDataSignalId;
  //! @brief Turned on when the start command is being processing.
  bool _isStreamStarting = false;
  //! @brief Raised by the gst bus listener when rtspbin's state is playing
  //! If gst pipeline returns an error, it will be set to false.
  bool _isStreamPlaying = false;
  //! @brief is stream default private boolean
  bool _isStreamDefault = false;
  //! @brief is stream shutdown started flag
  bool _isStreamShutdown = false;
  //! @brief is error detected flag
  bool _isErrorDetected = false;
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

  //! @brief The GST pipeline elements for class-wide access
  GstElement * _testSrc;
  GstElement * _textOverlay;
  GstElement * _h26xEnc;
  GstElement * _testSrcFilter;
  GstElement * _h26xEncFilter;
  GstElement * _h26xparse;
  GstElement * _h26xpay;
  GstElement * _udpSink;
  GstElement * _videoConvert;
  GstElement * _queue1;
  GstElement * _rtspSink;
  //! @brief The start time is saved for the first frame
  //! and used to calculate the relative timestamp of the next frame.
  GstClockTime _gstStartTimestamp;
  //! @brief Last frame's timestamp is recorded, and used for frame duration in buffer
  GstClockTime _gstTimestamp;
  int _encoderWidth;
  int _encoderHeight;
  int _encoderFps;
  int _encoderBitrate;
  uint _stats_encoder_max_bitrate;
  uint _stats_encoder_min_bitrate;
  uint _stats_encoder_nominal_bitrate;
  uint _stats_encoder_bitrate;
};
} // namespace depthai_ctrl
#endif //FOG_SW_DEPTHAI_GSTREAMER_INTERFACE_H
