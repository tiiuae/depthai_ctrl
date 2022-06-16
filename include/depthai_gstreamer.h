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

#ifndef FOG_SW_DEPTHAI_GSTREAMER_H
#define FOG_SW_DEPTHAI_GSTREAMER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>
#include <gstreamer_interface.hpp>

namespace depthai_ctrl
{

/// ROS2 Node, taking video stream coming from ROS2 messages and transmitting it to the specified RSTP/UDP address
class DepthAIGStreamer : public rclcpp::Node
{
  public:
    using CompressedImageMsg = sensor_msgs::msg::CompressedImage;

    DepthAIGStreamer(int argc, char* argv[]);
    DepthAIGStreamer(const rclcpp::NodeOptions & options);
    ~DepthAIGStreamer();

  bool IsStreamPlaying() { return _impl->IsStreamPlaying();}
  bool IsStreamDefault() { return _impl->IsStreamDefault();}
  bool IsErrorDetected() { return _impl->IsErrorDetected();}
  private:

    GstInterface *_impl;
    rclcpp::Subscription<CompressedImageMsg>::SharedPtr _video_subscriber;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _stream_command_subscriber;
    rclcpp::TimerBase::SharedPtr _handle_stream_status_timer;

    rclcpp::CallbackGroup::SharedPtr _callback_group_timer;
    rclcpp::CallbackGroup::SharedPtr _callback_group_video_subscriber;
    rclcpp::CallbackGroup::SharedPtr _callback_group_cmd_subscriber;

    bool _is_stop_requested;
    
    void Initialize();
    void GrabVideoMsg(CompressedImageMsg::SharedPtr video_msg);
    void HandleStreamStatus();
    void VideoStreamCommand(const std_msgs::msg::String::SharedPtr msg);

};

} // namespace depthai_ctrl

#endif  // FOG_SW_DEPTHAI_GSTREAMER_H
