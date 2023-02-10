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

#ifndef FOG_SW_DEPTHAI_UTILS_H
#define FOG_SW_DEPTHAI_UTILS_H

#include <arpa/inet.h>
#include <string>

namespace depthai_ctrl
{

static const std::vector<std::string> labelMap = {
  "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
  "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog",
  "horse",
  "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag",
  "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
  "baseball glove",
  "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife",
  "spoon",
  "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza",
  "donut", "cake", "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet", "tvmonitor",
  "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
  "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};

/// ROS2 Node which works as DepthAI Camera driver
struct DepthAIUtils
{
  static std::string ReadIpFromUdpAddress(std::string streamAddress)
  {
    // String format is: udp://<ip_addr>:<port>
    std::string addr = streamAddress;
    std::string udp_protocol = "udp://";
    addr.erase(0, udp_protocol.size());

    return addr.substr(0, addr.find(':'));
  }

  static int ReadPortFromUdpAddress(std::string streamAddress)
  {
    // String format is: udp://<ip_addr>:<port>
    std::string addr = streamAddress;
    std::string udp_protocol = "udp://";
    addr.erase(0, udp_protocol.size());

    return atoi(addr.substr(addr.find(':') + 1).c_str());
  }

  static bool ValidateCameraParameters(
    const int width,
    const int height,
    const double fps,
    const int bitrate,
    const int lens_position,
    std::string encoding,
    std::string & error)
  {
    if (width > 3840 || width % 8 != 0) {
      error = "Required video stream 'width' is incorrect.";
      return false;
    }
    if (height > 2160 || height % 8 != 0) {
      error = "Required video stream 'height' is incorrect.";
      return false;
    }

    if (fps < 0.1 || fps > 60) {
      error = "Required video stream 'fps' is incorrect.";
      return false;
    }

    if (bitrate < 400000) {
      error = "Required video stream 'bitrate' is incorrect.";
      return false;
    }

    if (lens_position < 0 || lens_position > 255) {
      error = "Required video stream 'lens_position' is incorrect.";
      return false;
    }

    if ((encoding != "H264") && (encoding != "H265")) {
      error = "Required video stream 'encoding' is incorrect.";
      return false;
    }
    return true;
  }

  static bool IsIpAddressValid(const std::string & ip_address)
  {
    struct sockaddr_in sa
    {
    };
    int result = inet_pton(AF_INET, ip_address.c_str(), &(sa.sin_addr));
    return result != 0;
  }

  static bool ValidateEncodingProfile(const std::string profile)
  {
    return profile == "H264" || profile == "H265";
  }

  static bool ValidateAddressParameters(const std::string address, std::string & res)
  {
    std::string udp_protocol = "udp://";
    std::string rtsp_protocol = "rtsp://";
    std::string rtsps_protocol = "rtsps://";
    std::string protocol;
    std::string addr = address;

    if (addr.empty()) {
      res = "Empty address.";
      return false;
    }

    if (addr.find(udp_protocol) == 0) {
      protocol = "udp";
      addr.erase(0, udp_protocol.size());
    } else if (addr.find(rtsp_protocol) == 0) {
      protocol = "rtsp";
      addr.erase(0, rtsp_protocol.size());
    } else if (addr.find(rtsps_protocol) == 0) {
      protocol = "rtsps";
      addr.erase(0, rtsps_protocol.size());
    } else {
      res = "Not valid protocol in stream address.";
      return false;
    }

    std::string ip_addr, port;
    if (protocol == "udp") {
      ip_addr = addr.substr(0, addr.find(':'));
      if (ip_addr.empty() || !IsIpAddressValid(ip_addr)) {
        res = "Not valid IP address.";
        return false;
      }
      port = addr.substr(addr.find(':') + 1);
      if (port.empty()) {
        res = "Empty port in address.";
        return false;
      }
    } else if (protocol == "rtsp" || protocol == "rtsps") {
      std::string user, key, path;
      user = addr.substr(0, addr.find(':'));
      if (user.empty()) {
        res = "Empty user in address.";
        return false;
      }
      addr = addr.substr(addr.find(':') + 1);
      key = addr.substr(0, addr.find('@'));
      if (key.empty()) {
        res = "Empty key in address.";
        return false;
      }
      addr = addr.substr(addr.find('@') + 1);
      ip_addr = addr.substr(0, addr.find(':'));
      if (ip_addr.empty() || !IsIpAddressValid(ip_addr)) {
        res = "Not valid IP address.";
        return false;
      }
      addr = addr.substr(addr.find(':') + 1);
      port = addr.substr(0, addr.find('/'));
      if (port.empty()) {
        res = "Empty port in address.";
        return false;
      }
      path = addr.substr(addr.find('/') + 1);
      if (path.empty()) {
        res = "Empty path in address.";
        return false;
      }
    }
    return true;
  }

};

}  // namespace depthai_ctrl

#endif  // FOG_SW_DEPTHAI_UTILS_H
