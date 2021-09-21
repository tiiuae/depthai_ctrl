//
// Created by sergey on 20.9.2021.
//

#ifndef FOG_SW_DEPTHAI_UTILS_H
#define FOG_SW_DEPTHAI_UTILS_H

#include <arpa/inet.h>
#include <string>

namespace depthai_ctrl
{

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

    static bool ValidateCameraParameters(const int width,
                                         const int height,
                                         const int fps,
                                         const int bitrate,
                                         std::string encoding,
                                         std::string& error)
    {
        if (width > 3840 || width % 8 != 0)
        {
            error = "Required video stream 'width' is incorrect.";
            return false;
        }
        if (height > 2160 || height % 8 != 0)
        {
            error = "Required video stream 'height' is incorrect.";
            return false;
        }

        if (fps < 5 || fps > 60)
        {
            error = "Required video stream 'fps' is incorrect.";
            return false;
        }

        if (bitrate < 400000)
        {
            error = "Required video stream 'bitrate' is incorrect.";
            return false;
        }

        if ((encoding != "H264") && (encoding != "H265"))
        {
            error = "Required video stream 'encoding' is incorrect.";
            return false;
        }
        return true;
    }

    static bool IsIpAddressValid(const std::string& ip_address)
    {
        struct sockaddr_in sa
        {
        };
        int result = inet_pton(AF_INET, ip_address.c_str(), &(sa.sin_addr));
        return result != 0;
    }

    static bool ValidateEncodingProfile(const std::string profile) { return profile == "H264" || profile == "H265"; }

    static bool ValidateAddressParameters(const std::string address, std::string& res)
    {
        std::string udp_protocol = "udp://";
        std::string rtsp_protocol = "rtsp://";
        std::string protocol;
        std::string addr = address;

        if (addr.empty())
        {
            res = "Empty address.";
            return false;
        }

        if (addr.find(udp_protocol) == 0)
        {
            protocol = "udp";
            addr.erase(0, udp_protocol.size());
        }
        else if (addr.find(rtsp_protocol) == 0)
        {
            protocol = "rtsp";
            addr.erase(0, rtsp_protocol.size());
        }
        else
        {
            res = "Not valid protocol in stream address.";
            return false;
        }

        std::string ip_addr, port;
        if (protocol == "udp")
        {
            ip_addr = addr.substr(0, addr.find(':'));
            if (ip_addr.empty() || !IsIpAddressValid(ip_addr))
            {
                res = "Not valid IP address.";
                return false;
            }
            port = addr.substr(addr.find(':') + 1);
            if (port.empty())
            {
                res = "Empty port in address.";
                return false;
            }
        }
        else if (protocol == "rtsp")
        {
            std::string user, key, path;
            user = addr.substr(0, addr.find(':'));
            if (user.empty())
            {
                res = "Empty user in address.";
                return false;
            }
            addr = addr.substr(addr.find(':') + 1);
            key = addr.substr(0, addr.find('@'));
            if (key.empty())
            {
                res = "Empty key in address.";
                return false;
            }
            addr = addr.substr(addr.find('@') + 1);
            ip_addr = addr.substr(0, addr.find(':'));
            if (ip_addr.empty() || !IsIpAddressValid(ip_addr))
            {
                res = "Not valid IP address.";
                return false;
            }
            addr = addr.substr(addr.find(':') + 1);
            port = addr.substr(0, addr.find('/'));
            if (port.empty())
            {
                res = "Empty port in address.";
                return false;
            }
            path = addr.substr(addr.find('/') + 1);
            if (path.empty())
            {
                res = "Empty path in address.";
                return false;
            }
        }
        return true;
    }

};

}  // namespace depthai_ctrl

#endif  // FOG_SW_DEPTHAI_UTILS_H
