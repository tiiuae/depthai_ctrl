#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <unordered_map>

#include "depthai/depthai.hpp"

    #include "rclcpp/rclcpp.hpp"
    #include "sensor_msgs/msg/imu.hpp"
namespace dai
{

namespace ros
{

namespace ImuMsgs = sensor_msgs::msg;
using ImuPtr = ImuMsgs::Imu::SharedPtr;
class ImuConverter
{
public:
  ImuConverter(const std::string & frameName);

  void toRosMsg(std::shared_ptr<dai::IMUData> inData, ImuMsgs::Imu & outImuMsg);
  ImuPtr toRosMsgPtr(const std::shared_ptr<dai::IMUData> inData);

private:
  uint32_t _sequenceNum;
  const std::string _frameName = "";
};

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
