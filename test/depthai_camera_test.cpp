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

#include "depthai_camera.h"
#include "gtest/gtest.h"

using ImageMsg = depthai_ctrl::DepthAICamera::ImageMsg;
using CompressedImageMsg = depthai_ctrl::DepthAICamera::CompressedImageMsg;

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    printf("Running main() from %s\n", __FILE__);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/// DepthAI camera is not connected, but ROS Node must not crush anyway
TEST(DepthAICameraTest, BasicTest)
{
    std::shared_ptr<depthai_ctrl::DepthAICamera> camera_node;
    EXPECT_NO_THROW(camera_node = std::make_shared<depthai_ctrl::DepthAICamera>());
    EXPECT_FALSE(camera_node->IsNodeRunning());


    auto left_subscriber = rclcpp::create_subscription<ImageMsg>(
        *camera_node, "camera/left/image_raw", rclcpp::SensorDataQoS(), [](const ImageMsg::SharedPtr a) { (void)a; });

    auto right_subscriber = rclcpp::create_subscription<ImageMsg>(
        *camera_node, "camera/right/image_raw", rclcpp::SensorDataQoS(), [](const ImageMsg::SharedPtr a) { (void)a; });

    auto color_subscriber = rclcpp::create_subscription<ImageMsg>(
        *camera_node, "camera/color/image_raw", rclcpp::SensorDataQoS(), [](const ImageMsg::SharedPtr a) { (void)a; });

    auto video_subscriber = rclcpp::create_subscription<CompressedImageMsg>(
        *camera_node, "camera/color/video", rclcpp::SensorDataQoS(), [](const CompressedImageMsg::SharedPtr a) { (void)a; });

    // Each topic must have 1 publisher
    EXPECT_EQ(1UL, left_subscriber->get_publisher_count());
    EXPECT_EQ(1UL, right_subscriber->get_publisher_count());
    EXPECT_EQ(1UL, color_subscriber->get_publisher_count());
    EXPECT_EQ(1UL, video_subscriber->get_publisher_count());

    EXPECT_NO_THROW(camera_node.reset());
}
