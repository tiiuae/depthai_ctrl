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

#include "depthai_ctrl/depthai_camera.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace depthai_ctrl;


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::cout << "DepthAI Camera Node." << std::endl;
  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions options;
  std::cout << "DepthAICamera in" << std::endl;
  auto cameraNode = std::make_shared<DepthAICamera>();
  std::cout << "DepthAICamera out" << std::endl;
  exec.add_node(cameraNode);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}
