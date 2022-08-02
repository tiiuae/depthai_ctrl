cmake_minimum_required(VERSION 3.5)

# all these settings are needed to properly compile depthai-core as subdirectory
set(BUILD_SHARED_LIBS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_definitions(-DSPDLOG_ACTIVE_LEVEL=6)
add_definitions(-DSPDLOG_HEADER_ONLY=1)
add_definitions(-DFMT_HEADER_ONLY=1)
set(HUNTER_CONFIGURATION_TYPES ${CMAKE_BUILD_TYPE}) # makes possible to compile also Debug build
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/depthai-core ${CMAKE_CURRENT_BINARY_DIR}/depthai-core EXCLUDE_FROM_ALL)

project(depthai_ctrl)

# pkg-config configurations path.
if(NOT DEFINED ENV{PKG_CONFIG_PATH})
  set(ENV{PKG_CONFIG_PATH} "/usr/lib/x86_64-linux-gnu/pkgconfig/")
else()
  string(FIND "/usr/lib/x86_64-linux-gnu/pkgconfig/" $ENV{PKG_CONFIG_PATH} POS_INC_PATH)
  if(${POS_INC_PATH} STREQUAL "-1")
    set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:/usr/lib/x86_64-linux-gnu/pkgconfig/")
  endif()
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic
)
endif()

#find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0)
find_package(nlohmann_json REQUIRED)

include_directories(SYSTEM ${GST_INCLUDE_DIRS})
include_directories(include)
link_directories(${GST_LIBRARY_DIRS})

# DepthAI Camera as Component library
add_library(depthai_camera SHARED src/depthai_camera.cpp)
ament_target_dependencies(depthai_camera PUBLIC rclcpp std_msgs sensor_msgs)
target_link_libraries(depthai_camera PUBLIC depthai-core)


# DepthAI Camera as separate node
add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node PUBLIC rclcpp rclcpp_components)
target_link_libraries(camera_node PUBLIC depthai_camera)


# DepthAI GStreamer as Component library
add_library(gstreamer_interface SHARED src/gstreamer_interface.cpp)
ament_target_dependencies(gstreamer_interface PUBLIC rclcpp std_msgs sensor_msgs)
target_link_libraries(gstreamer_interface PUBLIC ${GST_LIBRARIES} gstapp-1.0)

# DepthAI GStreamer as Component library
add_library(depthai_gstreamer SHARED src/depthai_gstreamer.cpp)
ament_target_dependencies(depthai_gstreamer PUBLIC rclcpp std_msgs sensor_msgs diagnostic_updater)
target_link_libraries(depthai_gstreamer PUBLIC ${GST_LIBRARIES} gstapp-1.0 gstreamer_interface)

# DepthAI GStreamer as separate node
add_executable(gstreamer_node src/gstreamer_node.cpp)
ament_target_dependencies(gstreamer_node PUBLIC rclcpp rclcpp_components)
target_link_libraries(gstreamer_node PUBLIC depthai_gstreamer)

# Combined DepthAI node
add_executable(depthai_ctrl src/depthai_ctrl.cpp)
ament_target_dependencies(depthai_ctrl PUBLIC rclcpp rclcpp_components)
target_link_libraries(depthai_ctrl PUBLIC depthai_camera depthai_gstreamer)

# Explicitly force to re-compile binary whenever library changed
add_dependencies(camera_node depthai_camera)
add_dependencies(depthai_ctrl depthai_gstreamer depthai_camera)
add_dependencies(gstreamer_node depthai_gstreamer)

rclcpp_components_register_nodes(depthai_camera PLUGIN "${PROJECT_NAME}::DepthAICamera" EXECUTABLE depthai_camera)
rclcpp_components_register_nodes(depthai_gstreamer PLUGIN "${PROJECT_NAME}::DepthAIGStreamer" EXECUTABLE depthai_gstreamer)


if (BUILD_TESTING)
  enable_testing()

  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest CONFIG REQUIRED)

  add_executable(depthai_camera_test test/depthai_camera_test.cpp)
  add_executable(depthai_gstreamer_test test/depthai_gstreamer_test.cpp)

  ament_target_dependencies(depthai_camera_test rclcpp rclcpp_components)
  ament_target_dependencies(depthai_gstreamer_test PUBLIC rclcpp rclcpp_components)

  target_link_libraries(depthai_camera_test depthai_camera gtest pthread)
  target_link_libraries(depthai_gstreamer_test PUBLIC depthai_gstreamer gstreamer_interface ${GST_LIBRARIES} gstapp-1.0 gstrtspserver-1.0 gtest pthread)

  add_dependencies(depthai_camera_test depthai_camera)
  add_dependencies(depthai_gstreamer_test depthai_gstreamer)

  add_test(depthai_camera_test depthai_camera_test)
  add_test(depthai_gstreamer_test depthai_gstreamer_test)
endif (BUILD_TESTING)

install(TARGETS depthai_camera depthai_gstreamer gstreamer_interface
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS
        camera_node gstreamer_node depthai_ctrl
        DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        )

ament_package()