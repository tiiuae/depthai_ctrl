cmake_minimum_required(VERSION 3.5)

# all these settings are needed to properly compile depthai-core as subdirectory
set(BUILD_SHARED_LIBS OFF)
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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0)
find_package(nlohmann_json REQUIRED)

include_directories(${GST_INCLUDE_DIRS})
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
add_library(depthai_gstreamer SHARED src/depthai_gstreamer.cpp)
ament_target_dependencies(depthai_gstreamer PUBLIC rclcpp std_msgs sensor_msgs)
target_link_libraries(depthai_gstreamer PUBLIC ${GST_LIBRARIES})

# DepthAI GStreamer as separate node
add_executable(gstreamer_node src/gstreamer_node.cpp)
ament_target_dependencies(gstreamer_node PUBLIC rclcpp rclcpp_components)
target_link_libraries(gstreamer_node PUBLIC depthai_gstreamer)

rclcpp_components_register_nodes(depthai_camera PLUGIN "${PROJECT_NAME}::DepthAICamera" EXECUTABLE depthai_camera)
rclcpp_components_register_nodes(depthai_gstreamer PLUGIN "${PROJECT_NAME}::DepthAIGStreamer" EXECUTABLE depthai_gstreamer)

# Set some errors
#if(NOT MSVC)
#    target_compile_options(depthai_gstreamer PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Werror=return-type>)
#endif()

# Set compiler features (c++14), and disables extensions (g++)
#set_property(TARGET depthai_ctrl PROPERTY CXX_STANDARD 14)
#set_property(TARGET depthai_ctrl PROPERTY CXX_STANDARD_REQUIRED ON)
#set_property(TARGET depthai_ctrl PROPERTY CXX_EXTENSIONS OFF)
#

install(TARGETS depthai_camera depthai_gstreamer camera_node gstreamer_node
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        )

ament_package()