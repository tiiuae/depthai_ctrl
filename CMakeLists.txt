cmake_minimum_required(VERSION 3.5)

# Add depthai-core dependency
#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/depthai-core EXCLUDE_FROM_ALL)

set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_definitions(-DSPDLOG_ACTIVE_LEVEL=6)
add_definitions(-DSPDLOG_HEADER_ONLY=1)
add_definitions(-DFMT_HEADER_ONLY=1)
set(BUILD_TESTING OFF)
set(DEPTHAI_INSTALL OFF)
#set(SANITIZE_LINK_STATIC ON)
set(HUNTER_CONFIGURATION_TYPES ${CMAKE_BUILD_TYPE})
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/depthai-core ${CMAKE_CURRENT_BINARY_DIR}/depthai-core EXCLUDE_FROM_ALL)

project(depthai_ctrl_sub)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0)
find_package(nlohmann_json REQUIRED)

include_directories(${GST_INCLUDE_DIRS})
include_directories(include)
link_directories(${GST_LIBRARY_DIRS})

add_executable(depthai_ctrl src/depthai_ctrl.cpp)
ament_target_dependencies(depthai_ctrl PUBLIC rclcpp std_msgs)

# Link with libraries
target_link_libraries(depthai_ctrl
        PRIVATE
        depthai-core
        ${GST_LIBRARIES}
        nlohmann_json::nlohmann_json
        spdlog
        )

add_executable(depthai_ros_node src/depthai_ros_node.cpp src/ImageConverter.cpp)
ament_target_dependencies(depthai_ros_node PUBLIC rclcpp sensor_msgs)
# Link with libraries
target_link_libraries(depthai_ros_node
        PRIVATE
        depthai-core
        spdlog
        )



# Set some errors
if(NOT MSVC)
    target_compile_options(depthai_ctrl PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Werror=return-type>)
endif()

# Set compiler features (c++14), and disables extensions (g++)
set_property(TARGET depthai_ctrl PROPERTY CXX_STANDARD 14)
set_property(TARGET depthai_ctrl PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET depthai_ctrl PROPERTY CXX_EXTENSIONS OFF)

## Set compiler features (c++14), and disables extensions (g++)
#set_property(TARGET depthai_ros_node PROPERTY CXX_STANDARD 14)
#set_property(TARGET depthai_ros_node PROPERTY CXX_STANDARD_REQUIRED ON)
#set_property(TARGET depthai_ros_node PROPERTY CXX_EXTENSIONS OFF)

install(TARGETS depthai_ctrl)
install(TARGETS depthai_ros_node)
