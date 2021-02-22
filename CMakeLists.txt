cmake_minimum_required(VERSION 3.5)

# Add depthai-core dependency
#add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/depthai-core EXCLUDE_FROM_ALL)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/depthai-core ${CMAKE_CURRENT_BINARY_DIR}/depthai-core EXCLUDE_FROM_ALL)

project(depthai_ctrl_sub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic
)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEPTHAI_INC}")
set(CMAKE_CXX_LDFLAGS "${CMAKE_CXX_LDFLAGS} ${DEPTHAI_LIB}")
message("DEPTHAI_INC: ${DEPTHAI_INC}")
message("DEPTHAI_LIB: ${DEPTHAI_LIB}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET gstreamer-1.0)
find_package(nlohmann_json REQUIRED)

include_directories(${GST_INCLUDE_DIRS})
link_directories(${GST_LIBRARY_DIRS})

add_executable(depthai_ctrl depthai_ctrl.cpp)
ament_target_dependencies(depthai_ctrl rclcpp std_msgs)

# Link with libraries
target_link_libraries(depthai_ctrl depthai-core)
target_link_libraries(depthai_ctrl ${GST_LIBRARIES})
target_link_libraries(depthai_ctrl nlohmann_json::nlohmann_json)

# Set some errors
if(NOT MSVC)
    target_compile_options(depthai_ctrl PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Werror=return-type>)
endif()

# Set compiler features (c++14), and disables extensions (g++)
set_property(TARGET depthai_ctrl PROPERTY CXX_STANDARD 14)
set_property(TARGET depthai_ctrl PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET depthai_ctrl PROPERTY CXX_EXTENSIONS OFF)

install(TARGETS
  depthai_ctrl
  DESTINATION lib/${PROJECT_NAME}
)
