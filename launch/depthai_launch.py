#!/usr/bin/env python

import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
import sys


def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "depthai_ctrl"
    pkg_path = get_package_share_directory(package_name=pkg_name)

    DRONE_DEVICE_ID=os.getenv('DRONE_DEVICE_ID')

    ld.add_action(launch.actions.DeclareLaunchArgument("debug", default_value="false"))
    ld.add_action(launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false"))
    dbg_sub = None
    if sys.stdout.isatty():
        dbg_sub = launch.substitutions.PythonExpression(['"" if "false" == "', launch.substitutions.LaunchConfiguration("debug"), '" else "debug_ros2launch ' + os.ttyname(sys.stdout.fileno()) + '"'])

    namespace=DRONE_DEVICE_ID

    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_depthai',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # DepthAI Camera
            ComposableNode(
                namespace=namespace,
                name='depthai_camera',
                package=pkg_name,
                plugin='depthai_ctrl::DepthAICamera',
                
                parameters=[
                    #pkg_path + '/config/camera_config.yaml',
                    #{"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
                ],
                remappings=[
                ],

            ),
            # DepthAI GStreamer
            ComposableNode(
                namespace=namespace,
                name='depthai_gstreamer',
                package=pkg_name,
                plugin='depthai_ctrl::DepthAIGStreamer',
                remappings=[

                ],
                parameters=[
                    #pkg_path + '/config/gstreamer_config.yaml',
                    {"start_stream_on_boot": True,
                     "address":"rtsps://DroneUser:22f6c4de-6144-4f6c-82ea-8afcdf19f316@video-stream.sacplatform.com:8555/performancetest1"},
                    #{"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},
                ],
            ),
        ],
        output='screen',
        prefix=dbg_sub,
        #parameters=[{"use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")},],
    ))
    return ld
