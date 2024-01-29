#!/usr/bin/env python

import launch
from launch.launch_description import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetLaunchConfiguration
from launch_ros.descriptions import ParameterFile
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "depthai_ctrl"
    pkg_path = get_package_share_directory(package_name=pkg_name)

    DRONE_DEVICE_ID = os.getenv('DRONE_DEVICE_ID')

    xacro_path = os.path.join(pkg_path, 'urdf', 'depthai_descr.urdf.xacro')

    exit_if_camera_start_fails = LaunchConfiguration('exit_if_camera_start_fails', default = 'false')
    camera_model = LaunchConfiguration('camera_model',  default = 'OAK-D')
    camera_name  = LaunchConfiguration('camera_name',   default = 'oak')
    base_frame   = LaunchConfiguration('base_frame',    default = 'oak-d_frame')
    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')
    cam_pos_x    = LaunchConfiguration('cam_pos_x',     default = '0.0')
    cam_pos_y    = LaunchConfiguration('cam_pos_y',     default = '0.0')
    cam_pos_z    = LaunchConfiguration('cam_pos_z',     default = '0.0')
    cam_roll     = LaunchConfiguration('cam_roll',      default = '0.0')
    cam_pitch    = LaunchConfiguration('cam_pitch',     default = '0.0')
    cam_yaw      = LaunchConfiguration('cam_yaw',       default = '0.0')
    parameters_file = LaunchConfiguration('params_file')
    use_mono_cams = LaunchConfiguration('use_mono_cams')
    use_raw_color_cam = LaunchConfiguration('use_raw_color_cam')
    use_video_from_color_cam = LaunchConfiguration('use_video_from_color_cam')
    use_auto_focus = LaunchConfiguration('use_auto_focus')
    use_usb_three = LaunchConfiguration('use_usb_three')
    use_neural_network = LaunchConfiguration('use_neural_network')
    use_passthrough_preview = LaunchConfiguration('use_passthrough_preview')
    use_state_publisher = LaunchConfiguration('use_state_publisher')
    remappings = []

    declare_exit_if_camera_start_fails_cmd = DeclareLaunchArgument(
        'exit_if_camera_start_fails',
        default_value=exit_if_camera_start_fails,
        description='Whether to exit if the camera fails to start.')

    declare_camera_model_cmd = DeclareLaunchArgument(
        'camera_model',
        default_value=camera_model,
        description='The model of the camera. Using a wrong camera model can disable camera features. Valid models: `OAK-D, OAK-D-LITE`.')

    declare_camera_name_cmd = DeclareLaunchArgument(
        'camera_name',
        default_value=camera_name,
        description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.')

    declare_base_frame_cmd = DeclareLaunchArgument(
        'base_frame',
        default_value=base_frame,
        description='Name of the base link.')

    declare_parent_frame_cmd = DeclareLaunchArgument(
        'parent_frame',
        default_value=parent_frame,
        description='Name of the parent link from other a robot TF for example that can be connected to the base of the OAK.')

    declare_pos_x_cmd = DeclareLaunchArgument(
        'cam_pos_x',
        default_value=cam_pos_x,
        description='Position X of the camera with respect to the base frame.')

    declare_pos_y_cmd = DeclareLaunchArgument(
        'cam_pos_y',
        default_value=cam_pos_y,
        description='Position Y of the camera with respect to the base frame.')

    declare_pos_z_cmd = DeclareLaunchArgument(
        'cam_pos_z',
        default_value=cam_pos_z,
        description='Position Z of the camera with respect to the base frame.')

    declare_roll_cmd = DeclareLaunchArgument(
        'cam_roll',
        default_value=cam_roll,
        description='Roll orientation of the camera with respect to the base frame.')

    declare_pitch_cmd = DeclareLaunchArgument(
        'cam_pitch',
        default_value=cam_pitch,
        description='Pitch orientation of the camera with respect to the base frame.')

    declare_yaw_cmd = DeclareLaunchArgument(
        'cam_yaw',
        default_value=cam_yaw,
        description='Yaw orientation of the camera with respect to the base frame.')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_path, 'params', 'parameters.yaml']),
        description='Full path to the ROS2 parameters file to use')

    declare_use_mono_cams_cmd = DeclareLaunchArgument(
        'use_mono_cams',
        default_value='false',
        description='The mono camera of the camera.')

    declare_use_raw_color_cam_cmd = DeclareLaunchArgument(
        'use_raw_color_cam',
        default_value='false',
        description='The raw color camera of the camera.')

    declare_use_video_from_color_cam_cmd = DeclareLaunchArgument(
        'use_video_from_color_cam',
        default_value='false',
        description='The video from color camera of the camera.')

    declare_use_auto_focus_cmd = DeclareLaunchArgument(
        'use_auto_focus',
        default_value='false',
        description='The auto focus of the camera.')

    declare_use_usb_three_cmd = DeclareLaunchArgument(
        'use_usb_three',
        default_value='false',
        description='The usb three of the camera.')

    declare_use_neural_network_cmd = DeclareLaunchArgument(
        'use_neural_network',
        default_value='false',
        description='The neural network of the camera.')

    declare_use_passthrough_preview_cmd = DeclareLaunchArgument(
        'use_passthrough_preview',
        default_value='false',
        description='The passthrough preview of the camera.')

    decleare_use_state_publisher_cmd = DeclareLaunchArgument(
        'use_state_publisher',
        default_value='false',
        description='Whether to run the state publisher or not.')

    set_namespace_cmd = SetLaunchConfiguration('my_node_ns', DRONE_DEVICE_ID)

    depthai_node = Node(
            package=pkg_name,
            executable='camera_node',
            name='camera_node',
            output="screen",
            emulate_tty=True,
            namespace=DRONE_DEVICE_ID,
            remappings=remappings,
            parameters=[ParameterFile(parameters_file, allow_substs=True),
                        {
                            'use_mono_cams': use_mono_cams,
                            'use_raw_color_cam': use_raw_color_cam,
                            'use_video_from_color_cam': use_video_from_color_cam,
                            'use_auto_focus': use_auto_focus,
                            'use_usb_three': use_usb_three,
                            'use_neural_network': use_neural_network,
                            'use_passthrough_preview': use_passthrough_preview,
                            'exit_if_camera_start_fails': exit_if_camera_start_fails
                        }],
    )

    rsp_node = Node(
            condition=IfCondition(use_state_publisher),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='oak_state_publisher',
            namespace=DRONE_DEVICE_ID,
            parameters=[{'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ',
                    'camera_name:=', camera_name, ' ',
                    'camera_model:=', camera_model, ' ',
                    'base_frame:=', base_frame, ' ',
                    'parent_frame:=', parent_frame, ' ',
                    'cam_pos_x:=', cam_pos_x, ' ',
                    'cam_pos_y:=', cam_pos_y, ' ',
                    'cam_pos_z:=', cam_pos_z, ' ',
                    'cam_roll:=', cam_roll, ' ',
                    'cam_pitch:=', cam_pitch, ' ',
                    'cam_yaw:=', cam_yaw
                ])}]
        )

    ld = LaunchDescription()
    ld.add_action(declare_exit_if_camera_start_fails_cmd)
    ld.add_action(declare_camera_name_cmd)
    ld.add_action(declare_camera_model_cmd)
    ld.add_action(declare_base_frame_cmd)
    ld.add_action(declare_parent_frame_cmd)
    ld.add_action(declare_pos_x_cmd)
    ld.add_action(declare_pos_y_cmd)
    ld.add_action(declare_pos_z_cmd)
    ld.add_action(declare_roll_cmd)
    ld.add_action(declare_pitch_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_mono_cams_cmd)
    ld.add_action(declare_use_raw_color_cam_cmd)
    ld.add_action(declare_use_video_from_color_cam_cmd)
    ld.add_action(declare_use_auto_focus_cmd)
    ld.add_action(declare_use_usb_three_cmd)
    ld.add_action(declare_use_neural_network_cmd)
    ld.add_action(declare_use_passthrough_preview_cmd)
    ld.add_action(decleare_use_state_publisher_cmd)

    ld.add_action(set_namespace_cmd)

    ld.add_action(rsp_node)
    ld.add_action(depthai_node)
    return ld
