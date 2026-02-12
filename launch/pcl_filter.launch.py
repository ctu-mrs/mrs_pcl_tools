import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
        IfElseSubstitution,
        EnvironmentVariable,
        LaunchConfiguration,
        PathJoinSubstitution,
        PythonExpression,
        )

import os
import sys

def generate_launch_description():

    # ld = launch.LaunchDescription([SetEnvironmentVariable("LD_PRELOAD", "/usr/lib/x86_64-linux-gnu/libasan.so.5")])
    ld = launch.LaunchDescription()

    pkg_name = "mrs_pcl_tools"

    pkg_share_path = get_package_share_directory(pkg_name)

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', "uav1"),
        description="The uav name used for namespacing.",
    ))

    # #} end of custom_config

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=os.getenv('USE_SIM_TIME', "false"),
        description="Should the node subscribe to sim time?",
    ))

    # #} end of custom_config

    # #{ custom_config

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
        ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
            else_value=custom_config
            )

    # #} end of custom_config

    # #{ points_in

    points_in = LaunchConfiguration('points_in')

    ld.add_action(DeclareLaunchArgument(
        'points_in',
        default_value='~/points_in',
        description='pointcloud topic'
    ))

    # #} end of lidar_3d_0

    # #{ standalone

    standalone = LaunchConfiguration('standalone')

    declare_standalone = DeclareLaunchArgument(
        'standalone',
        default_value='true',
        description='Whether to start a as a standalone or load into an existing container.'
    )

    ld.add_action(declare_standalone)

    # #} end of standalone

    # #{ container_name

    container_name = LaunchConfiguration('container_name')

    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    )

    ld.add_action(declare_container_name)

    # #} end of container_name

    # #{ node_name

    node_name = LaunchConfiguration('node_name')

    ld.add_action(DeclareLaunchArgument(
        'node_name',
        default_value='pcl_filter',
        description='The runtime node name.'
    ))

    # #} end of lidar_3d_0

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    # #} end of log_level

    config_files = [
        pkg_share_path + '/config/pcl_filter.yaml'
    ]

    filter_node = ComposableNode(
        package=pkg_name,
        plugin='mrs_pcl_tools::PCLFiltration',
        namespace=uav_name,
        name=node_name,
        parameters=[
            {'config_files': config_files},
            {'custom_config': custom_config},
            {"use_sim_time": use_sim_time},
            {"uav_name": uav_name},
        ],
        remappings=[
            # subscribers
            ("~/points_in", points_in),
            # pulisherrs
            ("~/points_out", "~/filtered_points"),
            ("~/points_over_max_range_out", "~/points_over_max_range"),
        ],
    )

    # #{ load into container

    load_into_existing = LoadComposableNodes(
        target_container= container_name,
        composable_node_descriptions = [filter_node],
        condition = UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    # #} end of load into container

    # #{ standalone container

    ld.add_action(ComposableNodeContainer(
        namespace=uav_name,
        name=[node_name,'_container'],
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        arguments = ['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[filter_node],
        condition = IfCondition(standalone)
    ))

    # #} end of standalone container

    return ld
