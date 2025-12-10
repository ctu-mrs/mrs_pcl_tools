import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch.actions import SetEnvironmentVariable
from launch_ros.descriptions import ComposableNode
import os
import sys


def generate_launch_description():

    # ld = launch.LaunchDescription([SetEnvironmentVariable("LD_PRELOAD", "/usr/lib/x86_64-linux-gnu/libasan.so.5")])
    ld = launch.LaunchDescription()

    pkg_name = "mrs_pcl_tools"
    pkg_share_path = get_package_share_directory(pkg_name)

    ld.add_action(launch.actions.DeclareLaunchArgument("use_sim_time", default_value="false"))

    ld.add_action(launch.actions.DeclareLaunchArgument("debug", default_value="false"))
    dbg_sub = None
    if sys.stdout.isatty():
        dbg_sub = launch.substitutions.PythonExpression([
            '"" if "false" == "',
            launch.substitutions.LaunchConfiguration("debug"),
            '" else "debug_ros2launch ' + os.ttyname(sys.stdout.fileno()) + '"'
        ])

    UAV_NAME = os.getenv('UAV_NAME', 'uav1')

    config_files = [
        pkg_share_path + '/config/pcl_filter.yaml',
        pkg_share_path + '/config/pcl_filtration_config.yaml',
    ]

    namespace = UAV_NAME
    ld.add_action(
        ComposableNodeContainer(
            namespace='',
            name=namespace + '_mrs_pcl_tools',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package=pkg_name,
                    plugin='mrs_pcl_tools::Testing',
                    namespace=namespace,
                    name='mrs_pcl_tools',
                    parameters=[
                        {
                            'config_files': config_files
                        },
                        {
                            "use_sim_time": launch.substitutions.LaunchConfiguration("use_sim_time")
                        },
                    ],
                    remappings=[],
                ),
            ],
            output='screen',
        ))

    return ld
