import launch
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration, PythonExpression, EnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "pcl_filter"

    this_pkg_path = get_package_share_directory(pkg_name)
    
    uav_name = os.getenv('UAV_NAME', "uav1")

    standalone = LaunchConfiguration("standalone")
    debug = LaunchConfiguration("debug")
    # custom_config = LaunchConfiguration("custom_config")
    # node_name = LaunchConfiguration("node_name")
    # name_suffix = LaunchConfiguration("name_suffix")
    topic_3d_lidar_in = LaunchConfiguration("topic_3d_lidar_in")
    topic_3d_lidar_out = LaunchConfiguration("topic_3d_lidar_out")

    # #{ uav_name 
    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=EnvironmentVariable('UAV_NAME',default_value='uav1'),
        description="The uav name used for namespacing",
    ))

    container_name = LaunchConfiguration('container_name')
    # Declare args
    declared_arguments = [
        DeclareLaunchArgument("UAV_NAME", default_value="", description="Name of UAV"),
        DeclareLaunchArgument("standalone", default_value="true"),
        DeclareLaunchArgument("debug", default_value="false"),
        DeclareLaunchArgument("custom_config", default_value=""),
        DeclareLaunchArgument("node_name", default_value="pcl_filter"),
        DeclareLaunchArgument("container_name", default_value=""),
        DeclareLaunchArgument("topic_3d_lidar_in", default_value="os_cloud_nodelet/points"),
        DeclareLaunchArgument("topic_3d_lidar_out", default_value="~points_processed"),
    ]

    ld.add_action(declared_arguments)


    # Node definition
    pcl_filter_node = ComposableNode(
        package=pkg_name,
        plugin='mrs_pcl_tools::pcl_filter',
        namespace=uav_name,
        name='pcl_filter',
        parameters=[
            {"uav_name": uav_name},
            {"use_sim_time": False},
            {"config": this_pkg_path +'/config/pcl_filter.yaml'},
        ],

        remappings=[
            ("~lidar3d_in", topic_3d_lidar_in),
            ("~rplidar_in", "rplidar/scan_raw"),
            ("~rangefinder_in", "mavros/distance_sensor/garmin"),
            ("~lidar3d_out", topic_3d_lidar_out),
            ("~lidar3d_over_max_range_out", "~points_over_max_range"),
            ("~rplidar_out", "rplidar/scan_processed"),
            ("~diagnostics_out", "~diagnostics"),
        ],
    )

    load_into_existing = LoadComposableNodes(
        target_container = container_name,
        composable_node_descriptions = [pcl_filter_node],
        condition = UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    ld.add_action(ComposableNodeContainer(
        namespace=uav_name,
        name='pcl_filter_container',
        package = 'rclcpp_components',
        executable = 'component_container_mt',
        output = 'screen',
        # arguments = ['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        composable_node_descriptions=[pcl_filter_node],
        condition = IfCondition(standalone)
    ))

    return ld