# Copyright 2025 Lihan Chen
'
''
---导航和SLAM系统启动：
集成了nav2导航堆栈和slam_toolbox，支持地图保存、点云处理、激光雷达扫描等功能。
支持通过参数动态配置启动行为（如是否使用仿真时间、是否自动启动导航堆栈等）。

---模块化设计：
使用Node启动各个功能模块，包括地图保存服务器、生命周期管理器、点云到激光雷达转换、SLAM工具箱等。
支持通过参数化配置动态调整节点行为。

---参数化配置：
提供了丰富的启动参数（如namespace、params_file、use_sim_time等），允许用户根据需求灵活配置。
参数通过LaunchConfiguration动态获取，并通过RewrittenYaml动态生成配置文件。

---仿真时间支持：
支持通过use_sim_time参数决定是否使用仿真时间（通常用于Gazebo仿真环境）。

---日志管理：
支持通过log_level参数动态设置日志级别。
支持节点崩溃后的自动重启（通过use_respawn参数）。

---坐标变换发布：
使用tf2_ros发布静态变换，例如从map到odom的变换。

'''

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Getting directories and launch-files
    bringup_dir = get_package_share_directory("pb2025_nav_bringup")

    # Input parameters declaration
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    # Variables
    lifecycle_nodes = ["map_saver"]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {"use_sim_time": use_sim_time}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="True",
        description="Automatically startup the nav2 stack",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        "log_level", default_value="info", description="log level"
    )

    start_map_saver_server_cmd = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
    )

    start_lifecycle_manager_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    start_pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("cloud_in", "terrain_map"),
            ("scan", "obstacle_scan"),
        ],
    )

    start_sync_slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=["--ros-args", "--log-level", log_level],
        remappings=[
            ("/map", "map"),
            ("/map_metadata", "map_metadata"),
            ("/map_updates", "map_updates"),
        ],
    )

    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        name="point_lio",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[
            configured_params,
            {"prior_pcd.enable": False},
            {"pcd_save.pcd_save_en": True},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    start_static_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_map2odom",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Running Map Saver Server
    ld.add_action(start_map_saver_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    ld.add_action(start_pointcloud_to_laserscan_node)
    ld.add_action(start_sync_slam_toolbox_node)
    ld.add_action(start_point_lio_node)
    ld.add_action(start_static_transform_node)

    return ld
