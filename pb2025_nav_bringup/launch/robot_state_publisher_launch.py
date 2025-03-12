# Copyright 2025 Lihan Chen

'''
---机器人描述启动：
该脚本用于启动机器人描述模块，包括发布机器人状态（robot_state_publisher）和关节状态（joint_state_publisher）。
这些功能通常用于在ROS环境中描述机器人的几何和运动学信息。

---模块化设计：
使用IncludeLaunchDescription嵌套其他启动脚本（如robot_description_launch.py），实现模块化启动。
支持通过参数动态配置启动行为。

---参数化配置：
提供了多个启动参数（如namespace、use_sim_time、robot_name等），允许用户根据需求灵活配置。
参数通过LaunchConfiguration动态获取。

---命名空间和重映射：
使用PushRosNamespace为所有启动的节点设置命名空间。
使用SetRemap重映射/tf和/tf_static话题，以适应机器人系统的需求。

'''

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap


def generate_launch_description():
    # Get the launch directory
    pkg_pb2025_robot_description_dir = get_package_share_directory(
        "pb2025_robot_description"
    )

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_name = LaunchConfiguration("robot_name")

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name",
        default_value="pb2025_sentry_robot",
        description="The file name of the robot xmacro to be used",
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace=namespace),
            SetRemap("/tf", "tf"),
            SetRemap("/tf_static", "tf_static"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        pkg_pb2025_robot_description_dir,
                        "launch",
                        "robot_description_launch.py",
                    )
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "robot_name": robot_name,
                    "use_rviz": "False",
                }.items(),
            ),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_robot_name_cmd)

    # Add the actions to launch all nodes
    ld.add_action(bringup_cmd_group)

    return ld
