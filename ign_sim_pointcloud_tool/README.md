# isaac_sim_pointcloud_tool

此包将 Ignition Gazebo 中的 pointcloud 转换为 velodyne 格式。这是一个 ROS 包。它订阅了 Ignition Gazebo 发布的 LiDAR rostopic。它以 Velodyne 格式重新发布了 LiDAR rostopoic。

一些 SLAM 算法需要 Velodyne 格式的点云，以便提取角点。但是 Isaac ROS 只发送包含 XYZ 信息的 pointcloud。此软件包有助于将 pointcloud 转换为 velodyne 格式。
