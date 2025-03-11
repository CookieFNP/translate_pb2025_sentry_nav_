// Copyright 2025 Lihan Chen
//

#include "loam_interface/loam_interface.hpp"

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace loam_interface
{

/*
订阅LOAM生成的点云和里程计数据。
将点云和里程计数据从激光雷达的局部坐标系转换到全局坐标系（odom）。
发布转换后的点云和里程计数据。
*/
LoamInterfaceNode::LoamInterfaceNode(const rclcpp::NodeOptions & options)
: Node("loam_interface", options)
{
  /*
--构造函数初始化了ROS 2节点，并声明和获取了一系列参数：
state_estimation_topic_：LOAM生成的里程计数据主题。
registered_scan_topic_：LOAM生成的点云数据主题。
odom_frame_：全局坐标系（odom）的名称。
base_frame_：机器人的基座坐标系名称。
lidar_frame_：激光雷达的坐标系名称。

--初始化了以下内容：
tf_buffer_ 和 tf_listener_：用于处理坐标变换。
点云和里程计的发布者（pcd_pub_ 和 odom_pub_）。
点云和里程计的订阅者（pcd_sub_ 和 odom_sub_）。
  */
  this->declare_parameter<std::string>("state_estimation_topic", "");
  this->declare_parameter<std::string>("registered_scan_topic", "");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("lidar_frame", "");

  this->get_parameter("state_estimation_topic", state_estimation_topic_);
  this->get_parameter("registered_scan_topic", registered_scan_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);

  base_frame_to_lidar_initialized_ = false;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  pcd_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lidar_odometry", 5);

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    registered_scan_topic_, 5,
    std::bind(&LoamInterfaceNode::pointCloudCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    state_estimation_topic_, 5,
    std::bind(&LoamInterfaceNode::odometryCallback, this, std::placeholders::_1));
}

//处理点云数据，将点云从激光雷达的局部坐标系（lidar_odom）转换到全局坐标系（odom）。
//使用 pcl_ros::transformPointCloud 函数完成这一转换，并发布转换后的点云数据。
void LoamInterfaceNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  // NOTE: Input point cloud message is based on the `lidar_odom`
  // Here we transform it to the REAL `odom` frame
  auto out = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl_ros::transformPointCloud(odom_frame_, tf_odom_to_lidar_odom_, *msg, *out);
  pcd_pub_->publish(*out);
}

/*
处理里程计数据，将里程计从激光雷达的局部坐标系（lidar_odom）转换到全局坐标系（odom）。
首先通过TF查找从机器人基座坐标系（base_frame）到激光雷达坐标系（lidar_frame）的变换关系，
然后使用这个变换关系将里程计数据转换到全局坐标系。
初始化从基座坐标系到激光雷达坐标系的变换关系（tf_odom_to_lidar_odom_），（如果尚未初始化）。
  */
void LoamInterfaceNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  // NOTE: Input odometry message is based on the `lidar_odom`
  // Here we transform it to the `odom` frame
  if (!base_frame_to_lidar_initialized_) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, msg->header.stamp, rclcpp::Duration::from_seconds(0.5));
      tf2::Transform tf_base_frame_to_lidar;
      tf2::fromMsg(tf_stamped.transform, tf_base_frame_to_lidar);
      tf_odom_to_lidar_odom_ = tf_base_frame_to_lidar;
      base_frame_to_lidar_initialized_ = true;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      return;
    }
  }

  // Transform the odometry_msg (based lidar_odom) to the odom frame
  tf2::Transform tf_lidar_odom_to_lidar;
  tf2::fromMsg(msg->pose.pose, tf_lidar_odom_to_lidar);
  tf2::Transform tf_odom_to_lidar = tf_odom_to_lidar_odom_ * tf_lidar_odom_to_lidar;

  nav_msgs::msg::Odometry out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = odom_frame_;
  out.child_frame_id = lidar_frame_;

  const auto & origin = tf_odom_to_lidar.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(tf_odom_to_lidar.getRotation());

  odom_pub_->publish(out);
}

}  // namespace loam_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(loam_interface::LoamInterfaceNode)
