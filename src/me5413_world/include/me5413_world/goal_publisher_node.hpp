/* goal_publisher_node.hpp

 * Copyright (C) 2023 SS47816

 * Declarations for GoalPublisherNode class
 
**/

#ifndef GOAL_PUBLISHER_NODE_H_
#define GOAL_PUBLISHER_NODE_H_

#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/convert.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace me5413_world 
{

class GoalPublisherNode : public rclcpp::Node
{
 public:
  GoalPublisherNode();
  virtual ~GoalPublisherNode() {};

 private:
  void timerCallback();
  void robotOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom);
  void goalNameCallback(const std_msgs::msg::String::ConstSharedPtr& name);
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& goal_pose);
  void boxMarkersCallback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr& box_markers);
  
  tf2::Transform convertPoseToTransform(const geometry_msgs::msg::Pose& pose);
  geometry_msgs::msg::PoseStamped getGoalPoseFromConfig(const std::string& name);
  std::pair<double, double> calculatePoseError(const geometry_msgs::msg::Pose& pose_robot, const geometry_msgs::msg::Pose& pose_goal);

  // ROS declaration
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_absolute_position_error_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_absolute_heading_error_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_relative_position_error_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_relative_heading_error_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_robot_odom_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_goal_name_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_box_markers_;

  rclcpp::Clock::SharedPtr clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_bcaster_;

  // Robot pose
  std::string world_frame_;
  std::string map_frame_;
  std::string robot_frame_;
  std::string goal_type_;

  geometry_msgs::msg::Pose pose_world_robot_;
  geometry_msgs::msg::Pose pose_world_goal_;
  geometry_msgs::msg::Pose pose_map_robot_;
  geometry_msgs::msg::Pose pose_map_goal_;
  std::vector<geometry_msgs::msg::PoseStamped> box_poses_;

  std_msgs::msg::Float32 absolute_position_error_;
  std_msgs::msg::Float32 absolute_heading_error_;
  std_msgs::msg::Float32 relative_position_error_;
  std_msgs::msg::Float32 relative_heading_error_;
};

} // namespace me5413_world

#endif // GOAL_PUBLISHER_NODE_H_
