/* goal_publisher_node.cpp

 * Copyright (C) 2023 SS47816

 * ROS Node for publishing goal poses 
 
**/

#include "me5413_world/goal_publisher_node.hpp"

#include <functional>
#include <chrono>

using namespace std::chrono_literals;

namespace me5413_world 
{

GoalPublisherNode::GoalPublisherNode()
  : Node("goal_publisher_node"), tf2_buffer_(tf2_ros::Buffer(clock_)), tf2_listener_(tf2_buffer_), tf2_bcaster_(*this)
{
  this->pub_goal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);
  this->pub_absolute_position_error_ = this->create_publisher<std_msgs::msg::Float32>("/me5413_world/absolute/position_error", 1);
  this->pub_absolute_heading_error_ = this->create_publisher<std_msgs::msg::Float32>("/me5413_world/absolute/heading_error", 1);
  this->pub_relative_position_error_ = this->create_publisher<std_msgs::msg::Float32>("/me5413_world/relative/position_error", 1);
  this->pub_relative_heading_error_ = this->create_publisher<std_msgs::msg::Float32>("/me5413_world/relative/heading_error", 1);

  this->timer_ = this->create_timer(200ms, std::bind(&GoalPublisherNode::timerCallback, this));

  auto odom_cb = std::bind(&GoalPublisherNode::robotOdomCallback, this, std::placeholders::_1);
  this->sub_robot_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("/gazebo/ground_truth/state", 1, odom_cb);

  auto goal_name_cb = std::bind(&GoalPublisherNode::goalNameCallback, this, std::placeholders::_1);
  this->sub_goal_name_ = this->create_subscription<std_msgs::msg::String>("/rviz_panel/goal_name", 1, goal_name_cb);

  auto goal_cb = std::bind(&GoalPublisherNode::goalPoseCallback, this, std::placeholders::_1);
  this->sub_goal_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1, goal_cb);

  auto box_marker_cb = std::bind(&GoalPublisherNode::boxMarkersCallback, this, std::placeholders::_1);
  this->sub_box_markers_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("/gazebo/ground_truth/box_markers", 1, box_marker_cb);
  
  // Initialization
  this->robot_frame_ = "base_link";
  this->map_frame_ = "map";
  this->world_frame_ = "world";
  this->absolute_position_error_.data = 0.0;
  this->absolute_heading_error_.data = 0.0;
  this->relative_position_error_.data = 0.0;
  this->relative_heading_error_.data = 0.0;
};

void GoalPublisherNode::timerCallback()
{
  // Calculate absolute errors (wrt to world frame)
  const std::pair<double, double> error_absolute = calculatePoseError(this->pose_world_robot_, this->pose_world_goal_);
  // Calculate relative errors (wrt to map frame)
  const std::pair<double, double> error_relative = calculatePoseError(this->pose_map_robot_, this->pose_map_goal_);
  
  this->absolute_position_error_.data = error_absolute.first;
  this->absolute_heading_error_.data = error_absolute.second;
  this->relative_position_error_.data = error_relative.first;
  this->relative_heading_error_.data = error_relative.second;

  if (this->goal_type_ == "box")
  {
    this->absolute_heading_error_.data = 0.0;
    this->relative_heading_error_.data = 0.0;
  }

  // Publish errors
  this->pub_absolute_position_error_->publish(this->absolute_position_error_);
  this->pub_absolute_heading_error_->publish(this->absolute_heading_error_);
  this->pub_relative_position_error_->publish(this->relative_position_error_);
  this->pub_relative_heading_error_->publish(this->relative_heading_error_);

  return;
};

void GoalPublisherNode::robotOdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& odom)
{
  this->world_frame_ = odom->header.frame_id;
  this->robot_frame_ = odom->child_frame_id;
  this->pose_world_robot_ = odom->pose.pose;

  const tf2::Transform T_world_robot = convertPoseToTransform(this->pose_world_robot_);
  const tf2::Transform T_robot_world = T_world_robot.inverse();

  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = this->robot_frame_;
  transformStamped.child_frame_id = this->world_frame_;
  transformStamped.transform.translation.x = T_robot_world.getOrigin().getX();
  transformStamped.transform.translation.y = T_robot_world.getOrigin().getY();
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation.x = T_robot_world.getRotation().getX();
  transformStamped.transform.rotation.y = T_robot_world.getRotation().getY();
  transformStamped.transform.rotation.z = T_robot_world.getRotation().getZ();
  transformStamped.transform.rotation.w = T_robot_world.getRotation().getW();
  
  this->tf2_bcaster_.sendTransform(transformStamped);

  return;
};

void GoalPublisherNode::goalNameCallback(const std_msgs::msg::String::ConstSharedPtr& name)
{ 
  const std::string goal_name = name->data;
  const int end = goal_name.find_last_of("_");
  this->goal_type_ = goal_name.substr(1, end-1);
  const unsigned long goal_box_id = std::stoul(goal_name.substr(end+1, 1));

  geometry_msgs::msg::PoseStamped P_world_goal;
  if (this->goal_type_ == "box")
  {
    if (box_poses_.empty())
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Box poses unknown, please spawn boxes first!");
      return;
    }
    else if (goal_box_id >= box_poses_.size())
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Box id is outside the available range, please select a smaller id!");
      return;
    }
    
    P_world_goal = box_poses_[goal_box_id - 1];
  }
  else
  {
    // Get the Pose of the goal in world frame
    P_world_goal = getGoalPoseFromConfig(goal_name);
  }

  this->pose_world_goal_ = P_world_goal.pose;
  // Get the Transform from world to map from the tf_listener
  geometry_msgs::msg::TransformStamped transform_map_world;
  try
  {
    transform_map_world = this->tf2_buffer_.lookupTransform(this->map_frame_, this->world_frame_, rclcpp::Time(0));
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  // Transform the goal pose to map frame
  geometry_msgs::msg::PoseStamped P_map_goal;
  tf2::doTransform(P_world_goal, P_map_goal, transform_map_world);
  P_map_goal.header.stamp = this->now();
  P_map_goal.header.frame_id = map_frame_;

  // Transform the robot pose to map frame
  tf2::doTransform(this->pose_world_robot_, this->pose_map_robot_, transform_map_world);

  // Publish goal pose in map frame 
  if (this->goal_type_ != "box")
  {
    this->pub_goal_->publish(P_map_goal);
  }

  return;
};

void GoalPublisherNode::goalPoseCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& goal_pose)
{
  this->pose_map_goal_ = goal_pose->pose;
}

tf2::Transform GoalPublisherNode::convertPoseToTransform(const geometry_msgs::msg::Pose& pose)
{
  tf2::Transform T;
  T.setOrigin(tf2::Vector3(pose.position.x, pose.position.y, 0));
  tf2::Quaternion q;
  q.setValue(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  T.setRotation(q);

  return T;
};

void GoalPublisherNode::boxMarkersCallback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr& box_markers)
{
  this->box_poses_.clear();
  for (const auto& box : box_markers->markers)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose = box.pose;
    this->box_poses_.emplace_back(pose);
  }

  return;
};

geometry_msgs::msg::PoseStamped GoalPublisherNode::getGoalPoseFromConfig(const std::string& name)
{
  /** 
   * Get the Transform from goal to world from the file
   */

  double x, y, yaw;
  this->get_parameter("/me5413_world" + name + "/x", x);
  this->get_parameter("/me5413_world" + name + "/y", y);
  this->get_parameter("/me5413_world" + name + "/yaw", yaw);
  this->get_parameter("/me5413_world/frame_id", this->world_frame_);

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  q.normalize();

  geometry_msgs::msg::PoseStamped P_world_goal;
  P_world_goal.pose.position.x = x;
  P_world_goal.pose.position.y = y;
  P_world_goal.pose.orientation = tf2::toMsg(q);

  return P_world_goal;
};

std::pair<double, double> GoalPublisherNode::calculatePoseError(const geometry_msgs::msg::Pose& pose_robot, const geometry_msgs::msg::Pose& pose_goal)
{
  // Positional Error
  const double position_error = std::sqrt(
    std::pow(pose_robot.position.x - pose_goal.position.x, 2) + 
    std::pow(pose_robot.position.y - pose_goal.position.y, 2)
  );

  // Heading Error
  tf2::Quaternion q_robot, q_goal;
  tf2::fromMsg(pose_robot.orientation, q_robot);
  tf2::fromMsg(pose_goal.orientation, q_goal);
  const tf2::Matrix3x3 m_robot = tf2::Matrix3x3(q_robot);
  const tf2::Matrix3x3 m_goal = tf2::Matrix3x3(q_goal);

  double roll, pitch, yaw_robot, yaw_goal;
  m_robot.getRPY(roll, pitch, yaw_robot);
  m_goal.getRPY(roll, pitch, yaw_goal);

  const double heading_error = (yaw_robot - yaw_goal)/M_PI*180.0;

  return std::pair<double, double>(position_error, heading_error);
}

} // namespace me5413_world

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<me5413_world::GoalPublisherNode>());  // spin the ros node.
  return 0;
}
