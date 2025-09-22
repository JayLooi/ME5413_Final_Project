/* object_spawner_gz_plugin.cpp

 * Copyright (C) 2024 nuslde, SS47816

 * Gazebo Plugin for spawning objects
 
**/

#include "me5413_world/object_spawner_gz_plugin.hpp"

#include <functional>
#include <chrono>

using namespace std::chrono_literals;

namespace gazebo
{

const int NUM_BOX_TYPES = 4;
const int MIN_X_COORD = 2.0;
const int MIN_Y_COORD = 11.0;
const int MAX_X_COORD = 22.0;
const int MAX_Y_COORD = 19.0;
const int Z_COORD = 3.0;

ObjectSpawner::ObjectSpawner() {}

ObjectSpawner::~ObjectSpawner() {}
  
void ObjectSpawner::Configure(const gz::sim::Entity &_entity,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              gz::sim::EntityComponentManager &_ecm,
                              gz::sim::EventManager &/*_eventMgr*/)
{
  if (!rclcpp::ok())
  {
    rclcpp::init(0, nullptr);
  }

  ros_node_ = std::make_shared<rclcpp::Node>("object_spawner");

  clt_delete_objects_ = ros_node_->create_client<ros_gz_interfaces::srv::DeleteEntity>("/gazebo/delete_model");
  this->timer_ = ros_node_->create_timer(100ms, std::bind(&ObjectSpawner::timerCallback, this));

  auto respawn_cb = std::bind(&ObjectSpawner::respawnCmdCallback, this, std::placeholders::_1);
  this->sub_respawn_objects_ = ros_node_->create_subscription<std_msgs::msg::Int16>("/rviz_panel/respawn_objects", 1, respawn_cb);

  auto open_bridge_cb = std::bind(&ObjectSpawner::openBridgeCallback, this, std::placeholders::_1);
  this->sub_cmd_open_bridge_ = ros_node_->create_subscription<std_msgs::msg::Bool>("/cmd_open_bridge", 1, open_bridge_cb);

  this->pub_rviz_markers_ = ros_node_->create_publisher<visualization_msgs::msg::MarkerArray>("/gazebo/ground_truth/box_markers", 0);

  bridge_open_called_ = false;

  ros_thread_ = std::thread([this]() { rclcpp::spin(ros_node_); });
}

void ObjectSpawner::PreUpdate(const gz::sim::UpdateInfo &_info,
                              gz::sim::EntityComponentManager &_ecm)
{
  
}

void ObjectSpawner::timerCallback()
{
  // publish rviz markers
  this->pub_rviz_markers_->publish(this->box_markers_msg_);
};

void ObjectSpawner::spawn_object(gz::msgs::EntityFactory& req_msg, const std::string& name, uint32_t timeout)
{
  gz::msgs::Boolean rep;
  bool result;

  req_msg.set_name(name);

  bool executed = this->gz_transport_node_.Request("/world/default/create", req_msg, timeout, rep, result);

  if (executed && result)
  {
    std::cout << req_msg.name() << " spawned" << std::endl;
  }
  else
  {
    std::cout << req_msg.name() << " spawning failed" << std::endl;
  }
}

void ObjectSpawner::spawnRandomBridge()
{
  gz::msgs::EntityFactory bridge_msg;
  this->bridge_name = "bridge";
  bridge_msg.set_sdf_filename("model://bridge");

  std::srand(std::time(0));
  bridge_position_ = (static_cast<double>(std::rand()) / RAND_MAX * 0.5 + 0.25) * (MAX_X_COORD - MIN_X_COORD) + MIN_X_COORD;
  gz::msgs::Set(bridge_msg.mutable_pose(), gz::math::Pose3d(
    gz::math::Vector3d(bridge_position_, 9.0, 2.6), 
    gz::math::Quaterniond(1.57079632679, 0, 0)));
  
  spawn_object(bridge_msg, this->bridge_name, 2000);
};

void ObjectSpawner::spawnRandomBoxes()
{
  std::srand(std::time(0));
  this->box_names.clear();
  this->box_points.clear();
  this->box_markers_msg_.markers.clear();
  
  // The following two vectors should have the same size:
  std::vector<int> box_labels = {1, 2, 3, 4, 5, 6, 7, 8, 9}; // all possible box labels, between 1 and 9
  std::vector<int> box_nums = {1, 2, 3, 4, 5}; // can contain any positive number, but maek sure there's only one solution
  if (box_labels.size() < 1 || box_nums.size() < 1)
  {
    RCLCPP_ERROR(ros_node_->get_logger(), "The box_labels and box_nums should not be empty! Stoppping the spawning process");
    return;
  }

  // Randomise the number of boxes
  std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(box_nums.begin(), box_nums.end(), g);
  std::shuffle(box_labels.begin(), box_labels.end(), g);
  box_nums = std::vector<int>(box_nums.begin(), box_nums.begin() + NUM_BOX_TYPES);
  box_labels = std::vector<int>(box_labels.begin(), box_labels.begin() + NUM_BOX_TYPES);
  
  std::vector<std::vector<int>> boxes;
  for (int i = 0; i < box_nums.size(); i++)
  {
    for (int j = 0; j < box_nums[i]; j++)
    {
      boxes.push_back(std::vector<int>{box_labels[i], j});
    }
  }

  // Generate destination box points
  const double spacing = (MAX_X_COORD - MIN_X_COORD)/(box_labels.size() + 1);
  for (int i = 0; i < box_labels.size(); i++)
  {
    const gz::math::Vector3d point = gz::math::Vector3d(spacing*(i + 1) + MIN_X_COORD, 0.0, Z_COORD);
    gz::msgs::EntityFactory box_msg;
    const std::string box_name = "number" + std::to_string(box_labels[i]);
    this->box_names.push_back(box_name);
    box_msg.set_sdf_filename("model://" + box_name);
    gz::msgs::Set(box_msg.mutable_pose(), gz::math::Pose3d(point, gz::math::Quaterniond(0, 0, 0)));
    spawn_object(box_msg, box_name, 2000);
    RCLCPP_DEBUG_STREAM(ros_node_->get_logger(), "Generated " << box_name << " at " << point);
    GZ_SLEEP_MS(500);
  }

  // Generate random box points
  // visualization_msgs::MarkerArray text_markers_msg;
  for (int i = 0; i < boxes.size(); i++)
  {
    gz::math::Vector3d point;
    bool has_collision = true;
    // Check for collsions
    while (has_collision)
    {
      has_collision = false;
      point = gz::math::Vector3d(static_cast<double>(std::rand()) / RAND_MAX * (MAX_X_COORD - MIN_X_COORD) + MIN_X_COORD,
                                 static_cast<double>(std::rand()) / RAND_MAX * (MAX_Y_COORD - MIN_Y_COORD) + MIN_Y_COORD,
                                 Z_COORD);
      for (const auto& pre_point : this->box_points)
      {
        const double dist = (point - pre_point).Length();
        if (dist <= 1.2)
        {
          has_collision = true;
          break;
        }
      }
    } 

    // Add this box to the list
    this->box_points.push_back(point);
    
    // Publish gazebo model for this box
    gz::msgs::EntityFactory box_msg;
    const std::string box_name = "number" + std::to_string(boxes[i][0]) + "_" + std::to_string(boxes[i][1]);
    box_msg.set_sdf_filename("model://number" + std::to_string(boxes[i][0]));
    this->box_names.push_back(box_name);
    gz::msgs::Set(box_msg.mutable_pose(), gz::math::Pose3d(point, gz::math::Quaterniond(0, 0, 0)));
    spawn_object(box_msg, box_name, 2000);
    RCLCPP_DEBUG_STREAM(ros_node_->get_logger(), "Generated " << box_name << " at " << point);
    GZ_SLEEP_MS(500);
    // // Publish rviz marker for this box
    // visualization_msgs::Marker box_marker;
    // box_marker.header.frame_id = "world";
    // box_marker.header.stamp = ros::Time();
    // box_marker.ns = "gazebo";
    // box_marker.id = 2*i;
    // box_marker.type = visualization_msgs::Marker::CUBE;
    // box_marker.action = visualization_msgs::Marker::ADD;
    // box_marker.frame_locked = true;
    // box_marker.lifetime = ros::Duration(0.2);
    // box_marker.pose.position.x = point.X();
    // box_marker.pose.position.y = point.Y();
    // box_marker.pose.position.z = point.Z();
    // box_marker.pose.orientation.x = 0.0;
    // box_marker.pose.orientation.y = 0.0;
    // box_marker.pose.orientation.z = 0.0;
    // box_marker.pose.orientation.w = 1.0;
    // box_marker.scale.x = 0.8;
    // box_marker.scale.y = 0.8;
    // box_marker.scale.z = 0.8;
    // box_marker.color.a = 0.7;
    // box_marker.color.r = static_cast<double>(std::rand()) / RAND_MAX * 0.5 + 0.25;
    // box_marker.color.g = static_cast<double>(std::rand()) / RAND_MAX * 0.5 + 0.25;
    // box_marker.color.b = static_cast<double>(std::rand()) / RAND_MAX * 0.5 + 0.25;
    // this->box_markers_msg_.markers.emplace_back(box_marker);

    // visualization_msgs::Marker text_marker = box_marker;
    // text_marker.id = 2*i + 1;
    // text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    // text_marker.text = std::to_string(boxes[i][0]);
    // text_marker.pose.position.z += 0.5;
    // text_marker.scale.z = 0.5;
    // text_marker.color.a = 0.8;
    // text_marker.color.r = 0.0;
    // text_marker.color.g = 0.0;
    // text_marker.color.b = 0.0;
    // text_markers_msg.markers.emplace_back(text_marker);
  }

  // // merge the two marker arrays
  // this->box_markers_msg_.markers.insert(this->box_markers_msg_.markers.end(), text_markers_msg.markers.begin(), text_markers_msg.markers.end());
};

void ObjectSpawner::deleteObject(const std::string& object_name)
{
  auto request = std::make_shared<ros_gz_interfaces::srv::DeleteEntity::Request>();
  auto entity = ros_gz_interfaces::msg::Entity();
  entity.name = object_name;
  entity.type = gz::msgs::Entity::MODEL;
  request->entity = entity;
  auto result = clt_delete_objects_->async_send_request(request);
  if (result.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
  {
    RCLCPP_INFO_STREAM(ros_node_->get_logger(),  "Object: " << object_name << " successfully deleted");
  }
  else
  {
    RCLCPP_ERROR_STREAM(ros_node_->get_logger(), "Failed to delete Object: " << object_name << std::endl);
  }
};

void ObjectSpawner::deleteBridge()
{
  deleteObject(this->bridge_name);
  this->bridge_name = "";
  this->bridge_point = gz::math::Vector3d();
};

void ObjectSpawner::spawnCone()
{
  gz::msgs::EntityFactory cone_msg;
  this->cone_name = "Construction Barrel";
  cone_msg.set_sdf_filename("model://construction_barrel");

  gz::msgs::Set(cone_msg.mutable_pose(), gz::math::Pose3d(
    gz::math::Vector3d(bridge_position_ + 0.8, 7.0, 3.0), //centre of bridge
    gz::math::Quaterniond(0, 0, 0)));
  spawn_object(cone_msg, this->cone_name, 2000);
};

void ObjectSpawner::deleteCone()
{
  deleteObject(this->cone_name);
  this->cone_name = "";
};


void ObjectSpawner::deleteBoxes()
{
  this->box_markers_msg_.markers.clear();
  this->pub_rviz_markers_->publish(this->box_markers_msg_);

  for (const auto& box_name: this->box_names)
  {
    deleteObject(box_name);
  }
  this->box_names.clear();
  this->box_points.clear();
};

void ObjectSpawner::respawnCmdCallback(const std_msgs::msg::Int16::ConstSharedPtr& respawn_msg)
{
  const int cmd = respawn_msg->data;
  if (cmd == 0)
  {
    deleteBridge();
    deleteCone();
    deleteBoxes();
    RCLCPP_INFO_STREAM(ros_node_->get_logger(),  "Random Objects Cleared!");
  }
  else if (cmd == 1)
  {
    deleteCone();
    deleteBridge();
    deleteBoxes();
    spawnRandomBridge();
    spawnRandomBoxes();
    spawnCone();
    RCLCPP_INFO_STREAM(ros_node_->get_logger(),  "Random Objects Respawned!");
    bridge_open_called_ = false;
  }
  else
  {
    RCLCPP_INFO_STREAM(ros_node_->get_logger(),  "Respawn Command Not Recognized!");
  }
};

void ObjectSpawner::openBridgeCallback(const std_msgs::msg::Bool::ConstSharedPtr& open_bridge_msg)
{
  const bool open_bridge = open_bridge_msg->data;
  if (open_bridge == true)
  {
    if (bridge_open_called_ == false)
    {
      bridge_open_called_ = true;
      deleteCone();
      RCLCPP_INFO_STREAM(ros_node_->get_logger(),  "Bridge will now open for 10s");
      GZ_SLEEP_MS(10);
      spawnCone();
      RCLCPP_INFO_STREAM(ros_node_->get_logger(),  "Bridge is now closed, cannot be opened again");
    }
    else
    {
      RCLCPP_INFO_STREAM(ros_node_->get_logger(),  "Bridge has been opened before, cannot be opened again");
    }
  }
  else
  {
    RCLCPP_INFO_STREAM(ros_node_->get_logger(),  "Bridge open command is false, nothing to be done");
  }
}

} // namespace gazebo

// Register this plugin with the simulator
GZ_ADD_PLUGIN(
  gazebo::ObjectSpawner,
  gz::sim::System,
  gazebo::ObjectSpawner::ISystemConfigure,
  gazebo::ObjectSpawner::ISystemPreUpdate
);
