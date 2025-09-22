/* object_spawner_gz_plugin.cpp

 * Copyright (C) 2024 nuslde, SS47816

 * Gazebo Plugin for spawning objects
 
**/

#include <ctime>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <algorithm>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "gz/math/Vector3.hh"
#include "gz/math/Pose3.hh"
#include "gz/transport/Node.hh"
#include "gz/msgs.hh"
#include "gz/common/Util.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/System.hh"
// #include "gz/sim/Util.hh"
// #include "gz/sim/World.hh"
#include "gz/plugin/Register.hh"
#include "ros_gz_interfaces/srv/delete_entity.hpp"

namespace gazebo
{
class ObjectSpawner :
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
 public:
  std::string bridge_name;
  std::string cone_name;
  gz::math::Vector3d bridge_point; //@shuo is this one still needed?
  std::vector<std::string> box_names;
  std::vector<gz::math::Vector3d> box_points;

  ObjectSpawner();
  virtual ~ObjectSpawner();
  virtual void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &/*_eventMgr*/) override;
  virtual void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm) override;

 private:
  gz::transport::Node gz_transport_node_;
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<ros_gz_interfaces::srv::DeleteEntity>::SharedPtr clt_delete_objects_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_respawn_objects_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_cmd_open_bridge_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_rviz_markers_;
  visualization_msgs::msg::MarkerArray box_markers_msg_;
  std::thread ros_thread_;

  bool bridge_open_called_;
  double bridge_position_;
  
  void timerCallback();
  void spawn_object(gz::msgs::EntityFactory& req_msg, const std::string& name, uint32_t timeout);
  void spawnRandomBridge();
  void spawnRandomBoxes();
  void deleteObject(const std::string& object_name);
  void deleteBridge();
  void deleteCone();
  void spawnCone();
  void deleteBoxes();
  void respawnCmdCallback(const std_msgs::msg::Int16::ConstSharedPtr& respawn_msg);
  void openBridgeCallback(const std_msgs::msg::Bool::ConstSharedPtr& open_bridge_msg);
};

} // namespace gazebo
