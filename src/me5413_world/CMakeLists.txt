cmake_minimum_required(VERSION 3.8)
project(me5413_world)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz_common_vendor REQUIRED)
find_package(gz-common REQUIRED)
find_package(gz_physics_vendor REQUIRED)
find_package(gz-physics REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

add_executable(goal_publisher_node src/goal_publisher_node.cpp)
target_include_directories(goal_publisher_node PRIVATE
  include
)
target_link_libraries(goal_publisher_node
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_eigen::tf2_eigen
  tf2_geometry_msgs::tf2_geometry_msgs
  ${std_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

add_library(object_spawner_gz_plugin SHARED src/object_spawner_gz_plugin.cpp)
target_include_directories(object_spawner_gz_plugin PRIVATE
  include
)
target_link_libraries(object_spawner_gz_plugin
  rclcpp::rclcpp
  gz-sim::gz-sim
  gz-math::gz-math
  gz-transport::gz-transport
  gz-common::gz-common
  gz-physics::gz-physics
  gz-msgs::gz-msgs
  ${ros_gz_interfaces_TARGETS}
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  worlds
  models
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  object_spawner_gz_plugin
  DESTINATION share/${PROJECT_NAME}/plugins
)

install(TARGETS
  goal_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
