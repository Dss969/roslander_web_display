// Copyright 2026 ROSLander contributors
// Licensed under the Apache License, Version 2.0.
// Humble adaptation of the IsGoalNearby concept in ros-navigation/navigation2.

#ifndef ROSLANDER_BT_NODES__IS_GOAL_NEARBY_CONDITION_HPP_
#define ROSLANDER_BT_NODES__IS_GOAL_NEARBY_CONDITION_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace roslander_bt_nodes
{

class IsGoalNearbyCondition : public BT::ConditionNode
{
public:
  IsGoalNearbyCondition(const std::string & name, const BT::NodeConfiguration & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  void clearOutputs();

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  nav_msgs::msg::Path path_;
  std::size_t closest_index_{0};
  bool was_near_{false};
};

}  // namespace roslander_bt_nodes
#endif  // ROSLANDER_BT_NODES__IS_GOAL_NEARBY_CONDITION_HPP_
