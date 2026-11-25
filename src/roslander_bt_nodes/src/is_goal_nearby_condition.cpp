// Copyright 2026 ROSLander contributors
// Licensed under the Apache License, Version 2.0.
// References: navigation2 IsGoalNearbyCondition (Apache-2.0), upstream PR #5876.

#include "roslander_bt_nodes/is_goal_nearby_condition.hpp"

#include <cmath>
#include <limits>
#include <utility>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"

namespace roslander_bt_nodes
{

IsGoalNearbyCondition::IsGoalNearbyCondition(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::ConditionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
}

BT::PortsList IsGoalNearbyCondition::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "Current global path"),
    BT::InputPort<double>("proximity_threshold", 0.3, "Near-goal path distance in metres"),
    BT::InputPort<double>(
      "max_robot_pose_search_dist", 1.5,
      "Forward path distance searched from the previous closest point; negative searches all"),
    BT::InputPort<std::string>("global_frame", "map", "Reference frame for robot pose"),
    BT::InputPort<std::string>("robot_base_frame", "base_link", "Robot frame"),
    BT::InputPort<double>("transform_tolerance", 0.1, "TF timeout in seconds"),
    BT::OutputPort<nav_msgs::msg::Path>(
      "remaining_path", "Path from the closest point through the goal, for IsPathValid"),
    BT::OutputPort<double>("distance_remaining", "Remaining integrated path length in metres")
  };
}

void IsGoalNearbyCondition::clearOutputs()
{
  if (config().output_ports.count("remaining_path")) {
    setOutput("remaining_path", nav_msgs::msg::Path{});
  }
  if (config().output_ports.count("distance_remaining")) {
    setOutput("distance_remaining", std::numeric_limits<double>::infinity());
  }
}

BT::NodeStatus IsGoalNearbyCondition::tick()
{
  clearOutputs();
  nav_msgs::msg::Path incoming;
  double threshold, search_distance, tf_timeout;
  std::string global_frame, base_frame;
  if (!getInput("path", incoming) ||
    !getInput("proximity_threshold", threshold) ||
    !getInput("max_robot_pose_search_dist", search_distance) ||
    !getInput("transform_tolerance", tf_timeout) ||
    !getInput("global_frame", global_frame) ||
    !getInput("robot_base_frame", base_frame) ||
    incoming.poses.empty() || incoming.header.frame_id.empty() ||
    global_frame.empty() || base_frame.empty() ||
    !std::isfinite(threshold) || threshold <= 0.0 ||
    !std::isfinite(search_distance) || search_distance == 0.0 ||
    !std::isfinite(tf_timeout) || tf_timeout < 0.0)
  {
    was_near_ = false;
    return BT::NodeStatus::FAILURE;
  }
  for (const auto & pose : incoming.poses) {
    if (!std::isfinite(pose.pose.position.x) || !std::isfinite(pose.pose.position.y)) {
      was_near_ = false;
      return BT::NodeStatus::FAILURE;
    }
  }

  const bool bounded_search = search_distance > 0.0;
  if (incoming != path_ || !bounded_search) {
    path_ = std::move(incoming);
    closest_index_ = 0;
  }

  geometry_msgs::msg::PoseStamped global_pose, robot_pose;
  if (!nav2_util::getCurrentPose(
      global_pose, *tf_buffer_, global_frame, base_frame, tf_timeout) ||
    !nav2_util::transformPoseInTargetFrame(
      global_pose, robot_pose, *tf_buffer_, path_.header.frame_id, tf_timeout))
  {
    was_near_ = false;
    return BT::NodeStatus::FAILURE;
  }

  auto begin = path_.poses.begin() + closest_index_;
  auto end = path_.poses.end();
  if (bounded_search) {
    end = nav2_util::geometry_utils::first_after_integrated_distance(
      begin, end, search_distance);
  }
  const auto closest = nav2_util::geometry_utils::min_by(
    begin, end, [&robot_pose](const auto & pose) {
      return nav2_util::geometry_utils::euclidean_distance(robot_pose, pose);
    });
  closest_index_ = static_cast<std::size_t>(closest - path_.poses.begin());
  const double remaining = nav2_util::geometry_utils::calculate_path_length(path_, closest_index_);
  const double distance_to_path = nav2_util::geometry_utils::euclidean_distance(robot_pose, *closest);

  if (config().output_ports.count("remaining_path")) {
    nav_msgs::msg::Path remaining_path;
    remaining_path.header = path_.header;
    remaining_path.poses.assign(closest, path_.poses.end());
    setOutput("remaining_path", remaining_path);
  }
  if (config().output_ports.count("distance_remaining")) {
    setOutput("distance_remaining", remaining);
  }

  // Upstream checks integrated path length. Also require the robot to be near
  // that path, so a localization jump far beyond its end cannot suppress planning.
  const bool near = remaining < threshold && distance_to_path < threshold;
  if (near && !was_near_) {
    RCLCPP_INFO(
      node_->get_logger(), "IsGoalNearby: remaining=%.3f m, distance_to_path=%.3f m",
      remaining, distance_to_path);
  }
  was_near_ = near;
  return near ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace roslander_bt_nodes

namespace roslander_bt_nodes
{
// A map-frame global path is spatial geometry, not an old sensor observation.
// Humble ControllerServer transforms its stamped endpoint at the planning time,
// while DWB transforms its unstamped 2D endpoint at the latest TF. Explicitly
// use latest TF for the controller copy, keeping the planner/validation path intact.
class UseLatestPathTF : public BT::SyncActionNode
{
public:
  UseLatestPathTF(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path", "Fixed global-frame path"),
      BT::InputPort<std::string>("global_frame", "map", "Fixed global frame"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path", "Controller path using latest TF")
    };
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path path;
    std::string global_frame;
    if (!getInput("input_path", path) || !getInput("global_frame", global_frame) ||
      path.poses.empty() || global_frame.empty() || path.header.frame_id != global_frame)
    {
      setOutput("output_path", nav_msgs::msg::Path{});
      return BT::NodeStatus::FAILURE;
    }
    path.header.stamp = builtin_interfaces::msg::Time{};
    for (auto & pose : path.poses) {
      if (!pose.header.frame_id.empty() && pose.header.frame_id != global_frame) {
        setOutput("output_path", nav_msgs::msg::Path{});
        return BT::NodeStatus::FAILURE;
      }
      pose.header.frame_id = global_frame;
      pose.header.stamp = builtin_interfaces::msg::Time{};
    }
    setOutput("output_path", path);
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace roslander_bt_nodes

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<roslander_bt_nodes::IsGoalNearbyCondition>("IsGoalNearby");
  factory.registerNodeType<roslander_bt_nodes::UseLatestPathTF>("UseLatestPathTF");
}
