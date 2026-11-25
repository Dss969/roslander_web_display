// Copyright 2026 ROSLander contributors
// Licensed under the Apache License, Version 2.0.

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "roslander_bt_nodes/is_goal_nearby_condition.hpp"

namespace
{
class RosEnvironment : public testing::Environment
{
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};
const auto ros_environment = testing::AddGlobalTestEnvironment(new RosEnvironment);

class NearbyTest : public testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("goal_nearby_unit_test");
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    // Tests populate static transforms synchronously; no TF subscriber is needed.
    tf_->setUsingDedicatedThread(true);
    bb_ = BT::Blackboard::create();
    bb_->set("node", node_);
    bb_->set("tf_buffer", tf_);
    bb_->set("threshold", 0.3);
    bb_->set("search", 1.5);
    bb_->set("path", nav_msgs::msg::Path{});
    bb_->set("goal", geometry_msgs::msg::PoseStamped{});
    bb_->set("goals", std::vector<geometry_msgs::msg::PoseStamped>{});
    factory_.registerNodeType<roslander_bt_nodes::IsGoalNearbyCondition>("IsGoalNearby");
    pose(0.0, 0.0);
  }

  nav_msgs::msg::Path line(double start = 0.0, double end = 1.0)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (int i = 0; i <= 10; ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = start + (end - start) * i / 10.0;
      p.pose.orientation.w = 1.0;
      path.poses.push_back(p);
    }
    return path;
  }

  void pose(double x, double y)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node_->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_footprint";
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf_->setTransform(t, "test", true));
  }

  BT::NodeStatus tick()
  {
    if (!tree_) {
      tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(R"(
        <root main_tree_to_execute="Test"><BehaviorTree ID="Test">
          <IsGoalNearby path="{path}" proximity_threshold="{threshold}"
            max_robot_pose_search_dist="{search}" global_frame="map"
            robot_base_frame="base_footprint" transform_tolerance="0.0"
            remaining_path="{remaining}" distance_remaining="{distance}"/>
        </BehaviorTree></root>)", bb_));
    }
    return tree_->tickRoot();
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  BT::Blackboard::Ptr bb_;
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
};

TEST_F(NearbyTest, EmptyPathFailsAndClearsOutputs)
{
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(bb_->get<nav_msgs::msg::Path>("remaining").poses.empty());
  EXPECT_TRUE(std::isinf(bb_->get<double>("distance")));
}

TEST_F(NearbyTest, MeasuresRemainingPathAndRemovesTraversedPoints)
{
  bb_->set("path", line());
  pose(0.8, 0.0);
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
  EXPECT_NEAR(bb_->get<double>("distance"), 0.2, 1e-9);
  const auto remaining = bb_->get<nav_msgs::msg::Path>("remaining");
  ASSERT_EQ(remaining.poses.size(), 3u);
  EXPECT_NEAR(remaining.poses.front().pose.position.x, 0.8, 1e-9);
}

TEST_F(NearbyTest, FarAlongPathStillRequiresPlanning)
{
  bb_->set("path", line());
  pose(0.2, 0.0);
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_NEAR(bb_->get<double>("distance"), 0.8, 1e-9);
}

TEST_F(NearbyTest, LoopDoesNotJumpToNearbyFutureEndpoint)
{
  auto path = line();
  for (const auto & xy : std::vector<std::pair<double, double>>{
      {1.0, 1.0}, {0.0, 1.0}, {0.0, 0.05}})
  {
    auto p = path.poses.back();
    p.pose.position.x = xy.first;
    p.pose.position.y = xy.second;
    path.poses.push_back(p);
  }
  bb_->set("path", path);
  bb_->set("search", 0.5);
  pose(0.0, 0.05);
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_GT(bb_->get<double>("distance"), 3.0);
}

TEST_F(NearbyTest, ChangedPathResetsSearchCursor)
{
  bb_->set("path", line());
  pose(0.9, 0.0);
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
  auto path = line();
  path.header.stamp.sec = 1;
  bb_->set("path", path);
  pose(0.1, 0.0);
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_NEAR(bb_->get<double>("distance"), 0.9, 1e-9);
}

TEST_F(NearbyTest, TransformsRobotIntoPathFrame)
{
  auto path = line();
  path.header.frame_id = "odom";
  for (auto & p : path.poses) {p.header.frame_id = "odom";}
  bb_->set("path", path);
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "odom";
  t.transform.translation.x = 10.0;
  t.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_->setTransform(t, "test", true));
  pose(10.8, 0.0);
  EXPECT_EQ(tick(), BT::NodeStatus::SUCCESS);
  EXPECT_NEAR(bb_->get<double>("distance"), 0.2, 1e-9);
}

TEST_F(NearbyTest, MissingTransformDoesNotSuppressPlanning)
{
  auto path = line();
  path.header.frame_id = "missing_frame";
  bb_->set("path", path);
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(bb_->get<nav_msgs::msg::Path>("remaining").poses.empty());
}

TEST_F(NearbyTest, FarOffPathEndpointIsNotNearby)
{
  bb_->set("path", line());
  pose(2.0, 0.0);
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  EXPECT_NEAR(bb_->get<double>("distance"), 0.0, 1e-9);
}

TEST_F(NearbyTest, InvalidInputsDoNotSuppressPlanning)
{
  auto path = line();
  path.poses.back().pose.position.x = std::numeric_limits<double>::quiet_NaN();
  bb_->set("path", path);
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  bb_->set("path", line());
  bb_->set("threshold", 0.0);
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
  bb_->set("threshold", 0.3);
  bb_->set("search", 0.0);
  EXPECT_EQ(tick(), BT::NodeStatus::FAILURE);
}

TEST_F(NearbyTest, PluginCanBeLoadedWithOptionalOutputsUnconnected)
{
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(
    BT::SharedLibrary::getOSName("roslander_is_goal_nearby_condition_bt_node"));
  bb_->set("path", line());
  pose(0.9, 0.0);
  auto tree = factory.createTreeFromText(R"(
    <root main_tree_to_execute="Test"><BehaviorTree ID="Test">
      <IsGoalNearby path="{path}" robot_base_frame="base_footprint"/>
    </BehaviorTree></root>)", bb_);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
}

TEST_F(NearbyTest, PolicyKeepsValidNearPathButReplansOnGoalChangeInvalidPathAndDistance)
{
  factory_.registerFromPlugin(
    BT::SharedLibrary::getOSName("nav2_globally_updated_goal_condition_bt_node"));
  int planning_calls = 0;
  bool path_valid = true;
  factory_.registerSimpleCondition("PathIsValid", [&path_valid](BT::TreeNode &) {
      return path_valid ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  factory_.registerSimpleAction("ComputePath", [&planning_calls](BT::TreeNode &) {
      ++planning_calls;
      return BT::NodeStatus::SUCCESS;
    });
  bb_->set("path", line());
  pose(0.9, 0.0);
  auto tree = factory_.createTreeFromText(R"(
    <root main_tree_to_execute="Test"><BehaviorTree ID="Test">
      <Fallback>
        <ReactiveSequence>
          <Inverter><GlobalUpdatedGoal/></Inverter>
          <IsGoalNearby path="{path}" proximity_threshold="0.3"
            robot_base_frame="base_footprint" remaining_path="{remaining}"/>
          <PathIsValid/>
        </ReactiveSequence>
        <ComputePath/>
      </Fallback>
    </BehaviorTree></root>)", bb_);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(planning_calls, 1);  // First tick always plans, even with an old nearby path.
  for (int i = 0; i < 4; ++i) {tree.tickRoot();}
  EXPECT_EQ(planning_calls, 1);
  auto goal = geometry_msgs::msg::PoseStamped{};
  goal.pose.position.x = 2.0;
  bb_->set("goal", goal);
  tree.tickRoot();
  EXPECT_EQ(planning_calls, 2);
  path_valid = false;
  tree.tickRoot();
  EXPECT_EQ(planning_calls, 3);
  path_valid = true;
  pose(0.5, 0.0);
  tree.tickRoot();
  EXPECT_EQ(planning_calls, 4);
}

TEST_F(NearbyTest, ControllerCopyUsesLatestTFWithoutMutatingPlannerPath)
{
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(
    BT::SharedLibrary::getOSName("roslander_is_goal_nearby_condition_bt_node"));
  auto original = line();
  original.header.stamp.sec = 20;
  for (auto & p : original.poses) {p.header.stamp.sec = 20;}
  bb_->set("path", original);
  auto tree = factory.createTreeFromText(R"(
    <root main_tree_to_execute="Test"><BehaviorTree ID="Test">
      <UseLatestPathTF input_path="{path}" output_path="{controller_path}"/>
    </BehaviorTree></root>)", bb_);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  const auto result = bb_->get<nav_msgs::msg::Path>("controller_path");
  ASSERT_EQ(result.poses.size(), original.poses.size());
  EXPECT_EQ(result.header.stamp.sec, 0);
  for (std::size_t i = 0; i < result.poses.size(); ++i) {
    EXPECT_EQ(result.poses[i].header.stamp.sec, 0);
    EXPECT_EQ(result.poses[i].pose, original.poses[i].pose);
  }
  EXPECT_EQ(bb_->get<nav_msgs::msg::Path>("path"), original);
  tree.tickRoot();
  EXPECT_EQ(bb_->get<nav_msgs::msg::Path>("controller_path"), result);
}

TEST_F(NearbyTest, LatestTFRejectsEmptyOrMovingFramePaths)
{
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(
    BT::SharedLibrary::getOSName("roslander_is_goal_nearby_condition_bt_node"));
  auto tree = factory.createTreeFromText(R"(
    <root main_tree_to_execute="Test"><BehaviorTree ID="Test">
      <UseLatestPathTF input_path="{path}" output_path="{controller_path}"/>
    </BehaviorTree></root>)", bb_);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  auto path = line();
  path.header.frame_id = "base_footprint";
  bb_->set("path", path);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
  path = line();
  path.poses.back().header.frame_id = "odom";
  bb_->set("path", path);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::FAILURE);
}

TEST_F(NearbyTest, RetainedGoalUsesCurrentLocalizationCorrectionInsteadOfPlanningTime)
{
  // Reproduce a 15 cm map->odom correction after a path was planned. A stamped
  // endpoint and an unstamped DWB endpoint otherwise describe different goals.
  geometry_msgs::msg::TransformStamped correction;
  correction.header.frame_id = "map";
  correction.child_frame_id = "odom";
  correction.transform.rotation.w = 1.0;
  correction.header.stamp.sec = 1;
  ASSERT_TRUE(tf_->setTransform(correction, "test", false));
  correction.header.stamp.sec = 2;
  correction.transform.translation.x = 0.15;
  ASSERT_TRUE(tf_->setTransform(correction, "test", false));

  auto path = line();
  path.header.stamp.sec = 1;
  for (auto & p : path.poses) {p.header.stamp.sec = 1;}
  const auto old_goal = tf_->transform(path.poses.back(), "odom");

  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(
    BT::SharedLibrary::getOSName("roslander_is_goal_nearby_condition_bt_node"));
  bb_->set("path", path);
  auto tree = factory.createTreeFromText(R"(
    <root main_tree_to_execute="Test"><BehaviorTree ID="Test">
      <UseLatestPathTF input_path="{path}" output_path="{controller_path}"/>
    </BehaviorTree></root>)", bb_);
  ASSERT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);
  const auto current_goal = tf_->transform(
    bb_->get<nav_msgs::msg::Path>("controller_path").poses.back(), "odom");
  EXPECT_NEAR(old_goal.pose.position.x, 1.0, 1e-9);
  EXPECT_NEAR(current_goal.pose.position.x, 0.85, 1e-9);
  // Robot at odom x=0.87 is 2 cm from the current goal but 13 cm from the old one.
  EXPECT_GT(std::abs(old_goal.pose.position.x - 0.87), 0.08);
  EXPECT_LT(std::abs(current_goal.pose.position.x - 0.87), 0.08);
}
}  // namespace
