// Copyright 2024 Intelligent Robotics Lab - Gentlebots
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <utility>

#include "arm/manipulation/place_object.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "manipulation_interfaces/action/place.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

namespace manipulation
{

using namespace std::chrono_literals;
using namespace std::placeholders;

PlaceObject::PlaceObject(
  const std::string & xml_tag_name, const std::string & action_name,
  const BT::NodeConfiguration & conf)
: manipulation::BtActionNode<
    manipulation_interfaces::action::Place, rclcpp_cascade_lifecycle::CascadeLifecycleNode>(
    xml_tag_name, action_name, conf), tf_buffer_(), tf_listener_(tf_buffer_) {}

void PlaceObject::on_tick()
{

  RCLCPP_DEBUG(node_->get_logger(), "PlaceObject ticked");
  // moveit_msgs::msg::CollisionObject::SharedPtr object_;
  // getInput("object_to_place", object_);
  // goal_.object_goal = *object_;
  std::string position_to_place;
  getInput("position_to_place", position_to_place);

  geometry_msgs::msg::PoseStamped target_pose;
  geometry_msgs::msg::TransformStamped map_to_goal;
  try {
    map_to_goal = tf_buffer_.lookupTransform(
      "map", position_to_place,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      node_->get_logger(), "Could not transform %s to %s: %s",
      "map", position_to_place, ex.what());
    throw BT::RuntimeError("Could not transform");
  }
  target_pose.header.frame_id = "map";
  target_pose.pose.position.x = map_to_goal.transform.translation.x;
  target_pose.pose.position.y = map_to_goal.transform.translation.y;

  RCLCPP_INFO(
    node_->get_logger(), "Sending goal to place object x: %f, y: %f, "
    "in frame %s", target_pose.pose.position.x, target_pose.pose.position.y,
    target_pose.header.frame_id.c_str());

  auto goal_msg = manipulation_interfaces::action::Place::Goal();
  goal_msg.place_pose = target_pose;
}

BT::NodeStatus 
PlaceObject::on_success() 
{
  return BT::NodeStatus::SUCCESS;
}

} // namespace manipulation
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  BT::NodeBuilder builder = [](const std::string & name,
      const BT::NodeConfiguration & config) {
      return std::make_unique<manipulation::PlaceObject>(name, "/place", config);
    };

  factory.registerBuilder<manipulation::PlaceObject>("PlaceObject", builder);
}
