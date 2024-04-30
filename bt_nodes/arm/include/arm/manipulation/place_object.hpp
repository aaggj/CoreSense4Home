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
// See the License for the specific language governing permissions andGO2OBJECT
// limitations under the License.

#ifndef ARM_MANIPULATION__PLACE_OBJECT_HPP_
#define ARM_MANIPULATION__PLACE_OBJECT_HPP_

#include <algorithm>
#include <string>

#include "manipulation_interfaces/action/place.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "arm/manipulation/BTActionNode.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

namespace manipulation
{

class PlaceObject : public manipulation::BtActionNode<
    manipulation_interfaces::action::Place, rclcpp_cascade_lifecycle::CascadeLifecycleNode>
{
public:
  explicit PlaceObject(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;
  BT::NodeStatus on_success() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("position_to_place"),
        BT::OutputPort<std::string>("left_position")      
      });
  }
private:
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

} // namespace manipulation

#endif // ARM_MANIPULATION__PLACE_OBJECT_HPP_
