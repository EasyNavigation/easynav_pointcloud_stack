// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.
/// \file
/// \brief Implementation of the PointcloudMapsBuilderNode class.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_pointcloud_maps_builder/PointcloudMapsBuilderNode.hpp"
#include "easynav_common/types/Perceptions.hpp"

namespace easynav
{

PointcloudMapsBuilderNode::PointcloudMapsBuilderNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("pointcloud_maps_builder_node", options)
{
  cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  if (!has_parameter("sensor_topic")) {
    declare_parameter("sensor_topic", "points");
  }

  if (!has_parameter("downsample_resolution")) {
    declare_parameter("downsample_resolution", 1.0);
  }

  if (!has_parameter("perception_default_frame")) {
    declare_parameter("perception_default_frame", "map");
  }

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "map_builder_pointcloud/cloud_filtered", rclcpp::QoS(1).transient_local().reliable());
}

PointcloudMapsBuilderNode::~PointcloudMapsBuilderNode()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  }
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
PointcloudMapsBuilderNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  get_parameter("sensor_topic", sensor_topic_);
  get_parameter("downsample_resolution", downsample_resolution_);
  get_parameter("perception_default_frame", perception_default_frame_);

  auto perception_entry = std::make_shared<Perception>();
  perception_entry->data.points.clear();
  perception_entry->data.clear();
  perception_entry->frame_id = "";
  perception_entry->stamp = now();
  perception_entry->valid = false;
  perception_entry->new_data = true;

  perceptions_.push_back(perception_entry);

  perception_entry->subscription = create_typed_subscription<sensor_msgs::msg::PointCloud2>(
        *this, sensor_topic_, perception_entry, cbg_);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PointcloudMapsBuilderNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  pub_->on_activate();
  return CallbackReturnT::SUCCESS;
}
CallbackReturnT
PointcloudMapsBuilderNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  pub_->on_deactivate();
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PointcloudMapsBuilderNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;

  pub_.reset();

  return CallbackReturnT::SUCCESS;
}

void PointcloudMapsBuilderNode::cycle()
{
  // Finish cycle if no new perceptions
  if (std::none_of(perceptions_.begin(), perceptions_.end(),
    [](const auto & perception)
    {return perception && perception->new_data;}))
  {
    return;
  }

  if (pub_->get_subscription_count() > 0) {
    auto processed_perceptions = PerceptionsOpsView(perceptions_);
    // Fuse perceptions if the frame_id is different from default and downsample
    if (!perceptions_.empty() && perceptions_[0] &&
      perceptions_[0]->frame_id != perception_default_frame_)
    {
      processed_perceptions.downsample(downsample_resolution_).fuse(perception_default_frame_);
    } else {
      processed_perceptions.downsample(downsample_resolution_);
    }

    auto downsampled_points = processed_perceptions.as_points();
    if (downsampled_points.empty()) {
      return;
    }

    auto msg = points_to_rosmsg(downsampled_points);
    msg.header.frame_id = perceptions_[0]->frame_id;

    if (perceptions_[0]->stamp.nanoseconds() != 0) {
      msg.header.stamp = perceptions_[0]->stamp;
    } else {
      msg.header.stamp = now();
    }

    pub_->publish(msg);

    // Mark perceptions as not new after published
    for (auto & perception : perceptions_) {
      if (perception->new_data) {
        perception->new_data = false;
      }
    }
  }
}
} // namespace easynav
