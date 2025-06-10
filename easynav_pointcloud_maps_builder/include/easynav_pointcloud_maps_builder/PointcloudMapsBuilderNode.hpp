// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in sh0rt)
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
/// \brief Definition of the PointcloudMapsBuilderNode class.

#ifndef EASYNAV_OUTDOOR_MAPS_BUILDER__POINTCLOUDMAPSBUILDERNODE_HPP_
#define EASYNAV_OUTDOOR_MAPS_BUILDER__POINTCLOUDMAPSBUILDERNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "easynav_common/types/Perceptions.hpp"

namespace easynav
{

/**
 * @class PointcloudMapsBuilderNode
 * @brief Lifecycle node that subscribes to point cloud sensor data and manages point cloud map building.
 *
 * This node processes perception data (point clouds) to build and update point cloud maps.
 * It leverages ROS 2 lifecycle for clean startup, activation, deactivation, and cleanup phases.
 * The node publishes processed (filtered or downsampled) point cloud maps for downstream use.
 */
class PointcloudMapsBuilderNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(PointcloudMapsBuilderNode)

  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructor.
   * @param options Node initialization options.
   */
  explicit PointcloudMapsBuilderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor.
   */
  ~PointcloudMapsBuilderNode();

  /**
   * @brief Lifecycle configure callback.
   * @param state Current lifecycle state.
   * @return CallbackReturnT indicating success or failure.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle activate callback.
   * @param state Current lifecycle state.
   * @return CallbackReturnT indicating success or failure.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle deactivate callback.
   * @param state Current lifecycle state.
   * @return CallbackReturnT indicating success or failure.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Lifecycle cleanup callback.
   * @param state Current lifecycle state.
   * @return CallbackReturnT indicating success or failure.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Perform a processing cycle on perception data and update the point cloud map.
   *
   * This should be called periodically (e.g., in a timer or main loop) to process incoming sensor data,
   * update internal map representations, and publish outputs.
   */
  void cycle();

private:
  /// Sensor topic name to subscribe to point clouds.
  std::string sensor_topic_;

  /// Perception data container.
  Perceptions perceptions_;

  /// Callback group for subscription and timers.
  rclcpp::CallbackGroup::SharedPtr cbg_;

  /// Downsampling resolution for point clouds.
  double downsample_resolution_;

  /// Default frame ID for perception and publishing.
  std::string perception_default_frame_;

  /// Publisher for processed point cloud map.
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace easynav

#endif  // EASYNAV_OUTDOOR_MAPS_BUILDER__POINTCLOUDMAPSBUILDERNODE_HPP_
