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
/// \brief Declaration of the PointCloudMapsManager method.

#ifndef EASYNAV_POINTCLOUD_MAPS_MANAGER_HPP_
#define EASYNAV_POINTCLOUD_MAPS_MANAGER_HPP_

#include <vector>
#include <stdexcept>
#include <algorithm>
#include <utility>
#include <fstream>
#include <sstream>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "easynav_core/MapsManagerBase.hpp"
#include "easynav_pointcloud_common/PointCloudData.hpp"

#include "yaets/tracing.hpp"

namespace easynav
{

/**
 * @class PointCloudMapsManager
 * @brief A plugin-based map manager using the PointCloudData structure.
 *
 * This manager implements a minimal mapping approach using Point Cloud Data
 * (PointCloudData) for both static and dynamic maps. It supports publishing and
 * receiving ROS Poitn Cloud 2 messages.
 */
class PointCloudMapsManager : public easynav::MapsManagerBase
{
public:
  /**
   * @brief Default constructor.
   */
  PointCloudMapsManager();

  /**
   * @brief Destructor.
   */
  ~PointCloudMapsManager();

  /**
   * @brief Initializes the maps manager.
   *
   * Creates necessary publishers/subscribers and initializes the map instances.
   *
   * @return std::expected<void, std::string> Success or error string.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Updates the internal maps using the current navigation state.
   *
   * Intended to be called periodically. May perform dynamic map updates
   * based on new sensor data or internal state.
   *
   * @param nav_state Current state of the navigation system.
   */
  virtual void update(NavState & nav_state) override;

  /**
   * @brief Replaces the current static map.
   *
   * @param new_map Shared pointer to a new map object. Must be of type PointCloudData.
   */
  void set_static_map(const PointCloudData & new_map);

  /**
   * @brief Replaces the current dynamic map.
   *
   * @param new_map Shared pointer to a new map object. Must be of type PointCloudData.
   */
  void set_dynamic_map(const PointCloudData & new_map);

protected:
  /**
   * @brief Full path to the map file.
   */
  std::string map_path_;

private:
  /**
   * @brief Internal static map.
   */
  PointCloudData static_map_;

  /**
   * @brief Internal dynamic map.
   */
  PointCloudData dynamic_map_;

  /**
   * @brief Publisher for the static Point Cloud.
   */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_map_pub_;

  /**
   * @brief Publisher for the dynamic Point Cloud.
   */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_map_pub_;

  /**
   * @brief Subscriber for external incoming static map updates.
   */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr incoming_map_sub_;

  /**
   * @brief Service for saving current map to disk.
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr savemap_srv_;

  /**
   * @brief Cached Point Cloud 2 message for the static map.
   */
  sensor_msgs::msg::PointCloud2 static_map_msg_;

  /**
   * @brief Cached Point Cloud 2 message for the dynamic map.
   */
  sensor_msgs::msg::PointCloud2 dynamic_map_msg_;

  /**
   * @brief Buffer storing transformations between frames.
   */
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  /**
   * @brief Transform listener that populates the tf buffer.
   */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  pcl::PointCloud<pcl::PointXYZ> fused_perception_;

  std::shared_ptr<yaets::TraceSession> session_;
};

}  // namespace easynav

#endif  // EASYNAV_POINTCLOUD_MAPS_MANAGER_HPP_
