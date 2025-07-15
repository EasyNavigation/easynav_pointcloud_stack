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
/// \brief Implementation of the PointCloudMapsManager class.

#include <stdexcept>

#include "easynav_pointcloud_maps_manager/PointCloudMapsManager.hpp"
#include "easynav_common/types/Perceptions.hpp"
#include "easynav_common/types/PointPerception.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

namespace easynav
{

using std::placeholders::_1;

PointCloudMapsManager::PointCloudMapsManager()
{
    NavState::register_printer<PointCloudData>(
    [](const PointCloudData & map) {
      std::string ret = "PointCloud of (" +
      std::to_string(map.width()) + " x " +
      std::to_string(map.height()) + ") with resolution ";
      return ret;
    });
}

PointCloudMapsManager::~PointCloudMapsManager()
{
}

std::expected<void, std::string>
PointCloudMapsManager::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  std::string package_name, map_path_file, map_topic_in;
  node->declare_parameter(plugin_name + ".package_name", package_name);
  node->declare_parameter(plugin_name + ".map_path_file", map_path_file);
  node->declare_parameter(plugin_name + ".map_topic_in", map_topic_in);

  node->get_parameter(plugin_name + ".package_name", package_name);
  node->get_parameter(plugin_name + ".map_path_file", map_path_file);
  node->get_parameter(plugin_name + ".map_topic_in", map_topic_in);

  RCLCPP_INFO(get_node()->get_logger(),
        "PointCloud :    ..:: PARAMETERS ::..");

  if (package_name != "" && map_path_file != "") {
    std::string pkgpath;
    try {
      pkgpath = ament_index_cpp::get_package_share_directory(package_name);
      RCLCPP_INFO(get_node()->get_logger(),
        "PointCloud : Package name = %s", package_name.c_str());
      map_path_ = pkgpath + "/" + map_path_file;
      RCLCPP_INFO(get_node()->get_logger(),
        "PointCloud : Map path file = %s", map_path_.c_str());
    } catch(ament_index_cpp::PackageNotFoundError & ex) {
      return std::unexpected("Package " + package_name + " not found. Error: " + ex.what());
    }

    if (!static_map_.load_from_file(map_path_)) {
      return std::unexpected("File [" + map_path_ + "] not found");
    } else {
      RCLCPP_INFO(get_node()->get_logger(),
        "PointCloud : Map loaded from file");
      dynamic_map_.deep_copy(static_map_);
    }
  } else {
    map_path_ = "pointcloud_map.pcd";
    RCLCPP_INFO(get_node()->get_logger(),
      "PointCloud : Path Map file to save = ./%s", map_path_.c_str());
  }

  std::string topic_name;
  if (map_topic_in != "") {
    topic_name = map_topic_in;
    RCLCPP_INFO(get_node()->get_logger(),
      "PointCloud : Map topic in name = %s", topic_name.c_str());
  } else {
    topic_name = node->get_name() + std::string("/") + plugin_name + "/topic_in";
    RCLCPP_INFO(get_node()->get_logger(),
        "PointCloud : Input Topic by default");
  }
  

  static_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    node->get_name() + std::string("/") + plugin_name + "/static_map",
    rclcpp::QoS(1).transient_local().reliable());

  dynamic_map_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    node->get_name() + std::string("/") + plugin_name + "/dynamic_map", 100);

  incoming_map_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_name, rclcpp::QoS(1).transient_local().reliable(),
    [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {

      RCLCPP_INFO(get_node()->get_logger(),
      "PointCloud : topic_callback : Reading Map");
      static_map_.from_point_cloud(*msg);

      static_map_.to_point_cloud(static_map_msg_);
      static_map_msg_.header.frame_id = "map";
      static_map_msg_.header.stamp = this->get_node()->now();

      static_map_pub_->publish(static_map_msg_);
    });

  savemap_srv_ = node->create_service<std_srvs::srv::Trigger>(
    node->get_name() + std::string("/") + plugin_name + "/savemap",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      (void)request;

      if (!static_map_.save_to_file(map_path_)) {
        response->success = false;
        response->message = "PointCloud : Failed to save map to: " + map_path_;
      } else {
        static_map_.show("Data to be saved");
        response->success = true;
        response->message = "PointCloud : Map successfully saved to: " + map_path_;
      }
    });

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node,
    true);

  static_map_.to_point_cloud(static_map_msg_);
  static_map_msg_.header.frame_id = "map";
  static_map_msg_.header.stamp = this->get_node()->now();
  static_map_pub_->publish(static_map_msg_);

  dynamic_map_.to_point_cloud(dynamic_map_msg_);
  dynamic_map_msg_.header.frame_id = "map";
  dynamic_map_msg_.header.stamp = this->get_node()->now();
  dynamic_map_pub_->publish(dynamic_map_msg_);

  return {};
}

void
PointCloudMapsManager::set_static_map(const PointCloudData & new_map)
{
  static_map_ = new_map;
}

void
PointCloudMapsManager::set_dynamic_map(const PointCloudData & new_map)
{
  dynamic_map_ = new_map;
}

void
PointCloudMapsManager::update(NavState & nav_state)
{
  dynamic_map_.deep_copy(static_map_);

  if (nav_state.has("points")) {
    const auto & perceptions = nav_state.get<PointPerceptions>("points");

    auto fused = PointPerceptionsOpsView(perceptions)
      .fuse("map")->as_points();

    dynamic_map_.refresh(dynamic_map_msg_, fused);
    dynamic_map_.show("dynamic fused");
  }
  nav_state.set("map.static", static_map_);
  nav_state.set("map.dynamic", dynamic_map_);

  dynamic_map_.to_point_cloud(dynamic_map_msg_);
  dynamic_map_msg_.header.frame_id = "map";
  dynamic_map_msg_.header.stamp = get_node()->now();
  dynamic_map_pub_->publish(dynamic_map_msg_);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::PointCloudMapsManager, easynav::MapsManagerBase)
