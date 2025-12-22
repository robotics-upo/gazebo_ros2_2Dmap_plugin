/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef GAZEBO_2DMAP_PLUGIN_H
#define GAZEBO_2DMAP_PLUGIN_H

#include <iostream>
#include <math.h>
#include <memory>

#include <common.h>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <octomap/octomap.h>
#include <rclcpp/rclcpp.hpp>
#include <sdf/sdf.hh>
#include <std_srvs/srv/empty.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace gazebo {

#if GAZEBO_MAJOR_VERSION >= 9
  typedef ignition::math::Vector3d vector3d;
  auto GetPhysicsPtr = std::mem_fn(&gazebo::physics::World::Physics);
#else
  typedef math::Vector3 vector3d;
  auto GetPhysicsPtr = std::mem_fn(&gazebo::physics::World::GetPhysicsEngine);
#endif

/// \brief    Octomap plugin for Gazebo.
/// \details  This plugin is dependent on ROS 2, and generates 2D occupancy maps
///           from Gazebo Classic (Gazebo 11) worlds.
class OccupancyMapFromWorld : public WorldPlugin {
 public:
  OccupancyMapFromWorld()
      : WorldPlugin(), name_("gazebo_2Dmap_plugin")
  {
    RCLCPP_INFO(rclcpp::get_logger(name_), "occupancy map plugin started");
  }
  virtual ~OccupancyMapFromWorld();



 protected:

  /// \brief Load the plugin.
  /// \param[in] _parent Pointer to the world that loaded this plugin.
  /// \param[in] _sdf SDF element that describes the plugin.
  void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

  bool worldCellIntersection(const vector3d& cell_center, const double cell_length,
                              gazebo::physics::RayShapePtr ray);
  

  void CreateOccupancyMap();

  /// \brief Compute world bounding box from all models
  /// \param[out] min_x Minimum x coordinate
  /// \param[out] max_x Maximum x coordinate
  /// \param[out] min_y Minimum y coordinate
  /// \param[out] max_y Maximum y coordinate
  /// \return true if bounding box was computed successfully
  bool ComputeWorldBounds(double& min_x, double& max_x, double& min_y, double& max_y);

  void cell2world(unsigned int cell_x, unsigned int cell_y,
                  double map_size_x, double map_size_y, double map_resolution,
                  double map_origin_x, double map_origin_y,
                  double& world_x, double &world_y);

  void world2cell(double world_x, double world_y,
                  double map_size_x, double map_size_y, double map_resolution,
                  double map_origin_x, double map_origin_y,
                  unsigned int& cell_x, unsigned int& cell_y);

  static bool cell2index(int cell_x, int cell_y,
                         unsigned int cell_size_x, unsigned int cell_size_y,
                         unsigned int& map_index);

  static bool index2cell(int index, unsigned int cell_size_x, unsigned int cell_size_y,
                         unsigned int& cell_x, unsigned int& cell_y);

 private:
  void ServiceCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    std::shared_ptr<std_srvs::srv::Empty::Response> res);

  physics::WorldPtr world_;
  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr map_service_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  nav_msgs::msg::OccupancyGrid* occupancy_map_;
  std::string name_;
  double map_resolution_;
  double map_height_;
  double map_size_x_;
  double map_size_y_;
  double init_robot_x_;
  double init_robot_y_;
  double map_margin_;  // Margin to add around auto-detected bounds
  double map_origin_x_;  // Map center X (0.0 for manual, computed for auto)
  double map_origin_y_;  // Map center Y (0.0 for manual, computed for auto)
};

} // namespace gazebo

#endif  // GAZEBO_2DMAP_PLUGIN_H
