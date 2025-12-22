/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#include "gazebo_2Dmap_plugin.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <gazebo/common/Time.hh>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo_ros/node.hpp>


namespace gazebo {

OccupancyMapFromWorld::~OccupancyMapFromWorld() {}

void OccupancyMapFromWorld::Load(physics::WorldPtr _parent,
                                 sdf::ElementPtr _sdf) {
  world_ = _parent;

  // Initialize ROS 2 node
  ros_node_ = gazebo_ros::Node::Get(_sdf);

  map_pub_ = ros_node_->create_publisher<nav_msgs::msg::OccupancyGrid>("map2d", 
    rclcpp::QoS(1).transient_local());
  
  map_service_ = ros_node_->create_service<std_srvs::srv::Empty>(
    "gazebo_2Dmap_plugin/generate_map",
    std::bind(&OccupancyMapFromWorld::ServiceCallback, this, 
              std::placeholders::_1, std::placeholders::_2));

  map_resolution_ = 0.05;  // Default resolution

  if(_sdf->HasElement("map_resolution"))
    map_resolution_ = _sdf->GetElement("map_resolution")->Get<double>();

  map_height_ = 0.2;  // Default height of the 2D map slice

  if(_sdf->HasElement("map_height"))
    map_height_ = _sdf->GetElement("map_height")->Get<double>();

  init_robot_x_ = 0.0;

  if(_sdf->HasElement("init_robot_x"))
    init_robot_x_ = _sdf->GetElement("init_robot_x")->Get<double>();

  init_robot_y_ = 0.0;

  if(_sdf->HasElement("init_robot_y"))
    init_robot_y_ = _sdf->GetElement("init_robot_y")->Get<double>();

  map_margin_ = 2.0;  // Default margin around auto-detected bounds

  if(_sdf->HasElement("map_margin"))
    map_margin_ = _sdf->GetElement("map_margin")->Get<double>();

  // Check if map sizes are manually specified
  bool map_size_x_specified = _sdf->HasElement("map_size_x");
  bool map_size_y_specified = _sdf->HasElement("map_size_y");

  map_size_x_ = 10.0;  // Default fallback
  map_size_y_ = 10.0;  // Default fallback
  map_origin_x_ = 0.0;  // Default: centered at world origin
  map_origin_y_ = 0.0;

  if(map_size_x_specified)
    map_size_x_ = _sdf->GetElement("map_size_x")->Get<double>();

  if(map_size_y_specified)
    map_size_y_ = _sdf->GetElement("map_size_y")->Get<double>();

  // If map sizes are not specified, try to compute them automatically
  if(!map_size_x_specified || !map_size_y_specified)
  {
    double min_x, max_x, min_y, max_y;
    if(ComputeWorldBounds(min_x, max_x, min_y, max_y))
    {
      if(!map_size_x_specified)
      {
        // Map is centered at world origin (0,0), so compute size to fit all bounds
        // Take the maximum absolute extent in X direction and double it (plus margin)
        double max_extent_x = std::max(std::abs(min_x), std::abs(max_x));
        map_size_x_ = 2.0 * max_extent_x + 2.0 * map_margin_;
        map_origin_x_ = 0.0;  // Always centered at world origin
        RCLCPP_INFO(ros_node_->get_logger(), 
          "Auto-detected map_size_x: %.2f (world bounds: %.2f to %.2f, centered at origin: 0.0, margin: %.2f)", 
          map_size_x_, min_x, max_x, map_margin_);
      }
      
      if(!map_size_y_specified)
      {
        // Map is centered at world origin (0,0), so compute size to fit all bounds
        // Take the maximum absolute extent in Y direction and double it (plus margin)
        double max_extent_y = std::max(std::abs(min_y), std::abs(max_y));
        map_size_y_ = 2.0 * max_extent_y + 2.0 * map_margin_;
        map_origin_y_ = 0.0;  // Always centered at world origin
        RCLCPP_INFO(ros_node_->get_logger(), 
          "Auto-detected map_size_y: %.2f (world bounds: %.2f to %.2f, centered at origin: 0.0, margin: %.2f)", 
          map_size_y_, min_y, max_y, map_margin_);
      }
    }
    else
    {
      RCLCPP_WARN(ros_node_->get_logger(), 
        "Failed to auto-detect world bounds. Using default values: map_size_x=%.1f, map_size_y=%.1f",
        map_size_x_, map_size_y_);
    }
  }

  RCLCPP_INFO(ros_node_->get_logger(), 
    "Map configuration - Resolution: %.2f, Height: %.2f, Size: %.1fx%.1f, Centered at world origin: (%.2f, %.2f), Init pos: (%.1f, %.1f)",
    map_resolution_, map_height_, map_size_x_, map_size_y_, map_origin_x_, map_origin_y_, init_robot_x_, init_robot_y_);
}

bool OccupancyMapFromWorld::ComputeWorldBounds(double& min_x, double& max_x, 
                                                double& min_y, double& max_y)
{
  if(!world_)
  {
    RCLCPP_ERROR(ros_node_->get_logger(), "World pointer is null");
    return false;
  }

  // Get all models in the world
#if GAZEBO_MAJOR_VERSION >= 8
  auto models = world_->Models();
#else
  auto models = world_->GetModels();
#endif

  if(models.empty())
  {
    RCLCPP_WARN(ros_node_->get_logger(), "No models found in world");
    return false;
  }

  bool bounds_initialized = false;
  min_x = min_y = 0.0;
  max_x = max_y = 0.0;

  int model_count = 0;
  
  // Iterate through all models and compute bounding box
  for(const auto& model : models)
  {
    if(!model) continue;
    
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::AxisAlignedBox bbox = model->BoundingBox();
    
    // Skip invalid bounding boxes
    if(!bbox.Min().IsFinite() || !bbox.Max().IsFinite())
    {
      RCLCPP_DEBUG(ros_node_->get_logger(), "Skipping model '%s' with invalid bounding box", 
                   model->GetName().c_str());
      continue;
    }
    
    ignition::math::Vector3d bbox_min = bbox.Min();
    ignition::math::Vector3d bbox_max = bbox.Max();
    
    double model_min_x = bbox_min.X();
    double model_max_x = bbox_max.X();
    double model_min_y = bbox_min.Y();
    double model_max_y = bbox_max.Y();
#else
    gazebo::math::Box bbox = model->GetBoundingBox();
    gazebo::math::Vector3 bbox_min = bbox.min;
    gazebo::math::Vector3 bbox_max = bbox.max;
    
    // Skip invalid bounding boxes
    if(!std::isfinite(bbox_min.x) || !std::isfinite(bbox_max.x))
    {
      RCLCPP_DEBUG(ros_node_->get_logger(), "Skipping model '%s' with invalid bounding box", 
                   model->GetName().c_str());
      continue;
    }
    
    double model_min_x = bbox_min.x;
    double model_max_x = bbox_max.x;
    double model_min_y = bbox_min.y;
    double model_max_y = bbox_max.y;
#endif

    if(!bounds_initialized)
    {
      min_x = model_min_x;
      max_x = model_max_x;
      min_y = model_min_y;
      max_y = model_max_y;
      bounds_initialized = true;
    }
    else
    {
      min_x = std::min(min_x, model_min_x);
      max_x = std::max(max_x, model_max_x);
      min_y = std::min(min_y, model_min_y);
      max_y = std::max(max_y, model_max_y);
    }
    
    model_count++;
    RCLCPP_DEBUG(ros_node_->get_logger(), "Model '%s': X[%.2f, %.2f], Y[%.2f, %.2f]",
                 model->GetName().c_str(), model_min_x, model_max_x, model_min_y, model_max_y);
  }

  if(model_count > 0 && bounds_initialized)
  {
    RCLCPP_DEBUG(ros_node_->get_logger(), 
      "Computed world bounds from %d models: X[%.2f, %.2f], Y[%.2f, %.2f]",
      model_count, min_x, max_x, min_y, max_y);
    return true;
  }

  RCLCPP_WARN(ros_node_->get_logger(), 
    "No valid bounding boxes found from %d models", static_cast<int>(models.size()));
  return false;
}

void OccupancyMapFromWorld::ServiceCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> res)
{
  (void)req;
  (void)res;
  CreateOccupancyMap();
}


bool OccupancyMapFromWorld::worldCellIntersection(const vector3d& cell_center,
                                                const double cell_length,
                                                gazebo::physics::RayShapePtr ray)

{
  //check for collisions with rays surrounding the cell
  //    ---
  //   | + |
  //    ---

  double dist;
  std::string entity_name;

  int cell_length_steps = 10;
  double side_length;

  //check for collisions with beams at increasing sizes to capture smaller
  //objects inside the cell
  for(int step=1; step<=cell_length_steps; step++)
  {
    side_length = cell_length / cell_length_steps * step;

    for(int i=-1; i<2; i+=2)
    {
#if GAZEBO_MAJOR_VERSION >= 9
      double start_x = cell_center.X() + i * side_length/2;
      double start_y = cell_center.Y() - i * side_length/2;

      for(int j=-1; j<2; j+=2)
      {
        double end_x = cell_center.X() + j * side_length/2;
        double end_y = cell_center.Y() + j * side_length/2;

        ray->SetPoints(vector3d(start_x, start_y, cell_center.Z()),
               vector3d(end_x, end_y, cell_center.Z()));
#else
        double start_x = cell_center.x + i * side_length/2;
        double start_y = cell_center.y - i * side_length/2;

      for(int j=-1; j<2; j+=2)
      {
        double end_x = cell_center.x + j * side_length/2;
        double end_y = cell_center.y + j * side_length/2;

        ray->SetPoints(vector3d(start_x, start_y, cell_center.z),
                       vector3d(end_x, end_y, cell_center.z));
#endif
        ray->GetIntersection(dist, entity_name);

        if(!entity_name.empty())
          return true;
      }
    }
  }

  return false;
}

void OccupancyMapFromWorld::cell2world(unsigned int cell_x, unsigned int cell_y,
                                       double map_size_x, double map_size_y,
                                       double map_resolution,
                                       double map_origin_x, double map_origin_y,
                                       double& world_x, double &world_y)
{
  // Convert cell coordinates to world coordinates
  // Cell (0,0) corresponds to the bottom-left corner of the map
  world_x = map_origin_x - map_size_x/2 + cell_x * map_resolution + map_resolution/2;
  world_y = map_origin_y - map_size_y/2 + cell_y * map_resolution + map_resolution/2;
}

void OccupancyMapFromWorld::world2cell(double world_x, double world_y,
                                       double map_size_x, double map_size_y,
                                       double map_resolution,
                                       double map_origin_x, double map_origin_y,
                                       unsigned int& cell_x, unsigned int& cell_y)
{
  // Convert world coordinates to cell coordinates
  cell_x = (world_x - map_origin_x + map_size_x/2) / map_resolution;
  cell_y = (world_y - map_origin_y + map_size_y/2) / map_resolution;
}

bool OccupancyMapFromWorld::cell2index(int cell_x, int cell_y,
                                       unsigned int cell_size_x, unsigned int cell_size_y,
                                       unsigned int& map_index)
{
  if(cell_x >= 0 && static_cast<unsigned int>(cell_x) < cell_size_x && 
     cell_y >= 0 && static_cast<unsigned int>(cell_y) < cell_size_y)
  {
    map_index = cell_y * cell_size_x + cell_x;
    return true;
  }
  else
  {
    //return false when outside map bounds
    return false;
  }
}

bool OccupancyMapFromWorld::index2cell(int index, unsigned int cell_size_x,
                                       unsigned int cell_size_y,
                                       unsigned int& cell_x, unsigned int& cell_y)
{
  cell_y = index / cell_size_x;
  cell_x = index % cell_size_x;

  if(cell_x < cell_size_x && cell_y < cell_size_y)
    return true;
  else
  {
    //return false when outside map bounds
    return false;
  }
}

void OccupancyMapFromWorld::CreateOccupancyMap()
{
  // Use computed map origin (0,0 for manual specification, actual world center for auto-detection)
  vector3d map_origin(map_origin_x_, map_origin_y_, map_height_);

  unsigned int cells_size_x = map_size_x_ / map_resolution_;
  unsigned int cells_size_y = map_size_y_ / map_resolution_;

  occupancy_map_ = new nav_msgs::msg::OccupancyGrid();
  occupancy_map_->data.resize(cells_size_x * cells_size_y);
  //all cells are initially unknown
  std::fill(occupancy_map_->data.begin(), occupancy_map_->data.end(), -1);
  occupancy_map_->header.stamp = ros_node_->now();
  occupancy_map_->header.frame_id = "odom"; //TODO map frame
  occupancy_map_->info.map_load_time = rclcpp::Time(0);
  occupancy_map_->info.resolution = map_resolution_;
  occupancy_map_->info.width = cells_size_x;
  occupancy_map_->info.height = cells_size_y;
#if GAZEBO_MAJOR_VERSION >= 9
  occupancy_map_->info.origin.position.x = map_origin.X() - map_size_x_ / 2;
  occupancy_map_->info.origin.position.y = map_origin.Y() - map_size_y_ / 2;
  occupancy_map_->info.origin.position.z = map_origin.Z();
#else
  occupancy_map_->info.origin.position.x = map_origin.x - map_size_x_ / 2;
  occupancy_map_->info.origin.position.y = map_origin.y - map_size_y_ / 2;
  occupancy_map_->info.origin.position.z = map_origin.z;
#endif
  occupancy_map_->info.origin.orientation.w = 1;

  gazebo::physics::PhysicsEnginePtr engine = GetPhysicsPtr(world_);
  engine->InitForThread();
  gazebo::physics::RayShapePtr ray =
      boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

  std::cout << "Starting wavefront expansion for mapping" << std::endl;

  //identify free space by spreading out from initial robot cell
  double robot_x = init_robot_x_;
  double robot_y = init_robot_y_;

  //find initial robot cell
  unsigned int cell_x, cell_y, map_index;
  world2cell(robot_x, robot_y, map_size_x_, map_size_y_, map_resolution_,
             map_origin_x_, map_origin_y_, cell_x, cell_y);

  if(!cell2index(cell_x, cell_y, cells_size_x, cells_size_y, map_index))
  {
    RCLCPP_ERROR(ros_node_->get_logger(), 
      "initial robot pos (%.2f, %.2f) is outside map bounds (origin: %.2f, %.2f, size: %.2fx%.2f), could not create map",
      robot_x, robot_y, map_origin_x_, map_origin_y_, map_size_x_, map_size_y_);
    return;
  }

  std::vector<unsigned int> wavefront;
  wavefront.push_back(map_index);

  //wavefront expansion for identifying free, unknown and occupied cells
  while(!wavefront.empty())
  {
    map_index = wavefront.at(0);
    wavefront.erase(wavefront.begin());

    index2cell(map_index, cells_size_x, cells_size_y, cell_x, cell_y);

    //mark cell as free
    occupancy_map_->data.at(map_index) = 0;

    //explore cells neighbors in an 8-connected grid
    unsigned int child_index;
    double world_x, world_y;
    uint8_t child_val;

    //8-connected grid
    for(int i=-1; i<2; i++)
    {
      for(int j=-1; j<2; j++)
      {
        //makes sure index is inside map bounds
        if(cell2index(cell_x + i, cell_y + j, cells_size_x, cells_size_y, child_index))
        {
          child_val = occupancy_map_->data.at(child_index);

          //only update value if cell is unknown
          if(child_val != 100 && child_val != 0 && child_val != 50)
          {
            cell2world(cell_x + i, cell_y + j, map_size_x_, map_size_y_, map_resolution_,
                       map_origin_x_, map_origin_y_, world_x, world_y);

            bool cell_occupied = worldCellIntersection(vector3d(world_x, world_y, map_height_),
                                                       map_resolution_, ray);

            if(cell_occupied)
              //mark cell as occupied
              occupancy_map_->data.at(child_index) = 100;


            else
            {
              //add cell to wavefront
              wavefront.push_back(child_index);
              //mark wavefront in map so we don't add children to wavefront multiple
              //times
              occupancy_map_->data.at(child_index) = 50;
            }
          }
        }
      }
    }
  }

  map_pub_->publish(*occupancy_map_);
  
  RCLCPP_INFO(ros_node_->get_logger(), 
    "Map published on /map2d topic - Size: %dx%d, Occupied cells: %ld, Free cells: %ld",
    occupancy_map_->info.width, occupancy_map_->info.height,
    std::count(occupancy_map_->data.begin(), occupancy_map_->data.end(), 100),
    std::count(occupancy_map_->data.begin(), occupancy_map_->data.end(), 0));
  
  std::cout << "\rOccupancy Map generation completed                  " << std::endl;
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(OccupancyMapFromWorld)

}  // namespace gazebo
