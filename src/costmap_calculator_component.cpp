// Copyright (c) 2022 OUXT Polaris
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

#include <chrono>
#include <data_buffer/data_buffer_base.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <robotx_costmap_calculator/costmap_calculator_component.hpp>
#include <string>
#include <vector>

namespace robotx_costmap_calculator
{
CostmapCalculatorComponent::CostmapCalculatorComponent(const rclcpp::NodeOptions & options)
: Node("robotx_costmap_calculator", options)
{
  std::string points_raw_topic;
  std::string laserscan_raw_topic;
  std::string current_pose_topic;
  declare_parameter<std::string>("points_raw_topic", "/perception/points_concatenate_node/output");
  get_parameter("points_raw_topic", points_raw_topic);
  declare_parameter<std::string>(
    "laserscan_raw_topic", "/perception/pointcloud_to_laserscan_node/output");
  get_parameter("laserscan_raw_topic", laserscan_raw_topic);
  declare_parameter<std::string>("current_pose_topic", "current_pose");
  get_parameter("current_pose_topic", current_pose_topic);
  declare_parameter("resolution", 1.0);
  get_parameter("resolution", resolution_);
  declare_parameter("num_grids", 20);
  get_parameter("num_grids", num_grids_);
  declare_parameter("update_rate", 10.0);
  get_parameter("update_rate", update_rate_);
  declare_parameter("range_max", 20.0);
  get_parameter("range_max", range_max_);
  declare_parameter("visualize_frame_id", "map");
  get_parameter("visualize_frame_id", visualize_frame_id_);
  declare_parameter("buffer_length", 5.0);
  double buffer_length;
  get_parameter("buffer_length", buffer_length);
  int point_buffer_size_;
  declare_parameter("point_buffer_size_", 2);
  get_parameter("point_buffer_size_", point_buffer_size_);
  int scan_buffer_size_;
  declare_parameter("scan_buffer_size_", 2);
  get_parameter("scan_buffer_size_", scan_buffer_size_);
  std::string key;
  rclcpp::Clock::SharedPtr clock;
  data_buffer = std::make_shared<data_buffer::PoseStampedDataBuffer>(clock, key, buffer_length);

  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    points_raw_topic, 10,
    std::bind(&CostmapCalculatorComponent::pointCloudCallback, this, std::placeholders::_1));

  laserscan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    laserscan_raw_topic, 10,
    std::bind(&CostmapCalculatorComponent::scanCallback, this, std::placeholders::_1));

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    current_pose_topic, 10,
    std::bind(&CostmapCalculatorComponent::poseCallback, this, std::placeholders::_1));

  grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 1);

  //timer
  /*
  const auto period = rclcpp::Rate(update_rate_).period();
  timer_ = rclcpp::create_timer(this,get_clock(),period, std::bind(&CostmapCalculatorComponent::timerCallback, this));
  */
  cloud_buffer_=boost::circular_buffer<sensor_msgs::msg::PointCloud2>(point_buffer_size_);
  scan_buffer_=boost::circular_buffer<sensor_msgs::msg::LaserScan>(scan_buffer_size_);
  map_data_ = boost::circular_buffer<grid_map::GridMap>(2);

  initGridMap();
}

void CostmapCalculatorComponent::initGridMap()
{
  map.add("current_laser_layer", 0.0);
  map.add("prev_laser_layer", 0.0);
  map.add("prev_prev_laser_layer", 0.0);
  map.add("prev_prev_point_layer",0.0);
  map.setFrameId("base_link");
  map.setGeometry(
    grid_map::Length(resolution_ * num_grids_, resolution_ * num_grids_),
    resolution_,grid_map::Position(0.0, 0.0));
}


double sigmoid(double a, double b, double x)
{
  double ret;
  ret = 1 / (1 + std::exp(-a * (x - b)));
  return ret;
}

void CostmapCalculatorComponent::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  geometry_msgs::msg::PoseStamped query_data;
  query_data = *pose;
  //std::vector<geometry_msgs::msg::PoseStamped> data_vector;
  data_buffer->addData(query_data);
  return;
}


void CostmapCalculatorComponent::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  
  map =getScanToGridMap(scan,scan->header.stamp);
  return;
}

void CostmapCalculatorComponent::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  //UpdatePoseTransform(point_transform_pose,cloud);
  //cloud_buffer_.push_back(*cloud);
  std::string point_current_layer_name("current_point_layer");
  std::string prev_point_layer_name("prev_point_layer");
  //current_cloud =cloud_buffer[0];
  //past_cloud = cloud_buffer[1];
  //std::cout<<__FILE__<<","<<__LINE__<<std::endl;
  //map[point_current_layer_name] =getPointCloudToGridMap(cloud,cloud->header.stamp,point_current_layer_name);
  map[prev_point_layer_name] =getPointCloudToGridMap(cloud,cloud->header.stamp,prev_point_layer_name);
  //map =getPointCloudToGridMap(cloud,cloud->header.stamp);
  auto outputMessage = grid_map::GridMapRosConverter::toMessage(map);
  grid_map_pub_->publish(std::move(outputMessage));
  return;
}


grid_map::GridMap CostmapCalculatorComponent::getScanToGridMap(const sensor_msgs::msg::LaserScan::SharedPtr scan,const rclcpp::Time stamp)
{
  map.clearAll();
  scan_buffer_.push_back(*scan);
  geometry_msgs::msg::PoseStamped scan_transform_pose;
  data_buffer->queryData(stamp,interpolation_pose);
  scan_transform_pose = interpolation_pose;
  geometry_msgs::msg::PoseStamped interpolation_pose;
  for (int i = 0; i < static_cast<int>(scan_buffer_[0].ranges.size()); i++) {
    if (range_max_ >= scan_buffer_[0].ranges[i]) {
      double theta = scan_buffer_[0].angle_min + scan_buffer_[0].angle_increment * static_cast<double>(i);
      double scan_x = scan_buffer_[0].ranges[i] * std::cos(theta);
      double scan_y = scan_buffer_[0].ranges[i] * std::sin(theta);
      for (grid_map::CircleIterator iterator(map, grid_map::Position(scan_x, scan_y), 0.5);
           !iterator.isPastEnd(); ++iterator) {
        if (std::isnan(map.at("current_laser_layer", *iterator))) {
          map.at("current_laser_layer", *iterator) = 0.0;
        } else {
          if(map.at("current_laser_layer", *iterator) < 1.0){
            map.at("current_laser_layer", *iterator) = map.at("current_laser_layer", *iterator) + 0.1;
        }
      }
    }
  }
  }
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    if (std::isnan(map.at("current_laser_layer", *iterator))) {
      map.at("current_laser_layer", *iterator) = 0.0;
    }
  }
  

  /*
  rclcpp::Time end_time = system_clock.now();
  geometry_msgs::msg::PoseStamped new_pose=data_buffer->queryData(start_time,end_time,data_vector);
  grid_map::Position position;
  position.x() = position.x() + new_pose.pose.position.x;
  position_transform.y() = position.y() + new_pose.pose.position.y;
  map.setPosition(position_transform);
  */
  //map_data_.push_back(map);
  //map["laser_past_layer"] = map_data_[0].get("laser_layer");
  //map["laser_sum_layer"] = map["laser_layer"] + 0.5 * map["laser_past_layer"];
  return map;
}

grid_map::Matrix CostmapCalculatorComponent::getPointCloudToGridMap(const sensor_msgs::msg::PointCloud2::SharedPtr cloud,const rclcpp::Time stamp,const std::string & grid_map_layer_name)
{
  map.add(grid_map_layer_name,0.0);
  RCLCPP_INFO(get_logger(), "grid_map_layer_name=%s",grid_map_layer_name.c_str());
  data_buffer->queryData(stamp,interpolation_pose);
  //std::cout<<__FILE__<<","<<__LINE__<<std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud, *pcl_cloud);  
  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    map.getPosition(*iterator, position);
    double x_min = position.x() - (resolution_ * 0.5);
    double x_max = position.x() + (resolution_ * 0.5);
    double y_min = position.y() - (resolution_ * 0.5);
    double y_max = position.y() + (resolution_ * 0.5);
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_min, x_max);
    pass.filter(*cloud_filtered);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min, y_max);
    pass.filter(*cloud_filtered);
    int num_points = cloud_filtered->size();
    if (num_points == 0) {
      map.at(grid_map_layer_name, *iterator) = 0.0;
    } else {
      map.at(grid_map_layer_name, *iterator) = sigmoid(1.0, 0.0, (double)num_points);
    }

  }
  return map[grid_map_layer_name];
} 
/*
grid_map::GridMap CostmapCalculatorComponent::getPointCloudToGridMap(const sensor_msgs::msg::PointCloud2::SharedPtr cloud,const rclcpp::Time stamp)
{
  cloud_buffer_.push_back(*cloud);
  data_buffer->queryData(stamp,interpolation_pose);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud, *pcl_cloud);  
  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    map.getPosition(*iterator, position);
    double x_min = position.x() - (resolution_ * 0.5);
    double x_max = position.x() + (resolution_ * 0.5);
    double y_min = position.y() - (resolution_ * 0.5);
    double y_max = position.y() + (resolution_ * 0.5);
    pass.setInputCloud(pcl_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_min, x_max);
    pass.filter(*cloud_filtered);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min, y_max);
    pass.filter(*cloud_filtered);
    int num_points = cloud_filtered->size();
    if (num_points == 0) {
      map.at("current_point_layer", *iterator) = 0.0;
    } else {
      map.at("current_point_layer", *iterator) = sigmoid(1.0, 0.0, (double)num_points);
    }

  }
  return map;
}


/*

  map["point_past_layer"] = map_data_[0].get("point_layer");
  map["point_sum_layer"] = map["point_layer"] + 0.5 * map["point_past_layer"];
  return;
}


void CostmapCalculatorComponent::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  rclcpp::Time end_time = system_clock.now();
  geometry_msgs::msg::PoseStamped new_pose=data_buffer->queryData(start_time,end_time,data_vector);
  grid_map::Position position;
  grid_map::Position position_transform;
  position_transform.x() = position.x() + new_pose.pose.position.x;
  position_transform.y() = position.y() + new_pose.pose.position.y;
  map.setPosition(position_transform);
  //map_data_.push_back(map);
  //map["laser_past_layer"] = map_data_[0].get("laser_layer");
  //map["laser_sum_layer"] = map["laser_layer"] + 0.5 * map["laser_past_layer"];
  auto message = grid_map::GridMapRosConverter::toMessage(map);
  grid_map_pub_->publish(std::move(message));
  return;
}*/
}  // namespace robotx_costmap_calculator

RCLCPP_COMPONENTS_REGISTER_NODE(robotx_costmap_calculator::CostmapCalculatorComponent)
