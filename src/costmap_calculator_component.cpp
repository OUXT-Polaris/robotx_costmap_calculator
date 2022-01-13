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
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <memory>
#include <robotx_costmap_calculator/costmap_calculator_component.hpp>
#include <string>
#include <vector>

namespace robotx_costmap_calculator
{
CostmapCalculatorComponent::CostmapCalculatorComponent(const rclcpp::NodeOptions & options)
: Node("robotx_costmap_calculator", options)
{
  std::string points_raw_topic;
  declare_parameter<std::string>("points_raw_topic", "/perception/points_concatenate_node/output");
  get_parameter("points_raw_topic", points_raw_topic);
  declare_parameter("resolution", 1.0);
  get_parameter("resolution", resolution_);
  declare_parameter("num_grids", 20);
  get_parameter("num_grids", num_grids_);

  std::string current_pose_topic;
  declare_parameter("current_pose_topic", "current_pose");
  get_parameter("current_pose_topic", current_pose_topic);
  current_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    current_pose_topic, 1,
    std::bind(&CostmapCalculatorComponent::currentPoseCallback, this, std::placeholders::_1));

  grid_map::GridMap map;
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    points_raw_topic, 10,
    std::bind(&CostmapCalculatorComponent::pointCloudCallback, this, std::placeholders::_1));
  grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);
}

double sigmoid(double a, double b, double x)
{
  double ret;
  ret = 1 / (1 + std::exp(-a * (x - b)));
  return ret;
}

void CostmapCalculatorComponent::currentPoseCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr data)
{
  current_pose_ = *data;
}

void CostmapCalculatorComponent::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud, *pcl_cloud);
  grid_map::GridMap map({"base_layer"});
  map.setFrameId("base_link");
  map.setGeometry(
    grid_map::Length(resolution_ * num_grids_, resolution_ * num_grids_ * 0.5), resolution_);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  rclcpp::Time start_time = get_clock()->now();
  rclcpp::Duration duration(0.0);
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    map.getPosition(*iterator, position);
    double x_min = position.x() - (resolution_ * 0.5);
    double x_max = position.x() + (resolution_ * 0.5);
    double y_min = position.y() - (resolution_ * 0.25);
    double y_max = position.y() + (resolution_ * 0.25);
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
      map.at("base_layer", *iterator) = 0.0;
    } else {
      map.at("base_layer", *iterator) = sigmoid(1.0, 0.0, (double)num_points);
    }
  }
  rclcpp::Time finish_time = get_clock()->now();
  duration =finish_time-start_time;
  RCLCPP_INFO(get_logger(),"gridmap_time:%f",duration.nanoseconds());
  auto outputMessage = grid_map::GridMapRosConverter::toMessage(map);
  grid_map_pub_->publish(std::move(outputMessage));
  return;
}

}  // namespace robotx_costmap_calculator
