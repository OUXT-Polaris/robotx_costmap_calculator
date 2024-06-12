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

#include <pcl_conversions/pcl_conversions.h>

#include <chrono>
#include <data_buffer/data_buffer_base.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <memory>
#include <rclcpp_components/register_node_macro.hpp>
#include <robotx_costmap_calculator/costmap_calculator_component.hpp>
#include <string>
#include <vector>
#ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

namespace robotx_costmap_calculator
{
CostmapCalculatorComponent::CostmapCalculatorComponent(const rclcpp::NodeOptions & options)
: Node("robotx_costmap_calculator", options)
{
  std::string points_raw_topic;
  std::string current_pose_topic;
  declare_parameter<std::string>("points_raw_topic", "/perception/points_concatenate_node/output");
  get_parameter("points_raw_topic", points_raw_topic);
  declare_parameter<std::string>("current_pose_topic", "/current_pose");
  get_parameter("current_pose_topic", current_pose_topic);
  declare_parameter("resolution", 1.0);
  get_parameter("resolution", resolution_);
  declare_parameter("num_grids", 20);
  get_parameter("num_grids", num_grids_);
  declare_parameter("range_max", 20.0);
  get_parameter("range_max", range_max_);
  declare_parameter("visualize_frame_id", "map");
  get_parameter("visualize_frame_id", visualize_frame_id_);
  double buffer_length;
  declare_parameter("buffer_length", 5.0);
  get_parameter("buffer_length", buffer_length);
  declare_parameter("cloud_buffer_size", 2);
  get_parameter("cloud_buffer_size", cloud_buffer_size_);
  declare_parameter("forgetting_rate", 0.6);
  get_parameter("forgetting_rate", forgetting_rate_);
  initGridMap();
  std::string key;
  pose_buffer_ =
    std::make_shared<data_buffer::PoseStampedDataBuffer>(get_clock(), key, buffer_length);

  pointcloud_sub_ = create_subscription<PointCloudAdaptedType>(
    points_raw_topic, 10, [this](const PCLPointCloudTypePtr msg) { pointCloudCallback(msg); });

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    current_pose_topic, 10,
    std::bind(&CostmapCalculatorComponent::poseCallback, this, std::placeholders::_1));

  grid_map_pub_ = create_publisher<GridMapAdaptedType>("grid_map", 1);

  cloud_buffer_ = boost::circular_buffer<PCLPointCloudTypePtr>(cloud_buffer_size_);
}

void CostmapCalculatorComponent::initGridMap()
{
  grid_map_.setFrameId("base_link");
  grid_map_.setGeometry(
    grid_map::Length(resolution_ * num_grids_, resolution_ * num_grids_), resolution_,
    grid_map::Position(0.0, 0.0));
}

double sigmoid(double a, double b, double x)
{
  double ret;
  ret = 1 / (1 + std::exp(-a * (x - b)));
  return ret;
}

void CostmapCalculatorComponent::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
  pose_buffer_->addData(*pose);
  return;
}

void CostmapCalculatorComponent::pointCloudCallback(const PCLPointCloudTypePtr cloud)
{
  cloud_buffer_.push_back(cloud);
  for (size_t i = 0; i < cloud_buffer_.size(); i++) {
    std::stringstream cloud_ss;
    if (i > 0) {
      PCLPointCloudTypePtr transform_cloud(new PCLPointCloudType());
      transform_cloud = cloud_buffer_[i];
      geometry_msgs::msg::PoseStamped poses;
      if (!pose_buffer_->queryData(
            [&]() {
              rclcpp::Time stamp;
              pcl_conversions::fromPCL(cloud->header.stamp, stamp);
              return stamp;
            }(),
            poses)) {
        return;
      }
      Eigen::Matrix3d rotation_matrix;
      geometry_msgs::msg::Quaternion current_pose_orientation = poses.pose.orientation;
      geometry_msgs::msg::Quaternion scan_orientation;
      scan_orientation =
        quaternion_operation::getRotation(poses.pose.orientation, current_pose_orientation);
      rotation_matrix = quaternion_operation::getRotationMatrix(scan_orientation);
      Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
      transform_matrix.block<3, 3>(0, 0) = rotation_matrix;
      pcl::transformPointCloud(*transform_cloud, *transform_cloud, transform_matrix);
      cloud_buffer_[i] = transform_cloud;
    }
    cloud_ss << i;
    std::string point_current_layer_name("point_layer" + cloud_ss.str());
    addPointCloudToGridMap(cloud_buffer_[i], point_current_layer_name);
  }
  combine();
  publish();
  return;
}

void CostmapCalculatorComponent::publish()
{
  grid_map_.setTimestamp(get_clock()->now().nanoseconds());
  grid_map_pub_->publish(grid_map_);
}

void CostmapCalculatorComponent::combine()
{
  grid_map_.add("combined", 0.0);
  if (cloud_buffer_.size() == cloud_buffer_size_) {
    for (size_t i = 0; i < cloud_buffer_size_; i++) {
      grid_map_["combined"] =
        std::pow(forgetting_rate_, i - 1) * grid_map_["point_layer" + std::to_string(i)];
    }
  }
}

void CostmapCalculatorComponent::addPointCloudToGridMap(
  const PCLPointCloudTypePtr & cloud, const std::string & grid_map_layer_name)
{
  grid_map_.add(grid_map_layer_name, 0.0);
  pcl::PassThrough<PCLPointType> pass;
  PCLPointCloudTypePtr cloud_filtered(new PCLPointCloudType());
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    grid_map_.getPosition(*iterator, position);
    double x_min = position.x() - (resolution_ * 0.5);
    double x_max = position.x() + (resolution_ * 0.5);
    double y_min = position.y() - (resolution_ * 0.5);
    double y_max = position.y() + (resolution_ * 0.5);
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_min, x_max);
    pass.filter(*cloud_filtered);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min, y_max);
    pass.filter(*cloud_filtered);
    int num_points = cloud_filtered->size();
    if (num_points == 0) {
      grid_map_.at(grid_map_layer_name, *iterator) = 0.0;
    } else {
      grid_map_.at(grid_map_layer_name, *iterator) = sigmoid(1.0, 0.0, (double)num_points);
    }
  }
}
}  // namespace robotx_costmap_calculator

RCLCPP_COMPONENTS_REGISTER_NODE(robotx_costmap_calculator::CostmapCalculatorComponent)
