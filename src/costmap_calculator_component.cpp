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

#include <quaternion_operation/quaternion_operation.h>
#include <tf2/LinearMath/Quaternion.h>

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
  std::string laserscan_raw_topic;
  std::string current_pose_topic;
  declare_parameter<std::string>("points_raw_topic", "/perception/points_concatenate_node/output");
  get_parameter("points_raw_topic", points_raw_topic);
  declare_parameter<std::string>(
    "laserscan_raw_topic", "/perception/pointcloud_to_laserscan_node/output");
  get_parameter("laserscan_raw_topic", laserscan_raw_topic);
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
  declare_parameter("scan_buffer_size", 2);
  get_parameter("scan_buffer_size", scan_buffer_size_);
  declare_parameter("forgetting_rate", 0.6);
  get_parameter("forgetting_rate", forgetting_rate_);
  declare_parameter<bool>("use_scan", true);
  get_parameter("use_scan", use_scan_);
  std::string key;
  pose_buffer_ =
    std::make_shared<data_buffer::PoseStampedDataBuffer>(get_clock(), key, buffer_length);

  if (use_scan_) {
    laserscan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      laserscan_raw_topic, 10,
      std::bind(&CostmapCalculatorComponent::scanCallback, this, std::placeholders::_1));
  } else {
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_raw_topic, 10,
      std::bind(&CostmapCalculatorComponent::pointCloudCallback, this, std::placeholders::_1));
  }

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    current_pose_topic, 10,
    std::bind(&CostmapCalculatorComponent::poseCallback, this, std::placeholders::_1));

  grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 1);

  cloud_buffer_ =
    boost::circular_buffer<sensor_msgs::msg::PointCloud2::SharedPtr>(scan_buffer_size_);
  scan_buffer_ = boost::circular_buffer<sensor_msgs::msg::LaserScan::SharedPtr>(scan_buffer_size_);

  initGridMap();
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

void CostmapCalculatorComponent::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  scan_buffer_.push_back(scan);
  geometry_msgs::msg::PoseStamped scan_pose;
  if (!pose_buffer_->queryData(scan->header.stamp, scan_pose)) {
    return;
  }
  for (size_t j = 0; j < scan_buffer_.size(); j++) {
    std::stringstream ss;
    ss << j;
    std::string scan_layer_name("scan_layer" + ss.str());
    if (j == scan_buffer_.size() - 1) {
      addPointsToGridMap(transformScanPoints(*scan_buffer_[j]), scan_layer_name);
    } else {
      geometry_msgs::msg::PoseStamped prev_scan_pose;
      if (!pose_buffer_->queryData(scan_buffer_[j]->header.stamp, prev_scan_pose)) {
        return;
      }
      addPointsToGridMap(
        transformScanPoints(*scan_buffer_[j], getRelativePose(scan_pose.pose, prev_scan_pose.pose)),
        scan_layer_name);
    }
  }
  combine();
  publish();
  return;
}

std::vector<geometry_msgs::msg::Point> CostmapCalculatorComponent::transformScanPoints(
  const sensor_msgs::msg::LaserScan & scan, const geometry_msgs::msg::Pose & pose) const
{
  std::vector<geometry_msgs::msg::Point> ret;
  Eigen::Matrix3d scan_rotation_matrix;
  scan_rotation_matrix = quaternion_operation::getRotationMatrix(pose.orientation);
  for (int i = 0; i < static_cast<int>(scan.ranges.size()); i++) {
    if (range_max_ >= scan.ranges[i]) {
      double theta = scan.angle_min + scan.angle_increment * static_cast<double>(i);
      Eigen::VectorXd v(3);
      v(0) = scan.ranges[i] * std::cos(theta);
      v(1) = scan.ranges[i] * std::sin(theta);
      v(2) = 0;
      v = scan_rotation_matrix * v;
      v(0) = v(0) + pose.position.x;
      v(1) = v(1) + pose.position.y;
      v(2) = v(2) + pose.position.z;
      geometry_msgs::msg::Point transformed;
      transformed.x = v(0);
      transformed.y = v(1);
      transformed.z = v(2);
      ret.emplace_back(transformed);
    }
  }
  return ret;
}

void CostmapCalculatorComponent::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  cloud_buffer_.push_back(cloud);
  for (size_t i = 0; i < cloud_buffer_.size(); i++) {
    std::stringstream cloud_ss;
    if (i > 0) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::fromROSMsg(*cloud_buffer_[i], *transform_cloud);
      geometry_msgs::msg::PoseStamped poses;
      if (!pose_buffer_->queryData(cloud->header.stamp, poses)) {
        return;
      }
      Eigen::Matrix3d rotation_matrix;
      geometry_msgs::msg::Quaternion current_pose_orientation = poses.pose.orientation;
      geometry_msgs::msg::Quaternion scan_orientation;
      scan_orientation =
        quaternion_operation::getRotation(poses.pose.orientation, current_pose_orientation);
      rotation_matrix = quaternion_operation::getRotationMatrix(scan_orientation);
      //rotation_matrix=quaternion_operation::getRotationMatrix(poses.pose.orientation);
      Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
      transform_matrix.block<3, 3>(0, 0) = rotation_matrix;
      //transform_matrix.block<3, 1>(0, 3) =
      //Eigen::Vector3d(poses.pose.position.x, poses.pose.position.y, poses.pose.position.z);
      pcl::transformPointCloud(*transform_cloud, *transform_cloud, transform_matrix);
      pcl::toROSMsg(*transform_cloud, *cloud_buffer_[i]);
    }
    cloud_ss << i;
    std::string point_current_layer_name("point_layer" + cloud_ss.str());
    addPointCloudToGridMap(*cloud_buffer_[i], point_current_layer_name);
  }
  combine();
  publish();
  return;
}

void CostmapCalculatorComponent::publish()
{
  auto msg = grid_map::GridMapRosConverter::toMessage(grid_map_);
  msg->header.stamp = get_clock()->now();
  grid_map_pub_->publish(std::move(msg));
}

void CostmapCalculatorComponent::combine()
{
  grid_map_.add("combined", 0.0);
  if (use_scan_) {
    if (scan_buffer_.size() == scan_buffer_size_) {
      for (size_t i = 0; i < scan_buffer_size_; i++) {
        grid_map_["combined"] =
          std::pow(forgetting_rate_, i - 1) * grid_map_["scan_layer" + std::to_string(i)];
      }
    }
  } else {
    if (cloud_buffer_.size() == scan_buffer_size_) {
      for (size_t i = 0; i < scan_buffer_size_; i++) {
        grid_map_["combined"] =
          std::pow(forgetting_rate_, i - 1) * grid_map_["point_layer" + std::to_string(i)];
      }
    }
  }
}

void CostmapCalculatorComponent::addPointsToGridMap(
  const std::vector<geometry_msgs::msg::Point> & points, const std::string & scan_layer_name)
{
  grid_map_.add(scan_layer_name, 0.0);
  for (const auto & point : points) {
    for (grid_map::CircleIterator iterator(
           grid_map_, grid_map::Position(point.x, point.y), resolution_ * 0.5);
         !iterator.isPastEnd(); ++iterator) {
      if (std::isnan(grid_map_.at(scan_layer_name, *iterator))) {
        grid_map_.at(scan_layer_name, *iterator) = 0.0;
      } else {
        if (grid_map_.at(scan_layer_name, *iterator) < 1.0) {
          grid_map_.at(scan_layer_name, *iterator) = grid_map_.at(scan_layer_name, *iterator) + 0.1;
        }
      }
    }
  }
}

void CostmapCalculatorComponent::addPointCloudToGridMap(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & grid_map_layer_name)
{
  grid_map_.add(grid_map_layer_name, 0.0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(cloud, *pcl_cloud);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  for (grid_map::GridMapIterator iterator(grid_map_); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    grid_map_.getPosition(*iterator, position);
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
      grid_map_.at(grid_map_layer_name, *iterator) = 0.0;
    } else {
      grid_map_.at(grid_map_layer_name, *iterator) = sigmoid(1.0, 0.0, (double)num_points);
    }
  }
}

const geometry_msgs::msg::Pose CostmapCalculatorComponent::getRelativePose(
  const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to) const
{
  geometry_msgs::msg::Transform from_translation;
  {
    from_translation.translation.x = from.position.x;
    from_translation.translation.y = from.position.y;
    from_translation.translation.z = from.position.z;
    from_translation.rotation = from.orientation;
  }

  tf2::Transform from_tf;
  {
    tf2::fromMsg(from_translation, from_tf);
  }

  geometry_msgs::msg::Transform to_translation;
  {
    to_translation.translation.x = to.position.x;
    to_translation.translation.y = to.position.y;
    to_translation.translation.z = to.position.z;
    to_translation.rotation = to.orientation;
  }

  tf2::Transform to_tf;
  {
    tf2::fromMsg(to_translation, to_tf);
  }

  tf2::Transform tf_delta = from_tf.inverse() * to_tf;

  geometry_msgs::msg::Pose ret;
  {
    tf2::toMsg(tf_delta, ret);
  }
  return ret;
}
}  // namespace robotx_costmap_calculator

RCLCPP_COMPONENTS_REGISTER_NODE(robotx_costmap_calculator::CostmapCalculatorComponent)
