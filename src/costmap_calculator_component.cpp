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

#include <chrono>
#include <data_buffer/data_buffer_base.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <memory>
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
  declare_parameter<std::string>("current_pose_topic", "/current_pose");
  get_parameter("current_pose_topic", current_pose_topic);
  declare_parameter("resolution", 0.5);
  get_parameter("resolution", resolution_);
  declare_parameter("num_grids", 50);
  get_parameter("num_grids", num_grids_);
  declare_parameter("range_max", 20.0);
  get_parameter("range_max", range_max_);
  declare_parameter("visualize_frame_id", "map");
  get_parameter("visualize_frame_id", visualize_frame_id_);
  declare_parameter("buffer_length", 2.0);
  double buffer_length;
  get_parameter("buffer_length", buffer_length);
  int point_buffer_size_;
  declare_parameter("point_buffer_size_", 2);
  get_parameter("point_buffer_size_", point_buffer_size_);
  int scan_buffer_size_;
  declare_parameter("scan_buffer_size_", 2);
  get_parameter("scan_buffer_size_", scan_buffer_size_);
  declare_parameter("point_late", 0.5);
  get_parameter("point_late", point_late);
  double scan_late;
  declare_parameter("scan_late", 0.1);
  get_parameter("scan_late", scan_late);
  declare_parameter("currentpoint_downlate", 0.6);
  get_parameter("currentpoint_downlate", currentpoint_downlate);
  std::string key;
  data_buffer =
    std::make_shared<data_buffer::PoseStampedDataBuffer>(get_clock(), key, buffer_length);

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

  combine_grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("combine_grid_map", 1);
  cloud_buffer_ = boost::circular_buffer<sensor_msgs::msg::PointCloud2>(point_buffer_size_);
  scan_buffer_ = boost::circular_buffer<sensor_msgs::msg::LaserScan>(scan_buffer_size_);
  map_data_ = boost::circular_buffer<grid_map::GridMap>(2);

  initGridMap();
}

void CostmapCalculatorComponent::initGridMap()
{
  map.setFrameId("base_link");
  map.setGeometry(
    grid_map::Length(resolution_ * num_grids_, resolution_ * num_grids_), resolution_,
    grid_map::Position(0.0, 0.0));
  combine_map.setFrameId("base_link");
  combine_map.setGeometry(
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
  pose_data = *pose;
  data_buffer->addData(pose_data);
  return;
}


void CostmapCalculatorComponent::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  scan_buffer_.push_back(*scan);
  for (size_t j = 0; j < scan_buffer_.size(); j++) {
    sensor_msgs::msg::LaserScan scan_ = scan_buffer_[j];
    if (j > 0) {
      geometry_msgs::msg::PoseStamped scan_poses;
      data_buffer->queryData(scan->header.stamp, scan_poses);
      TransformScan(scan_, scan_poses);
    }
    std::stringstream ss;
    ss << j;
    std::string scan_layer_name("scan_layer" + ss.str());
    map[scan_layer_name] = getScanToGridMap(scan_, scan_layer_name);    
  }
  // combine_map.add("point_combined_layer", 0.0);
  std::string scan_combined_layer_name("scan_combined_layer");
  combine_map.add(scan_combined_layer_name, 0.0);
  if (map.exists("scan_layer1")) {
    // combine_map["point_combined_layer"] = currentpoint_downlate * map["point_layer0"] + point_late  * map["point_layer1"];
    combine_map[scan_combined_layer_name] = map["scan_layer0"] + scan_late * map["scan_layer1"];
    sensor_msgs::msg::PointCloud2 input_cloud;
    grid_map::GridMapRosConverter::toPointCloud(combine_map, scan_combined_layer_name,input_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clustering_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(input_cloud,*clustering_cloud);
    costmapToObstaclePolygon(clustering_cloud);
  }
  auto combine_outputMessage = grid_map::GridMapRosConverter::toMessage(combine_map);
  auto outputMessage = grid_map::GridMapRosConverter::toMessage(map);
  grid_map_pub_->publish(std::move(outputMessage));
  combine_grid_map_pub_->publish(std::move(combine_outputMessage));
  return;
}

 void CostmapCalculatorComponent::costmapToObstaclePolygon(pcl::PointCloud<pcl::PointXYZ>::Ptr clustering_cloud)
 {
  std::vector<geometry_msgs::msg::Polygon> polygons;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (clustering_cloud);
  std::cout<<__FILE__<<","<<__LINE__<<std::endl;
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (1.0); // 2cm
  ec.setMinClusterSize (1);
  ec.setMaxClusterSize (2500);
  ec.setSearchMethod (tree);
  ec.setInputCloud (clustering_cloud);
  ec.extract (cluster_indices);
  int j = 0;
  std::cout<<__FILE__<<","<<__LINE__<<std::endl;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    std::cout<<__FILE__<<","<<__LINE__<<std::endl;
    
    j++;
  }

  return;
 }

void CostmapCalculatorComponent::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  cloud_buffer_.push_back(*cloud);
  // for (size_t i = 0; i < cloud_buffer_.size(); i++) {
  //   sensor_msgs::msg::PointCloud2 cloud_ = cloud_buffer_[i];
  //   std::stringstream cloud_ss;
  //   if (i > 0) {
  //     pcl::PointCloud<pcl::PointXYZ>::Ptr transform_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  //     pcl::fromROSMsg(cloud_, *transform_cloud);
  //     geometry_msgs::msg::PoseStamped poses;
  //     data_buffer->queryData(cloud->header.stamp, poses);
  //     Eigen::Matrix3d rotation_matrix;
  //     geometry_msgs::msg::Quaternion current_pose_orientation = pose_data.pose.orientation;
  //     geometry_msgs::msg::Quaternion scan_orientation;
  //     scan_orientation =
  //       quaternion_operation::getRotation(poses.pose.orientation, current_pose_orientation);
  //     rotation_matrix = quaternion_operation::getRotationMatrix(scan_orientation);
  //     //rotation_matrix=quaternion_operation::getRotationMatrix(poses.pose.orientation);
  //     Eigen::Matrix4d transform_matrix = Eigen::Matrix4d::Identity();
  //     transform_matrix.block<3, 3>(0, 0) = rotation_matrix;
  //     //transform_matrix.block<3, 1>(0, 3) =
  //     //Eigen::Vector3d(poses.pose.position.x, poses.pose.position.y, poses.pose.position.z);
  //     pcl::transformPointCloud(*transform_cloud, *transform_cloud, transform_matrix);
  //     pcl::toROSMsg(*transform_cloud, cloud_);
  //   }
  //   cloud_ss << i;
  //   std::string point_current_layer_name("point_layer" + cloud_ss.str());
  //   map[point_current_layer_name] = getPointCloudToGridMap(cloud_, point_current_layer_name);
  // }
  // combine_map.add("point_combined_layer", 0.0);
  // combine_map.add("scan_combined_layer", 0.0);
  // combine_map.add("combined_layer",0.0);
  // if (map.exists("point_layer1") && map.exists("scan_layer1")) {
  //   combine_map["point_combined_layer"] = currentpoint_downlate * map["point_layer0"] + point_late  * map["point_layer1"];
  //   combine_map["scan_combined_layer"] = map["scan_layer0"] + scan_late * map["scan_layer1"];
  //   if(combine_map.exists("scan_combined_layer")){
  //     combine_map["combined_layer"] =0.1*combine_map["point_combined_layer"] + 0.1*combine_map["scan_combined_layer"];
  //   }
  // }
  // auto combine_outputMessage = grid_map::GridMapRosConverter::toMessage(combine_map);
  // auto outputMessage = grid_map::GridMapRosConverter::toMessage(map);
  // grid_map_pub_->publish(std::move(outputMessage));
  // combine_grid_map_pub_->publish(std::move(combine_outputMessage));
  return;
}

void CostmapCalculatorComponent::TransformScan(
  const sensor_msgs::msg::LaserScan & scan, const geometry_msgs::msg::PoseStamped & pose)
{
  Eigen::Matrix3d scan_rotation_matrix;
  scan_rotation_matrix = quaternion_operation::getRotationMatrix(pose.pose.orientation);
  for (int i = 0; i < static_cast<int>(scan.ranges.size()); i++) {
    if (range_max_ >= scan.ranges[i]) {
      double theta = scan.angle_min + scan.angle_increment * static_cast<double>(i);
      double scan_x = scan.ranges[i] * std::cos(theta);
      double scan_y = scan.ranges[i] * std::sin(theta);
      double transform_x = scan_x * scan_rotation_matrix(0, 0) +
                           scan_y * scan_rotation_matrix(0, 1) + pose.pose.position.x;
      double transform_y = scan_x * scan_rotation_matrix(1, 0) +
                           scan_y * scan_rotation_matrix(1, 1) + pose.pose.position.y;
      scan_x = transform_x;
      scan_y = transform_y;
    }
  }
}

grid_map::Matrix CostmapCalculatorComponent::getScanToGridMap(
  const sensor_msgs::msg::LaserScan & scan, const std::string & scan_layer_name)
{
  map.add(scan_layer_name, 0.0);
  for (int i = 0; i < static_cast<int>(scan.ranges.size()); i++) {
    if (range_max_ >= scan.ranges[i]) {
      double theta = scan.angle_min + scan.angle_increment * static_cast<double>(i);
      double scan_x = scan.ranges[i] * std::cos(theta);
      double scan_y = scan.ranges[i] * std::sin(theta);
      //for (grid_map::CircleIterator iterator(map, grid_map::Position(scan_x, scan_y), 1.0);
      for (grid_map::CircleIterator iterator(
             map, grid_map::Position(scan_x, scan_y), resolution_ * 0.5);
           !iterator.isPastEnd(); ++iterator) {
        if (std::isnan(map.at(scan_layer_name, *iterator))) {
          map.at(scan_layer_name, *iterator) = 0.0;
        } else {
          if (map.at(scan_layer_name, *iterator) < 1.0) {
            map.at(scan_layer_name, *iterator) = map.at(scan_layer_name, *iterator) + 0.1;
          }
        }
      }
    }
  }
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    if (std::isnan(map.at(scan_layer_name, *iterator))) {
      map.at(scan_layer_name, *iterator) = 0.0;
    }
  }
  return map[scan_layer_name];
}

grid_map::Matrix CostmapCalculatorComponent::getPointCloudToGridMap(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & grid_map_layer_name)
{
  map.add(grid_map_layer_name, 0.0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(cloud, *pcl_cloud);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
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
}  // namespace robotx_costmap_calculator

RCLCPP_COMPONENTS_REGISTER_NODE(robotx_costmap_calculator::CostmapCalculatorComponent)
