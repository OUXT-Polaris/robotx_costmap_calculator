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
#include <rclcpp_components/register_node_macro.hpp>
#include <robotx_costmap_calculator/costmap_calculator_component.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <vector>
#include <grid_map_cv/GridMapCvConverter.hpp>

namespace robotx_costmap_calculator
{
CostmapCalculatorComponent::CostmapCalculatorComponent(const rclcpp::NodeOptions & options)
: Node("robotx_costmap_calculator", options)
{
  std::string points_raw_topic;
  std::string laserscan_raw_topic;
  declare_parameter<std::string>("points_raw_topic", "/perception/points_concatenate_node/output");
  get_parameter("points_raw_topic", points_raw_topic);
  declare_parameter<std::string>("laserscan_raw_topic", "/perception/pointcloud_to_laserscan_node/output");
  get_parameter("laserscan_raw_topic", laserscan_raw_topic);
  declare_parameter("resolution", 1.0);
  get_parameter("resolution", resolution_);
  declare_parameter("num_grids", 20);
  get_parameter("num_grids", num_grids_);
  declare_parameter("laser_resolution",0.1);
  get_parameter("laser_resolution", laser_resolution_);
  declare_parameter("laser_num_grids", 200);
  get_parameter("laser_num_grids", laser_num_grids_);
  declare_parameter("range_max", 20.0);
  get_parameter("range_max", range_max_);
  declare_parameter("visualize_frame_id", "map");
  get_parameter("visualize_frame_id", visualize_frame_id_);

  grid_map::GridMap map;
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    points_raw_topic, 10,
    std::bind(&CostmapCalculatorComponent::pointCloudCallback, this, std::placeholders::_1));

  laserscan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    laserscan_raw_topic, 10,
    std::bind(&CostmapCalculatorComponent::scanCallback, this, std::placeholders::_1));
    
  grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 1);

  map_data_=boost::circular_buffer<grid_map::GridMap>(2);
}

double sigmoid(double a, double b, double x)
{
  double ret;
  ret = 1 / (1 + std::exp(-a * (x - b)));
  return ret;
}

void CostmapCalculatorComponent::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{ 
  /*
  grid_map::GridMap map;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud, *pcl_cloud);
  map.add("point_layer", 0.0);
  map.add("point_past_layer", 0.0);
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
      map.at("point_layer", *iterator) = 0.0;
    } else {
      map.at("point_layer", *iterator) = sigmoid(1.0, 0.0, (double)num_points);
    }
  }
  rclcpp::Time finish_time = get_clock()->now();
  map_data_.push_back(map);
  map["point_past_layer"]=map_data_[0].get("point_layer");
  duration = finish_time - start_time;
  //RCLCPP_INFO(get_logger(), "gridmap_time:%f", duration.seconds());
  auto outputMessage = grid_map::GridMapRosConverter::toMessage(map);
  //grid_map_pub_->publish(std::move(outputMessage));
  return;
  */
}

void CostmapCalculatorComponent::scanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr scan)
{

  //std::cout<<__FILE__<<","<<__LINE__<<std::endl;
  grid_map::GridMap map;
  map.add("laser_layer",0.0);
  map.add("laser_sum_layer",0.0);
  map.add("laser_past_layer",0.0);
  map.setFrameId("base_link");
  map.setGeometry(
    grid_map::Length(laser_resolution_ * laser_num_grids_, laser_resolution_ * laser_num_grids_), laser_resolution_);
  for (int i=0; i < static_cast<int>(scan->ranges.size());i++){
    if(range_max_ >=scan->ranges[i]){
      double theta =scan->angle_min +scan->angle_increment *static_cast<double>(i);
      double scan_x=scan->ranges[i]*std::cos(theta);
      double scan_y=scan->ranges[i]*std::sin(theta);
      for (grid_map::CircleIterator iterator(map,grid_map::Position(scan_x,scan_y),0.5); !iterator.isPastEnd(); ++iterator) {
        if(std::isnan(map.at("laser_layer",*iterator))){
          map.at("laser_layer",*iterator)=0.1;
        }
        else{
          map.at("laser_layer",*iterator)=map.at("laser_layer",*iterator)+0.1;
        }
      }
    }
  }
  map_data_.push_back(map);
  map["laser_past_layer"]=map_data_[0].get("laser_layer");
  //map["laser_sum_layer"]=map["laser_layer"]+0.5*map["laser_past_layer"];
  /*
  map.clearAll();  
  map.setFrameId("base_link");
  laser_image = cv::Mat::zeros(cv::Size(20,20), CV_8U);
  for (int i=0; i < static_cast<int>(scan->ranges.size());i++){
    if(range_max_ >=scan->ranges[i]){
      double theta =scan->angle_min +scan->angle_increment *static_cast<double>(i);
      RCLCPP_INFO(get_logger(), "theta:%f",theta);
      double scan_x=scan->ranges[i]*std::cos(theta);
      double scan_y=scan->ranges[i]*std::sin(theta);
      RCLCPP_INFO(get_logger(), "scan_x[%d]:%f,scan_y[%d]:%f",i,scan_x,i,scan_y);
      int x=-std::round(scan_y)+10;
      int y=-std::round(scan_x)+10;
      if(x<20 &&0<x){
        if(y<20 &&0<y){
          laser_image.at<unsigned char>(y,x)=255;
          //laser_image.at<int>(5,0)=255;
        }
      } 
    }
  }
  grid_map::GridMapCvConverter::initializeFromImage(laser_image,laser_resolution_,map,grid_map::Position::Zero());
  grid_map::GridMapCvConverter::addLayerFromImage<uint16_t, 1>(
    laser_image, "laser_layer", map, 0.0,
    1.0, 0.5);
  */
  /*
  img_bridge = cv_bridge::CvImage(header, "mono8", laser_image);
  img_bridge.toImageMsg(img_msg);
  image_pub_->publish(img_msg);
  */
  auto message = grid_map::GridMapRosConverter::toMessage(map);
  grid_map_pub_->publish(std::move(message));
  map.clearAll();  
  return;
}
}  // namespace robotx_costmap_calculator

RCLCPP_COMPONENTS_REGISTER_NODE(robotx_costmap_calculator::CostmapCalculatorComponent)
