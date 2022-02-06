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
  declare_parameter("range_max", 100.0);
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
  image_pub_ =create_publisher<sensor_msgs::msg::Image>("laser_image",10);

  map_data_=boost::circular_buffer<grid_map::GridMap>(2);
  cv::namedWindow("laser_image", cv::WINDOW_AUTOSIZE);
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
  grid_map::GridMap map;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud, *pcl_cloud);
  map.add("base_layer", 0.0);
  map.add("past_layer", 0.0);
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
  map_data_.push_back(map);
  map["past_layer"]=map_data_[0].get("base_layer");
  duration = finish_time - start_time;
  RCLCPP_INFO(get_logger(), "gridmap_time:%f", duration.seconds());
  auto outputMessage = grid_map::GridMapRosConverter::toMessage(map);
  grid_map_pub_->publish(std::move(outputMessage));
  return;
}

void CostmapCalculatorComponent::scanCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  laser_image = cv::Mat::zeros(cv::Size(400, 400), CV_8U);
  for (int i=0; i < static_cast<int>(scan->ranges.size());i++){
    if(range_max_ >=scan->ranges[i]){
      double theta =scan->angle_min +scan->angle_increment *static_cast<double>(i);
      int image_x=10*scan->ranges[i]*std::cos(theta);
      int image_y=10*scan->ranges[i]*std::sin(theta);
      if(image_x<400 &&10<image_x){
        if(image_y<400 &&10<image_y){
          int x=image_x;
          int y=image_y;
          laser_image.at<unsigned char>(y,x)=255;
        }
      } 
    }
  }
  header.frame_id=visualize_frame_id_;
  img_bridge = cv_bridge::CvImage(header, "mono8", laser_image);
  img_bridge.toImageMsg(img_msg);
  image_pub_->publish(img_msg);
  return;
}
}  // namespace robotx_costmap_calculator

RCLCPP_COMPONENTS_REGISTER_NODE(robotx_costmap_calculator::CostmapCalculatorComponent)
