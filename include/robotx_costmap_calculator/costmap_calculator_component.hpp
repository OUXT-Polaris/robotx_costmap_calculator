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

#ifndef COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_HPP_
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_EXPORT __attribute__((dllexport))
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_EXPORT __declspec(dllexport)
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_BUILDING_DLL
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_PUBLIC \
  COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_EXPORT
#else
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_PUBLIC \
  COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_IMPORT
#endif
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_PUBLIC_TYPE \
  COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_PUBLIC
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_LOCAL
#else
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_PUBLIC
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_LOCAL
#endif
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "
#endif

// Headers in ROS
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>
#include <data_buffer/data_buffer_base.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs_data_buffer/pose_stamped_data_buffer.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_type_adapter/type_adapter.hpp>
#include <memory>
#include <pcl_apps_msgs/msg/polygon_array.hpp>
#include <pcl_type_adapter/pcl_type_adapter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace robotx_costmap_calculator
{
class CostmapCalculatorComponent : public rclcpp::Node
{
  using GridMapAdaptedType = rclcpp::TypeAdapter<grid_map::GridMap, grid_map_msgs::msg::GridMap>;
  using PCLPointType = pcl::PointXYZI;
  using PCLPointCloudType = pcl::PointCloud<PCLPointType>;
  using PCLPointCloudTypePtr = std::shared_ptr<PCLPointCloudType>;
  using PointCloudAdaptedType =
    rclcpp::TypeAdapter<PCLPointCloudTypePtr, sensor_msgs::msg::PointCloud2>;

public:
  COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_PUBLIC
  explicit CostmapCalculatorComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<GridMapAdaptedType>::SharedPtr grid_map_pub_;
  rclcpp::Subscription<PointCloudAdaptedType>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  void initGridMap();
  void pointCloudCallback(const PCLPointCloudTypePtr & cloud);
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
  boost::circular_buffer<PCLPointCloudTypePtr> cloud_buffer_;
  grid_map::GridMap grid_map_;
  void addPointCloudToGridMap(
    const PCLPointCloudTypePtr & cloud, const std::string & grid_map_layer_name);
  void combine();
  void publish();
  double resolution_;
  int num_grids_;
  double range_max_;
  double forgetting_rate_;
  size_t cloud_buffer_size_;
  std::string visualize_frame_id_;
  std::shared_ptr<data_buffer::PoseStampedDataBuffer> pose_buffer_;
  const geometry_msgs::msg::Pose getRelativePose(
    const geometry_msgs::msg::Pose & from, const geometry_msgs::msg::Pose & to) const;
};
}  // namespace robotx_costmap_calculator

#endif  //ROBOTX_COSTMAP_CALCULATOR_COSTMAP_COMPONENT_HPP_
