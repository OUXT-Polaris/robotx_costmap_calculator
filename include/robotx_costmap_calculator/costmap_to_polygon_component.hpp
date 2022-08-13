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

#ifndef COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_HPP_
#define COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_EXPORT __attribute__((dllexport))
#define COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_EXPORT __declspec(dllexport)
#define COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_BUILDING_DLL
#define COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_PUBLIC \
  COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_EXPORT
#else
#define COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_PUBLIC \
  COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_IMPORT
#endif
#define COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_PUBLIC_TYPE \
  COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_PUBLIC
#define COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_LOCAL
#else
#define COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_EXPORT \
  __attribute__((visibility("default")))
#define COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_PUBLIC \
  __attribute__((visibility("default")))
#define COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_PUBLIC
#define COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_LOCAL
#endif
#define COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "
#endif

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <memory>
#include <pcl_apps_msgs/msg/polygon_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace robotx_costmap_calculator
{
class CostmapToPolygonComponent : public rclcpp::Node
{
public:
  COSTMAP_CALCULATOR_COSTMAP_TO_POLYGON_COMPONENT_PUBLIC
  explicit CostmapToPolygonComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_sub_;
  void gridmapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
  std::unordered_map<double, double> position_map;
};
}  // namespace robotx_costmap_calculator

#endif  //ROBOTX_COSTMAP_TO_POLYGON_COSTMAP_COMPONENT_HPP_