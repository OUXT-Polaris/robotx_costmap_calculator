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
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_EXPORT __attribute__((visibility("default")))
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_COMPONENT_PUBLIC __attribute__((visibility("default")))
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

#include <robotx_costmap_calculator/costmap.hpp>

// HEaders in ROS
#include <pcl/filters/crop_hull.h>
#include <pcl/PCLPointCloud2.h>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace costmap_calculator
{
class CostmapCalculatorComponent : public rclcpp::Node
{
public:
  COSTMAP_CALCULATOR_PUBLIC
  explicit CostmapCalculatorComponent(const rclcpp::NodeOptions & options);
private:
  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr grid_map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
  std::unique_ptr<CostMap> costmap_ptr_;
  std::string points_raw_topic_;
  std::string output_topic_;
  double resolution_;
  int num_grids_;
};
}

#endif  //ROBOTX_COSTMAP_CALCULATOR_COSTMAP_COMPONENT_HPP_