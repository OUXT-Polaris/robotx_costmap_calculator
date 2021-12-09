#ifndef COSTMAP_CALCULATOR_COSTMAP_HPP_
#define COSTMAP_CALCULATOR_COSTMAP_HPP_


#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define COSTMAP_CALCULATOR_EXPORT __attribute__((dllexport))
#define COSTMAP_CALCULATOR_IMPORT __attribute__((dllimport))
#else
#define COSTMAP_CALCULATOR_EXPORT __declspec(dllexport)
#define COSTMAP_CALCULATOR_IMPORT __declspec(dllimport)
#endif
#ifdef COSTMAP_CALCULATOR_BUILDING_DLL
#define COSTMAP_CALCULATOR_PUBLIC \
  COSTMAP_CALCULATOR_EXPORT
#else
#define COSTMAP_CALCULATOR_PUBLIC \
  COSTMAP_CALCULATOR_IMPORT
#endif
#define COSTMAP_CALCULATOR_PUBLIC_TYPE \
  COSTMAP_CALCULATOR_PUBLIC
#define COSTMAP_CALCULATOR_LOCAL
#else
#define COSTMAP_CALCULATOR_EXPORT __attribute__((visibility("default")))
#define COSTMAP_CALCULATOR_IMPORT
#if __GNUC__ >= 4
#define COSTMAP_CALCULATOR_PUBLIC __attribute__((visibility("default")))
#define COSTMAP_CALCULATOR_LOCAL __attribute__((visibility("hidden")))
#else
#define COSTMAP_CALCULATOR_PUBLIC
#define COSTMAP_CALCULATOR_LOCAL
#endif
#define COSTMAP_CALCULATOR_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "
#endif

// HEaders in ROS
#include <robotx_costmap_calculator/base_layer.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

// Headers in STL
#include <memory>

namespace robotx_costmap_calculator
{
  class CostMap
    {
    public:
        CostMap(double resolution,int num_grids);
        ~CostMap();
        void overlayPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
    private:
        std::unique_ptr<BaseLayer> base_layer_ptr_;
        grid_map::GridMap map_;
        double resolution_;
        int num_grids_;
    };
}

#endif  //ROBOTX_COSTMAP_CALCULATOR_COSTMAP_HPP_