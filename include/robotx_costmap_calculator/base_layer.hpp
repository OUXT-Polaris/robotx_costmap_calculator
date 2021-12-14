#ifndef COSTMAP_CALCULATOR_BASE_LAYER_HPP_
#define COSTMAP_CALCULATOR_BASE_LAYER_HPP_


#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_EXPORT __attribute__((dllexport))
#define COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_EXPORT __declspec(dllexport)
#define COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_BUILDING_DLL
#define COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_PUBLIC \
  COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_EXPORT
#else
#define COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_PUBLIC \
  COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_IMPORT
#endif
#define COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_PUBLIC_TYPE \
  COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_PUBLIC
#define COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_LOCAL
#else
#define COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_EXPORT __attribute__((visibility("default")))
#define COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_PUBLIC
#define COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_LOCAL
#endif
#define COSTMAP_CALCULATOR_BASE_LAYER_COMPONENT_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "
#endif


#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

namespace robotx_costmap_calculator
{
    class BaseLayer
    {
    public:
        BaseLayer(double resolution,double num_grids);
        void overlayPointCloud(grid_map::GridMap& map,const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
    private:
        double resolution_;
        int num_grids_;
        double sigmoid(double a,double b,double x);
    };
}

#endif  //COSTMAP_CALCULATOR_BASE_LAYER_HPP_