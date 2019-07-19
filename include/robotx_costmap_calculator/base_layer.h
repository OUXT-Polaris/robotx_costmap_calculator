#ifndef COSTMAP_CALCULATOR_BASE_LAYER_H_INCLUDED
#define COSTMAP_CALCULATOR_BASE_LAYER_H_INCLUDED

#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>

namespace robotx_costmap_calculator
{
    class BaseLayer
    {
    public:
        BaseLayer(double resolution,int num_grids);
        ~BaseLayer();
        void overlayPointCloud(grid_map::GridMap& map,const sensor_msgs::PointCloud2::ConstPtr cloud);
    private:
        double resolution_;
        int num_grids_;
        double sigmoid(double a,double b,double x);
    };
}

#endif  //COSTMAP_CALCULATOR_BASE_LAYER_H_INCLUDED