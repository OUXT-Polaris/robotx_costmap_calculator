#ifndef COSTMAP_CALCULATOR_COSTMAP_H_INCLUDED
#define COSTMAP_CALCULATOR_COSTMAP_H_INCLUDED

// HEaders in ROS
#include <robotx_costmap_calculator/base_layer.h>
#include <grid_map_msgs/GridMap.h>
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
        void overlayPointCloud(const sensor_msgs::PointCloud2::ConstPtr cloud);
        grid_map_msgs::GridMap getGridMap()
        {
            grid_map_msgs::GridMap grid_map;
            grid_map::GridMapRosConverter::toMessage(map_,grid_map);
            return grid_map;
        }
    private:
        std::unique_ptr<BaseLayer> base_layer_ptr_;
        grid_map::GridMap map_;
        double resolution_;
        int num_grids_;
    };
}

#endif  //COSTMAP_CALCULATOR_COSTMAP_H_INCLUDED