#include <robotx_costmap_calculator/costmap.hpp>

namespace costmap_calculator
{
    CostMap::CostMap(double resolution,int num_grids)
    {
        resolution_ = resolution;
        num_grids_ = num_grids;
        map_ = grid_map::GridMap({"base_layer"});
        map_.setFrameId("base_link");
        map_.setGeometry(grid_map::Length(resolution_*num_grids_, resolution_*num_grids_), resolution_);
        base_layer_ptr_ = std::unique_ptr<BaseLayer>(new BaseLayer(resolution_,num_grids_));
    }

    void CostMap::overlayPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
    {   
        base_layer_ptr_->overlayPointCloud(map_,cloud);
        return;
    }
}