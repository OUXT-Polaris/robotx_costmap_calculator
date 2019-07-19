#include <robotx_costmap_calculator/robotx_costmap_calculator.h>

namespace robotx_costmap_calculator
{
    RobotXCostmapCalculator::RobotXCostmapCalculator()
    {

    }

    void RobotXCostmapCalculator::onInit()
    {
        nh_ = getNodeHandle();
        pnh_ = getPrivateNodeHandle();
        pnh_.param<std::string>("points_raw_topic", points_raw_topic_, "points_raw");
        pnh_.param<std::string>("output_topic", output_topic_, "output");
        costmap_ptr_ = std::unique_ptr<CostMap>(new CostMap(0.4,50));
        grid_map_pub_ = pnh_.advertise<grid_map_msgs::GridMap>(output_topic_,10);
        pointcloud_sub_ = nh_.subscribe(points_raw_topic_, 10, &RobotXCostmapCalculator::pointCloudCallback, this);
    }

    void RobotXCostmapCalculator::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr cloud)
    {
        costmap_ptr_->overlayPointCloud(cloud);
        grid_map_msgs::GridMap msg = costmap_ptr_->getGridMap();
        grid_map_pub_.publish(msg);
        return;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robotx_costmap_calculator::RobotXCostmapCalculator,nodelet::Nodelet);