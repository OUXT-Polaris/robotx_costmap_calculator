#ifndef COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_H_INCLUDED
#define COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_H_INCLUDED

// Headers in this package
#include <robotx_costmap_calculator/costmap.h>

// Headers in ROS
#include <pcl/filters/crop_hull.h>
#include <pcl_ros/filters/filter.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/PCLPointCloud2.h>
#include <grid_map_msgs/GridMap.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

// Headers in STL
#include <memory>

namespace robotx_costmap_calculator
{
    class RobotXCostmapCalculator : public nodelet::Nodelet
    {
    public:
        RobotXCostmapCalculator();
        void run();
    protected:
        void onInit();
    private:
        ros::Publisher grid_map_pub_;
        ros::Subscriber pointcloud_sub_;
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr cloud);
        ros::NodeHandle pnh_;
        ros::NodeHandle nh_;
        std::unique_ptr<CostMap> costmap_ptr_;
        std::string points_raw_topic_;
        std::string output_topic_;
        double resolution_;
        int num_grids_;
    };
}

#endif  //COSTMAP_CALCULATOR_COSTMAP_CALCULATOR_H_INCLUDED