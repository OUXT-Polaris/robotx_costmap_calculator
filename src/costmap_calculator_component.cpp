#include <costmap_calculator/costmap_calculator_component.hpp>

namespace costmap_calculator
{
CostmapCalculatorComponent::CostmapCalculatorComponent(const rclcpp::NodeOptions &options)
: Node("costmap_calculator",options)
{
        std::string points_raw_topic;
        declare_parameter("points_raw_topic", "/points_raw");
        get_parameter("point_raw_topic",points_raw_topic);
        declare_parameter("resolution", 1.0);
        get_parameter("resolution",resolution_);
        declare_parameter("num_grids", 20.0);
        get_parameter("num_grids",num_grids_);
        costmap_ptr_ = std::unique_ptr<CostMap>(new CostMap(resolution_,num_grids_));
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_raw_topic, 10, 
            std::bind(&CostmapCalculatorComponent::pointCloudCallback, this,std::placeholders::_1));
        grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("~/output_topic", 10);
    }

    void CostmapCalculatorComponent::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
    {
        costmap_ptr_->overlayPointCloud(cloud);
        grid_map_msgs::msg::GridMap gridmap_msg;
        gridmap_msg= costmap_ptr_->getGridMap();
        grid_map_pub_->publish(gridmap_msg);
        return;
    }
}
