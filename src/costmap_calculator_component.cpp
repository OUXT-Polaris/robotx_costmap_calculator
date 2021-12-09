#include <robotx_costmap_calculator/costmap_calculator_component.hpp>
#include <robotx_costmap_calculator/costmap.hpp>
#include <memory>
#include <string>

namespace robotx_costmap_calculator
{
CostmapCalculatorComponent::CostmapCalculatorComponent(const rclcpp::NodeOptions &options)
: Node("robotx_costmap_calculator",options)
{
        std::string points_raw_topic;
        declare_parameter("points_raw_topic", "/points_raw");
        get_parameter("point_raw_topic",points_raw_topic);
        declare_parameter("resolution", 1.0);
        get_parameter("resolution",resolution_);
        declare_parameter("num_grids", 20.0);
        get_parameter("num_grids",num_grids_);
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(points_raw_topic, 10, 
            std::bind(&CostmapCalculatorComponent::pointCloudCallback, this,std::placeholders::_1));
        grid_map_pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>("~/output_topic", 10);
    }


    void CostmapCalculatorComponent::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
    {
        grid_map::GridMap map_;
        std::unique_ptr<grid_map_msgs::msg::GridMap> outputMessage;
        outputMessage = grid_map::GridMapRosConverter::toMessage(map_);
        grid_map_pub_->publish(std::move(outputMessage));
        return;
    }
}
