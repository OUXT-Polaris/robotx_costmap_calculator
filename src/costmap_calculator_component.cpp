#include <robotx_costmap_calculator/costmap_calculator_component.hpp>
#include <robotx_costmap_calculator/costmap.hpp>
#include <robotx_costmap_calculator/base_layer.hpp>
#include <memory>
#include <string>
#include <vector>
#include <grid_map_ros/GridMapRosConverter.hpp>


namespace robotx_costmap_calculator
{
CostmapCalculatorComponent::CostmapCalculatorComponent(const rclcpp::NodeOptions &options)
: Node("robotx_costmap_calculator",options)
{
        std::string points_raw_topic;
        declare_parameter<std::string>("points_raw_topic", "/perception/points_concatenate_node/output");
        get_parameter("points_raw_topic",points_raw_topic);
        declare_parameter("resolution", 1.0);
        get_parameter("resolution",resolution_);
        declare_parameter("num_grids", 20);
        get_parameter("num_grids",num_grids_);
        costmap_ptr_=std::unique_ptr<CostMap>(new CostMap(resolution_,num_grids_));
        pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(points_raw_topic, 10, 
            std::bind(&CostmapCalculatorComponent::pointCloudCallback, this,std::placeholders::_1));
        grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("~/output_topic", 10);
    }


    void CostmapCalculatorComponent::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
    {   
        grid_map::GridMap map_;
        costmap_ptr_->overlayPointCloud(cloud);
        std::unique_ptr<grid_map_msgs::msg::GridMap> outputMessage;
        outputMessage = grid_map::GridMapRosConverter::toMessage(map_);
        grid_map_pub_->publish(std::move(outputMessage));
        return;
    }
}

