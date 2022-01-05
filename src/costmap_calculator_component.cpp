#include <robotx_costmap_calculator/costmap_calculator_component.hpp>
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
        grid_map::GridMap map;
        pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(points_raw_topic, 10, 
            std::bind(&CostmapCalculatorComponent::pointCloudCallback, this,std::placeholders::_1));
        grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("grid_map", 10);
    }

    double sigmoid(double a,double b, double x)
    {
        double ret;
        ret = 1/(1+std::exp(-a*(x-b)));
        return ret;
    }

    void CostmapCalculatorComponent::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
    {   
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloud, *pcl_cloud);
        grid_map::GridMap map({"base_layer"});
        map.setFrameId("base_link");
        map.setGeometry(grid_map::Length(resolution_*num_grids_, resolution_*num_grids_), resolution_);
        for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
        {   
            
            grid_map::Position position;
            map.getPosition(*iterator, position);
            double x_min = position.x() - (resolution_*0.5);
            double x_max = position.x() + (resolution_*0.5);
            double y_min = position.y() - (resolution_*0.5);
            double y_max = position.y() + (resolution_*0.5);
            pcl::PassThrough<pcl::PointXYZ> pass; 
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pass.setInputCloud(pcl_cloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(x_min,x_max);
            pass.filter(*cloud_filtered);
            pass.setInputCloud(cloud_filtered);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(y_min,y_max);
            pass.filter(*cloud_filtered);
            int num_points = cloud_filtered->size();
            if(num_points == 0)
            {
                map.at("base_layer",*iterator) = 0.0;
            }
            else
            {
                map.at("base_layer",*iterator) = sigmoid(1.0,0.0,(double)num_points);
            }
        }
        auto outputMessage = grid_map::GridMapRosConverter::toMessage(map);
        grid_map_pub_->publish(std::move(outputMessage));
        return;
    }

}

