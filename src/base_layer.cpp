#include <costmap_calculator/base_layer.hpp>

namespace costmap_calculator
{
    BaseLayer::BaseLayer(double resolution,int num_grids)
    {
        resolution_ = resolution;
        num_grids_ = num_grids;
    }

    BaseLayer::~BaseLayer()
    {

    }

    double BaseLayer::sigmoid(double a,double b,double x)
    {
        double ret;
        ret = 1/(1+std::exp(-a*(x-b)));
        return ret;
    }

    void BaseLayer::overlayPointCloud(grid_map::GridMap& map,const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*cloud, *pcl_cloud);
        grid_map::Matrix mat = map["base_layer"];
        for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
        {
            int i = iterator.getLinearIndex();
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
        return;
    }
}