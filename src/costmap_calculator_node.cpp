#include <robotx_costmap_calculator/costmap_calculator_component.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto component = std::make_shared<robotx_costmap_calculator::CostmapCalculatorComponent>(options);
    rclcpp::spin(component);
    rclcpp::shutdown();
    return 0;
}