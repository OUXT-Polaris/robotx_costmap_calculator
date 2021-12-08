#include <costmap_calculator_component.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char **argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto component = std::make_shered<costmap_calculator::costmap_calculatorComponent>(options);
    rclcpp::spin(component);
    rclcpp::shutdown();
    return 0;
}