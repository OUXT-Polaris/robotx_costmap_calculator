// Copyright (c) 2022 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <robotx_costmap_calculator/costmap_filter_component.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotx_costmap_calculator
{
CostmapFilterComponent::CostmapFilterComponent(const rclcpp::NodeOptions & options)
: Node("robotx_costmap_filter", options), filterChain_("grid_map::GridMap")
{
  std::string grid_map_topic;
  declare_parameter<std::string>("grid_map_topic", "/perception/combine_grid_map");
  get_parameter("grid_map_topic", grid_map_topic);

  //subscriber
  grid_map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
    grid_map_topic, 1,
    std::bind(&CostmapFilterComponent::gridmapCallback, this, std::placeholders::_1));

  filter_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("filter_map", 1);

  if (filterChain_.configure(
        filterChainParametersName_, get_node_logging_interface(),
        get_node_parameters_interface())) {
    RCLCPP_INFO(this->get_logger(), "Filter chain configured.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Could not configure the filter chain!");
    rclcpp::shutdown();
    return;
  }
}

void CostmapFilterComponent::gridmapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  grid_map::GridMap input_map;
  grid_map::GridMapRosConverter::fromMessage(*msg, input_map);
  grid_map::GridMap filtered_map;
  if (!filterChain_.update(input_map, filtered_map)) {
    RCLCPP_ERROR(this->get_logger(), "Could not update the grid map filter chain!");
    return;
  }
  auto filtered_map_msg = grid_map::GridMapRosConverter::toMessage(filtered_map);
  filter_map_pub_->publish(std::move(filtered_map_msg));
}
}  // namespace robotx_costmap_calculator
RCLCPP_COMPONENTS_REGISTER_NODE(robotx_costmap_calculator::CostmapFilterComponent)
