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
#include <robotx_costmap_calculator/costmap_interpolation_component.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace robotx_costmap_calculator
{
CostmapInterpolationComponent::CostmapInterpolationComponent(const rclcpp::NodeOptions & options)
: Node("robotx_costmap_filter", options)
{
  std::string grid_map_topic;
  declare_parameter<std::string>("grid_map_topic", "/perception/combine_grid_map");
  get_parameter("grid_map_topic", grid_map_topic);

  //subscriber
  grid_map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
    grid_map_topic, 1,
    std::bind(&CostmapInterpolationComponent::gridmapCallback, this, std::placeholders::_1));
}

void CostmapInterpolationComponent::gridmapCallback(
  const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  grid_map::GridMap map;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    map.getPosition(*iterator, position);
  }
}
}  // namespace robotx_costmap_calculator
RCLCPP_COMPONENTS_REGISTER_NODE(robotx_costmap_calculator::CostmapInterpolationComponent)
