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
: Node("robotx_costmap_interpolation", options)
{
  std::string grid_map_topic;
  declare_parameter<std::string>("grid_map_topic", "/perception/combine_grid_map");
  get_parameter("grid_map_topic", grid_map_topic);
  declare_parameter<std::string>("input_layer_name", "combined");
  get_parameter("input_layer_name", input_layer_name_);
  declare_parameter<std::string>("interpolation_type", "Cubic");
  get_parameter("interpolation_type", interpolationMethod_);
  declare_parameter("interpolation_map_resolution", 0.05);
  get_parameter("interpolation_map_resolution", interpolation_map_resolution_);
  declare_parameter("num_grids", 200);
  get_parameter("num_grids", num_grids_);

  //subscriber
  grid_map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
    grid_map_topic, 1,
    std::bind(&CostmapInterpolationComponent::gridmapCallback, this, std::placeholders::_1));

  interpolation_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("interpolation_map", 1);
  initGridMap();
}

void CostmapInterpolationComponent::initGridMap()
{
  interpolation_map_.setFrameId("base_link");
  interpolation_map_.setGeometry(
    grid_map::Length(
      interpolation_map_resolution_ * num_grids_, interpolation_map_resolution_ * num_grids_),
    interpolation_map_resolution_, grid_map::Position(0.0, 0.0));
}

void CostmapInterpolationComponent::gridmapCallback(
  const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  interpolation_map_.add("interpolation_layer", 0.0);
  grid_map::GridMap input_map;
  grid_map::GridMapRosConverter::fromMessage(*msg, input_map);
  for (grid_map::GridMapIterator iterator(interpolation_map_); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position position;
    interpolation_map_.getPosition(*iterator, position);
    interpolation_map_.at("interpolation_layer", *iterator) = interpolation_map_.atPosition(
      input_layer_name_, position,
      interpolationMethods.at(interpolationMethod_));  //change get_parameter input layer name
  }
  auto interpolation_map_msg = grid_map::GridMapRosConverter::toMessage(interpolation_map_);
  interpolation_map_pub_->publish(std::move(interpolation_map_msg));
}
}  // namespace robotx_costmap_calculator
RCLCPP_COMPONENTS_REGISTER_NODE(robotx_costmap_calculator::CostmapInterpolationComponent)
