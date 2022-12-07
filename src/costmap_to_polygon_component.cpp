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

#include <boost/algorithm/clamp.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <chrono>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <robotx_costmap_calculator/costmap_to_polygon_component.hpp>
#include <string>
#include <unordered_map>
#include <vector>

typedef double coord2_t;

namespace robotx_costmap_calculator
{
CostmapToPolygonComponent::CostmapToPolygonComponent(const rclcpp::NodeOptions & options)
: Node("robotx_costmap_to_polygon", options), buffer_(get_clock()), listener_(buffer_)
{
  std::string grid_map_topic;
  declare_parameter<std::string>("grid_map_topic", "/perception/interpolation_grid_map");
  get_parameter("grid_map_topic", grid_map_topic);
  declare_parameter("output_frame_id", "base_link");
  get_parameter("output_frame_id", output_frame_id_);
  declare_parameter("visualize_frame_id", "map");
  get_parameter("visualize_frame_id", visualize_frame_id_);
  declare_parameter("max_segment_distance", 2.0);
  get_parameter("max_segment_distance", max_segment_distance_);
  declare_parameter("min_segment_distance", 0.1);
  get_parameter("min_segment_distance", min_segment_distance_);
  declare_parameter("distance_ratio", 0.5);
  get_parameter("distance_ratio", distance_ratio_);

  //subscriber
  grid_map_sub_ = create_subscription<grid_map_msgs::msg::GridMap>(
    grid_map_topic, 1,
    std::bind(&CostmapToPolygonComponent::gridmapCallback, this, std::placeholders::_1));

  grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>("~/roundcheck_grid_map", 1);

  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("~/marker", 1);
}

void CostmapToPolygonComponent::gridmapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  grid_map::GridMap map;
  geometry_msgs::msg::Polygon object_area;
  std::vector<geometry_msgs::msg::Point32> points;
  grid_map::GridMapRosConverter::fromMessage(*msg, map);
  double resolution = map.getResolution();
  map.add("round_check", 0.0);
  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    grid_map::Position center_position;
    map.getPosition(*iterator, center_position);
    for (grid_map::SpiralIterator spiral_iterator(map, center_position, resolution);
         !spiral_iterator.isPastEnd(); ++spiral_iterator) {
      grid_map::Position position;
      map.getPosition(*spiral_iterator, position);
      if (std::isnan(map.at("interpolation_layer", *spiral_iterator))) {
        map.at("round_check", *spiral_iterator) = 0.0;
      } else {
        if (map.at("interpolation_layer", *spiral_iterator) > 0.0) {
          map.at("round_check", *spiral_iterator) = 1.0;
        }
      }
    }
    if (map.at("round_check", *iterator) == 1.0) {
      grid_map::Position center_position;
      map.getPosition(*iterator, center_position);
      geometry_msgs::msg::Point32 p;
      p.x = center_position(0);
      p.y = center_position(1);
      object_area.points.push_back(p);
      points.push_back(p);
    }
  }
  if (points.size() == 0) {
    return;
  }
  auto polygons = getPolygons(points);
  if (polygons) {
    if (previous_marker_size_ > polygons.get().size()) {
      marker_pub_->publish(generateDeleteMarker());
    }
    auto marker = generateMarker(polygons.get(), msg->header);
    marker_pub_->publish(marker);
    previous_marker_size_ = marker.markers.size();
  }
  auto marker = generateMarker(polygons.get(), msg->header);
  marker_pub_->publish(marker);
  auto check_msg = grid_map::GridMapRosConverter::toMessage(map);
  check_msg->header.stamp = get_clock()->now();
  grid_map_pub_->publish(std::move(check_msg));
  return;
}

boost::optional<geometry_msgs::msg::PointStamped> CostmapToPolygonComponent::transform(
  geometry_msgs::msg::PointStamped point, std::string target_frame_id, bool exact)
{
  if (point.header.frame_id == target_frame_id) {
    return point;
  }
  if (exact) {
    tf2::TimePoint time_point = tf2::TimePoint(
      std::chrono::seconds(point.header.stamp.sec) +
      std::chrono::nanoseconds(point.header.stamp.nanosec));
    try {
      geometry_msgs::msg::TransformStamped transform_stamped = buffer_.lookupTransform(
        target_frame_id, point.header.frame_id, time_point, tf2::durationFromSec(1.0));
      tf2::doTransform(point, point, transform_stamped);
      return point;
    } catch (...) {
      return boost::none;
    }
  } else {
    tf2::TimePoint time_point =
      tf2::TimePoint(std::chrono::seconds(0) + std::chrono::nanoseconds(0));
    try {
      geometry_msgs::msg::TransformStamped transform_stamped = buffer_.lookupTransform(
        target_frame_id, point.header.frame_id, time_point, tf2::durationFromSec(1.0));
      tf2::doTransform(point, point, transform_stamped);
      return point;
    } catch (...) {
      return boost::none;
    }
  }
}
double cross(
  const geometry_msgs::msg::Point32 & O, const geometry_msgs::msg::Point32 & p0,
  const geometry_msgs::msg::Point32 & p1)
{
  return (p0.x - O.x) * (p1.y - O.y) - (p0.y - O.y) * (p1.x - O.x);
}

boost::optional<std::vector<geometry_msgs::msg::Polygon>> CostmapToPolygonComponent::getPolygons(
  const std::vector<geometry_msgs::msg::Point32> points)
{
  std::vector<geometry_msgs::msg::Polygon> polygons;
  geometry_msgs::msg::Polygon poly;
  std::vector<bool> is_connected;
  unsigned int num_points = points.size();
  if (num_points <= 3) {
    return polygons;
  }
  for (unsigned int i = 0; i < num_points; i++) {
    geometry_msgs::msg::Point32 p0, p1, p;
    if (i == (num_points - 1)) {
      p0 = points[num_points - 1];
      p1 = points[0];
    } else {
      p0 = points[i];
      p1 = points[i + 1];
    }
    double l = std::hypot(p0.x - p1.x, p0.y - p1.y);
    double d0 = std::hypot(p0.x, p0.y);
    double d1 = std::hypot(p1.x, p1.y);
    double dist_threashold = boost::algorithm::clamp(
      std::min(d0, d1) * distance_ratio_, min_segment_distance_, max_segment_distance_);
    if (l > dist_threashold) {
      is_connected.push_back(false);
      continue;
    }
    is_connected.push_back(true);
  }
  std::vector<geometry_msgs::msg::Polygon> connect_polygons;
  std::vector<geometry_msgs::msg::Point32> buf;
  for (unsigned int i = 0; i < num_points; i++) {
    buf.push_back(points[i]);
    if (!is_connected[i]) {
      geometry_msgs::msg::Polygon connect_poly;
      connect_poly.points = buf;
      connect_polygons.push_back(connect_poly);
      buf.clear();
    }
  }
  if (is_connected[is_connected.size() - 1] && connect_polygons.size() > 0) {
    std::vector<geometry_msgs::msg::Point32> start_poly = connect_polygons[0].points;
    std::vector<geometry_msgs::msg::Point32> end_poly =
      connect_polygons[connect_polygons.size() - 1].points;
    end_poly.insert(end_poly.end(), start_poly.begin(), start_poly.end());
    connect_polygons.erase(connect_polygons.begin());
    connect_polygons.erase(connect_polygons.end());
    geometry_msgs::msg::Polygon poly;
    poly.points = end_poly;
    connect_polygons.push_back(poly);
  }
  unsigned int num_polygon = connect_polygons.size();
  for (unsigned int j = 0; j < num_polygon; j++) {
    std::vector<geometry_msgs::msg::Point32> costmap_points = connect_polygons[j].points;
    size_t k = 0;
    size_t Convex_num_points = costmap_points.size();
    if (Convex_num_points <= 3) {
      return polygons;
    }
    std::vector<geometry_msgs::msg::Point32> Convex_hull_point(2 * Convex_num_points);
    for (unsigned int i = 0; i < Convex_num_points; i++) {
      while (k > 2 &&
             cross(Convex_hull_point[k - 2], Convex_hull_point[k - 1], costmap_points[i]) <= 0)
        k--;
      Convex_hull_point[k++] = costmap_points[i];
      // }
    }
    for (unsigned int i = Convex_num_points - 1, t = k + 1; i > 0; --i) {
      while (k >= t &&
             cross(Convex_hull_point[k - 2], Convex_hull_point[k - 1], costmap_points[i - 1]) <= 0)
        k--;
      Convex_hull_point[k++] = costmap_points[i - 1];
      // }
    }
    Convex_hull_point.resize(k - 1);
    poly.points = Convex_hull_point;
    polygons.push_back(poly);
    costmap_points.clear();
  }
  return polygons;
}

visualization_msgs::msg::MarkerArray CostmapToPolygonComponent::generateDeleteMarker()
{
  visualization_msgs::msg::MarkerArray ret;
  visualization_msgs::msg::Marker marker;
  marker.action = marker.DELETEALL;
  ret.markers.push_back(marker);
  return ret;
}

visualization_msgs::msg::MarkerArray CostmapToPolygonComponent::generateMarker(
  std::vector<geometry_msgs::msg::Polygon> polygons, std_msgs::msg::Header header)
{
  visualization_msgs::msg::MarkerArray marker_array;
  auto stamp = get_clock()->now();
  header.stamp = stamp;
  int count = 0;
  for (auto poly_itr = polygons.begin(); poly_itr != polygons.end(); poly_itr++) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.header.frame_id = visualize_frame_id_;
    marker.ns = "polygon";
    marker.id = count;
    marker.type = marker.LINE_STRIP;
    marker.action = marker.ADD;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    for (auto point_itr = poly_itr->points.begin(); point_itr != poly_itr->points.end();
         point_itr++) {
      geometry_msgs::msg::PointStamped p;
      p.point.x = point_itr->x;
      p.point.y = point_itr->y;
      p.point.z = point_itr->z;
      p.header = header;
      p.header.frame_id = output_frame_id_;
      auto p_transformed = transform(p, visualize_frame_id_, false);
      if (p_transformed) {
        marker.points.push_back(p_transformed->point);
        marker.colors.push_back(marker.color);
      } else {
        marker_array = visualization_msgs::msg::MarkerArray();
        return marker_array;
      }
    }
    marker_array.markers.push_back(marker);
    count++;
  }
  return marker_array;
}
// namespace robotx_costmap_calculator
RCLCPP_COMPONENTS_REGISTER_NODE(robotx_costmap_calculator::CostmapToPolygonComponent)
