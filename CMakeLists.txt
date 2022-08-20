cmake_minimum_required(VERSION 3.5)
project(robotx_costmap_calculator)

#Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()


#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(cmake/robotx_costmap_calculator-extras.cmake)

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(grid_map_cmake_helpers REQUIRED)
ament_auto_find_build_dependencies()
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${EIGEN3_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})


ament_auto_add_library(costmap_calculator_component SHARED
  src/costmap_calculator_component.cpp)

# add_library(costmap_calculator_component SHARED
#   src/costmap_calculator_component.cpp
#   )

target_compile_definitions(costmap_calculator_component
  PRIVATE "COSTMAP_CALCULATOR_COMPONENT_BUILDING_DLL")
# ament_target_dependencies(costmap_calculator_component
#   rclcpp
#   rclcpp_components
#   sensor_msgs
#   grid_map_msgs
#   grid_map_core
#   grid_map_ros
#   pcl_conversions
#   geometry_msgs
#   data_buffer
#   geometry_msgs_data_buffer
#   pcl_apps_msgs
#   )
target_link_libraries(costmap_calculator_component
 ${PCL_LIBRARIES})

# add_executable(costmap_calculator_node
#   src/costmap_calculator_node.cpp
# )

ament_auto_add_executable(costmap_calculator_node
  src/costmap_calculator_node.cpp
)

rclcpp_components_register_nodes(costmap_calculator_component
 "robotx_costmap_calculator::CostmapCalculatorComponent")


# target_link_libraries(costmap_calculator_node
#  costmap_calculator_component)
# ament_target_dependencies(costmap_calculator_node
#   rclcpp
#   rclcpp_components
#   sensor_msgs
#   grid_map_core
#   grid_map_msgs
#   grid_map_ros
#   pcl_conversions
#   geometry_msgs
#   data_buffer
#   geometry_msgs_data_buffer
#   pcl_apps_msgs
#   )

ament_auto_add_library(costmap_to_polygon_component SHARED
  src/costmap_to_polygon_component.cpp)

# add_library(costmap_to_polygon_component SHARED
#   src/costmap_to_polygon_component.cpp
# )

target_compile_definitions(costmap_to_polygon_component
  PRIVATE "COSTMAP_TO_POLYGON_COMPONENT_BUILDING_DLL")
# ament_target_dependencies(costmap_to_polygon_component
#   rclcpp
#   rclcpp_components
#   grid_map_msgs
#   grid_map_ros
#   pcl_apps_msgs
# )

ament_auto_add_executable(costmap_to_polygon_node
  src/costmap_to_polygon_node.cpp
)
# add_executable(costmap_to_polygon_node
#   src/costmap_to_polygon_node.cpp
# )

# target_link_libraries(costmap_to_polygon_node
# costmap_to_polygon_component)
# ament_target_dependencies(costmap_to_polygon_node
#   rclcpp
#   rclcpp_components
#   grid_map_msgs
#   grid_map_ros
#   pcl_apps_msgs)

rclcpp_components_register_nodes(costmap_to_polygon_component
 "robotx_costmap_calculator::CostmapToPolygonComponent")

# costmap_filter_component
ament_auto_add_library(costmap_filter_component SHARED
  src/costmap_filter_component.cpp)

target_compile_definitions(costmap_filter_component
  PRIVATE "COSTMAP_FILTER_COMPONENT_BUILDING_DLL")

ament_auto_add_executable(costmap_filter_node
  src/costmap_filter_node.cpp
)

rclcpp_components_register_nodes(costmap_filter_component
 "robotx_costmap_calculator::CostmapFilterComponent")

# costmap_interpolation_component
ament_auto_add_library(costmap_interpolation_component SHARED
  src/costmap_interpolation_component.cpp)

target_compile_definitions(costmap_interpolation_component
  PRIVATE "COSTMAP_INTERPOLATION_COMPONENT_BUILDING_DLL")

ament_auto_add_executable(costmap_interpolation_node
  src/costmap_interpolation_node.cpp
)
rclcpp_components_register_nodes(costmap_interpolation_component
 "robotx_costmap_calculator::CostmapInterpolationComponent")

# workaround to allow deprecated header to build on both galactic and humble
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(costmap_calculator_component PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
  target_compile_definitions(costmap_calculator_node PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()

foreach(target ${target_list})
  target_compile_options(${target} PRIVATE "SHELL:--param gcc-min-expand=1")
  target_compile_options(${target} PRIVATE "SHELL:--param gcc-min-heapsize=32768")
endforeach()
 


install(
  DIRECTORY "include/"
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
