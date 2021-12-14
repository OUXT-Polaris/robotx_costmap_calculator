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
find_package(ament_cmake_core REQUIRED)
ament_auto_find_build_dependencies()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(pcl_apps_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)




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




add_library(costmap_calculator_component SHARED
  src/base_layer.cpp
  src/costmap.cpp
  src/costmap_calculator_component.cpp
  )

target_compile_definitions(costmap_calculator_component
  PRIVATE "COSTMAP_CALCULATOR_COMPONENT_BUILDING_DLL")
ament_target_dependencies(costmap_calculator_component
  rclcpp
  rclcpp_components
  sensor_msgs
  grid_map_msgs
  grid_map_core
  grid_map_ros
  pcl_apps_msgs
  pcl_conversions
  PCL
  )

add_executable(costmap_calculator_node
  src/costmap_calculator_node.cpp
)
rclcpp_components_register_nodes(costmap_calculator_component
 "costmap_calculator_component::CostmapCalculatorComponent")


target_link_libraries(costmap_calculator_node
 costmap_calculator_component ${PCL_LIBRARIES})
ament_target_dependencies(costmap_calculator_node
  rclcpp
  rclcpp_components
  sensor_msgs
  grid_map_core
  grid_map_msgs
  grid_map_ros
  pcl_apps_msgs
  pcl_conversions
  )


 foreach(target ${target_list})
   target_compile_options(${target} PRIVATE "SHELL:--param gcc-min-expand=1")
   target_compile_options(${target} PRIVATE "SHELL:--param gcc-min-heapsize=32768")
 endforeach()
 

install(TARGETS
  costmap_calculator_node
  DESTINATION lib/robotx_costmap_calculator
)

install(TARGETS costmap_calculator_component 
EXPORT export_base_layer
EXPORT export_costmap
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_export_libraries(costmap_calculator_component)
install(
  DIRECTORY "include/"
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()



ament_lint_auto_find_test_dependencies()
ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_components)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(grid_map_msgs)
ament_export_dependencies(grid_map_core)
ament_export_dependencies(grid_map_ros)
ament_export_dependencies(pcl_apps_msgs)
ament_export_dependencies(pcl_conversions)
ament_export_dependencies(PCL)


ament_package()

