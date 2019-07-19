cmake_minimum_required(VERSION 2.8.3)
project(robotx_costmap_calculator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  grid_map_core
  grid_map_ros
  grid_map_cv
  grid_map_filters
  grid_map_loader
  grid_map_msgs
  grid_map_octomap
  grid_map_rviz_plugin
  grid_map_visualization
  nodelet
  pcl_ros
  pluginlib
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES robotx_costmap_calculator
  CATKIN_DEPENDS grid_map nodelet pcl_ros pluginlib roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(ROBOTX_COSTMAP_CALCULATOR_SRC
  src/robotx_costmap_calculator.cpp
  src/object_layer.cpp
  src/base_layer.cpp
  src/costmap.cpp
)

add_library(robotx_costmap_calculator
  ${ROBOTX_COSTMAP_CALCULATOR_SRC}
)
target_link_libraries(robotx_costmap_calculator ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
class_loader_hide_library_symbols(robotx_costmap_calculator)

add_executable(robotx_costmap_calculator_node
  src/robotx_costmap_calculator_node.cpp
)
target_link_libraries(robotx_costmap_calculator_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
  TARGETS robotx_costmap_calculator robotx_costmap_calculator_node
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)