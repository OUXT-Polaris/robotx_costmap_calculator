# Copyright (c) 2020 OUXT Polaris
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    grid_map_demos_dir = get_package_share_directory('robotx_costmap_calculator')
    #visualization_config_file = LaunchConfiguration('visualization_config')
    
    pointcloud_to_gridmap_node = Node(
            package='robotx_costmap_calculator',
            executable='costmap_calculator_node',
            name='costmap_calculator_node',
            output='screen')

    """
    declare_visualization_config_file_cmd = DeclareLaunchArgument(
        'visualization_config',
        default_value=os.path.join(
            grid_map_demos_dir, 'config', 'pc_to_grid.yaml'),
        description='Full path to the Gridmap visualization config file to use')
   
    grid_map_visualization_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[visualization_config_file]
    )
    """

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments to the launch description
    #ld.add_action(declare_visualization_config_file_cmd)

    ld.add_action(pointcloud_to_gridmap_node)

    # Add node actions to the launch description
    #ld.add_action(grid_map_visualization_node)

    return ld