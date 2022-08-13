# robotx_costmap_calculator
### Subscribed Topics
#### point_cloud topic
/perception/points_concatenate_node/output(sensor_msgs::msg::PointCloud2)
#### Laser_Scan topic
/perception/pointcloud_to_laserscan_node/output(sensor_msgs::msg::LaserScan)
#### current_pose topic
/current_pose(geometry_msgs::msg::PoseStamped)

### publish Topics
#### /combined_grid_map(grid_map::msgs::msg::GridMap)
各レイヤーの統合トピック
#### /grid_map(grid_map::msgs::msg::GridMap)
PointCloudやLaserScanの表示

### parameters
| Name                         | Type   | Description                                  |default                                         |
| ---------------------------- | ------ | ---------------------------------------------|----------------------------------------------- |
| `resolution `                | double | Map resolution in xy plane [m/cell].         | 1.0                                            |
| `num_grids `                 | int    | Map xy Grid number                           | 20                                             |
| `resolution*num_grids `      | double | Map xy Size [m]                              | 20.0                                           |
| `visialize_frame_id`         | string | visualize rviz                               | map                                            |
| `point_buffer_size`          | int    | circular buffer size in PointCloud           | 2                                              |
| `scan_buffer_size`           | int    | circular buffer size in LaserScan            | 2                                              |

複数のレイヤーを作成して、時系列処理を行う
point_layer:Point_cloudからの情報を表示
scan_layer:laser_scanからの情報を表示
