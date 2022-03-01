# robotx_costmap_calculator
subscribe
point_cloud2 or laser_scan

publish
/grid_map(grid_map::msgs)

複数のレイヤーを作成して、時系列処理を行う
point_layer:時刻tkのPoint_cloudからの情報を表示
laser_layer:時刻tkのlaser_scanからの情報を表示
