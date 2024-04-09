map: 
	ros2 launch nav2_bringup localization_launch.py map:=./maps/hallway_map.yaml map_subscribe_transient_local:=true

orbslam:
	ros2 run orbslam3 mono /home/ole/Dev/project-amado/orb_slam3_ros2_ws/orb_slam3_ros2-main/vocabulary/ORBvoc.txt /home/ole/Dev/project-amado/orb_slam3_ros2_ws/orb_slam3_ros2-main/config/monocular/TELLO.yaml

pointcloud: 
	ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -p max_height:=3.0

toolbox:
	ros2 launch slam_toolbox online_async_launch.py

laserscan: 
	ros2 run depthimage_to_laserscan depthimage_to_laserscan_node
