nav2: 
	ros2 launch nav2_bringup navigation_launch.py

orbslam:
	ros2 run orbslam3 mono /home/ole/Dev/project-amado/orb_slam3_ros2_ws/orb_slam3_ros2-main/vocabulary/ORBvoc.txt /home/ole/Dev/project-amado/orb_slam3_ros2_ws/orb_slam3_ros2-main/config/monocular/TELLO.yaml

pointcloud: 
	ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -p max_height:=3.0

toolbox:
	ros2 run slam_toolbox sync_slam_toolbox_node --ros-args -p base_frame:='base_link'

laserscan: 
	ros2 run depthimage_to_laserscan depthimage_to_laserscan_node
