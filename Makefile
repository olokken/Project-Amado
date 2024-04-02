map: 
	ros2 launch nav2_bringup localization_launch.py map:=./maps/hallway_map.yaml map_subscribe_transient_local:=true

orbslam:
	ros2 run orbslam3 mono /home/ole/Dev/project-amado/orb_slam3_ros2_ws/orb_slam3_ros2-main/vocabulary/ORBvoc.txt /home/ole/Dev/project-amado/orb_slam3_ros2_ws/orb_slam3_ros2-main/config/monocular/TELLO.yaml
	