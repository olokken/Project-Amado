from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        # Tello driver node
        Node(
            package='tello',
            executable='tello',
            output='screen',
            namespace='/',
            name='tello',
            parameters=[
                {'connect_timeout': 10.0},
                {'tello_ip': '192.168.10.1'},
                {'tf_base': 'map'},
                {'tf_drone': 'drone'}
            ],
            remappings=[
                ('/image_raw', '/camera')
            ],
            respawn=True
        ),

        # Tello control node
        Node(
            package='tello_control',
            executable='tello_control',
            namespace='/',
            name='control',
            output='screen',
            respawn=False
        ),

        # RQT topic debug tool
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            output='screen',
            namespace='/',
            name='rqt',
            respawn=False
        ),

        # RViz data visualization tool
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            namespace='/',
            name='rviz2',
            respawn=True,
            arguments=['-d', '/home/ole/Dev/project-amado/tentone-tello-ros2-main/workspace/src/rviz.rviz']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            namespace='/',
            name='static_tf_pub_drone_to_base_link',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'base_link', 'drone'], 
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_camera',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'drone', 'camera_depth_frame'],        
        ),

        # SLAM Toolbox node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            output='screen',
            namespace='/',
            name='slam_toolbox',
            parameters=['/home/ole/Dev/project-amado/configs/slam_toolbox.yaml']
        ),
         Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            parameters=[
                {'range_min': 0.1},
                {'range_max': 10.0},
                {'scan_time': 0.033},
                {'output_frame_id': 'camera_depth_frame'},
            ],
            remappings=[
                ('/depth/image_rect_raw', '/camera/depth/image_raw'),
                ('/depth/camera_info', '/camera/depth/camera_info')
            ]
        ),         
    ]

    return LaunchDescription(nodes)
