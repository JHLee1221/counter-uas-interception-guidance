from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Load parameters and start the node
        Node(
            package='siyi_eo_ros2',
            executable='siyi_eo_status_node',
            name='siyi_eo_status_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('siyi_eo_ros2'), 'config', 'camera_status_params.yaml']),
            ],
            respawn=True
        ),
    ])
