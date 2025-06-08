from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
    	
        # Load parameters and start the node without rosconsole.config
        Node(
            package='counter_uas_guidance',
            executable='counter_uas_offboard_ctrl_node',
            name='counter_uas_offboard_ctrl_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([FindPackageShare('counter_uas_guidance'), 'config', 'params.yaml']),
            ],
            respawn=True
        ),
        #ExecuteProcess(
        #    cmd=['python3', '/home/jhlee/colcon_ws/src/kari_dronecop_imgproc/scripts/sahi_tensorrt.py'],
        #    name='sahi_tensorrt_script',
        #    output='screen'
        #),
    ])
