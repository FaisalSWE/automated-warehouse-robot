from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('warehouse_robot')
    urdf_path = os.path.join(package_dir, 'code/simulation/warehouse_robot.urdf')
    world_path = os.path.join(package_dir, 'code/simulation/warehouse.world')

    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='gzserver',
            arguments=['--verbose', world_path],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='gzclient',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            arguments=[urdf_path],
            output='screen'
        ),
        Node(
            package='warehouse_robot',
            executable='nav_node.py',
            output='screen'
        )
    ])
