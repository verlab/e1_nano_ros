"""Show one E1 Nano in RViz with joint sliders for the rotors."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('e1_description')
    with open(os.path.join(share, 'urdf', 'e1_nano.urdf')) as f:
        desc = f.read()
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': desc}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'rviz', 'e1.rviz')]),
    ])
