"""Helpers shared by the E1 launch files."""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def robot_description():
    with open(os.path.join(get_package_share_directory('e1_description'), 'urdf', 'e1_nano.urdf')) as f:
        return f.read()


def state_publishers(ids, prefix='e1_'):
    desc = robot_description()
    return [Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=f'{prefix}{i}',
                 name='robot_state_publisher', output='log',
                 parameters=[{'robot_description': desc, 'frame_prefix': f'{prefix}{i}/'}]) for i in ids]


def rviz_config(ids, prefix='e1_', world='world'):
    """Write an RViz config with one RobotModel per drone."""
    models = ''.join(f'''
    - Class: rviz_default_plugins/RobotModel
      Name: {prefix}{i}
      Enabled: true
      Description Source: Topic
      Description Topic:
        Value: /{prefix}{i}/robot_description
      TF Prefix: {prefix}{i}
''' for i in ids)
    cfg = f'''Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Cell Size: 0.25
      Plane Cell Count: 24
      Enabled: true
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Marker Scale: 0.08
      Show Names: false
    - Class: rviz_default_plugins/MarkerArray
      Name: Setpoints
      Enabled: true
      Topic:
        Value: /e1/setpoints
{models}
  Global Options:
    Fixed Frame: {world}
    Frame Rate: 30
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3.0
      Focal Point: {{X: 0, Y: 0, Z: 0.3}}
      Pitch: 0.6
      Yaw: 0.9
'''
    fd, path = tempfile.mkstemp(prefix='e1_', suffix='.rviz')
    with os.fdopen(fd, 'w') as f:
        f.write(cfg)
    return path
