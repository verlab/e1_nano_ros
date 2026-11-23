"""Real E1 Nano swarm: Zenoh router + ROS 2 driver + swarm controller + RViz.

Drones (firmware with the Zenoh link) connect to this router over WiFi. Poses come from a
motion-capture system and are relayed to the drones' key space.

    # OptiTrack through the built-in NatNet client (e1_mocap), rigid bodies named e1_<id>
    ros2 launch e1_bringup real.launch.py ids:="[1,2,3]" mocap:=natnet server_ip:=192.168.1.10 local_ip:=192.168.1.20
    # any other source publishing geometry_msgs/PoseStamped (vrpn_mocap, mocap4r2, Vicon...)
    ros2 launch e1_bringup real.launch.py ids:="[1]" mocap:=topic mocap_topic:="/vrpn_mocap/e1_{id}/pose"
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from common import rviz_config, state_publishers  # noqa: E402


def setup(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)
    ids = [int(x) for x in lc('ids').strip('[] ').split(',') if x.strip()]
    acts = []
    if lc('start_router') == 'true':
        acts.append(ExecuteProcess(cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'], output='screen',
                                   additional_env={'ZENOH_ROUTER_CONFIG_URI': lc('router_config')} if lc('router_config') else {}))
    mocap, topic = lc('mocap'), lc('mocap_topic')
    if mocap == 'natnet':
        topic = '/e1_{id}/mocap/pose'
        acts.append(Node(package='e1_mocap', executable='natnet', name='e1_mocap', output='screen',
                         parameters=[{'server_ip': lc('server_ip'), 'local_ip': lc('local_ip'),
                                      'multicast': lc('multicast') == 'true', 'up_axis': lc('up_axis')}]))
    elif mocap == 'none':
        topic = ''
    acts.append(Node(package='e1_bridge', executable='e1_bridge', name='e1_bridge', output='screen',
                     parameters=[{'zenoh_connect': lc('zenoh'), 'ids': ids, 'mocap_topic': topic}]))
    acts.append(Node(package='e1_control', executable='swarm_controller', name='e1_swarm_controller', output='screen',
                     parameters=[{'ids': ids}]))
    acts += state_publishers(ids)
    if lc('rviz') == 'true':
        acts.append(Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config(ids)], output='log'))
    return acts


def generate_launch_description():
    args = [('ids', '[1]'), ('mocap', 'natnet'), ('server_ip', '192.168.1.10'), ('local_ip', '0.0.0.0'),
            ('multicast', 'true'), ('up_axis', 'z'), ('mocap_topic', '/vrpn_mocap/e1_{id}/pose'), ('zenoh', 'tcp/127.0.0.1:7447'),
            ('start_router', 'true'), ('router_config', ''), ('rviz', 'true')]
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args] + [OpaqueFunction(function=setup)])
