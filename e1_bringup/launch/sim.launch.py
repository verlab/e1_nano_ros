"""E1 Nano simulation (software in the loop) + ROS 2 driver + swarm controller + RViz.

    ros2 launch e1_bringup sim.launch.py num:=4 layout:=grid rviz:=true mission:=formation
Arguments: num, layout (grid|line|circle), spacing, backend (mujoco|genesis), viewer, rviz,
           start_router, zenoh (endpoint), mission (none|square|formation), record (mp4 path)
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from common import rviz_config, state_publishers  # noqa: E402


def setup(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)
    n, zenoh = int(lc('num')), lc('zenoh')
    ids = list(range(1, n + 1))
    actions = []
    if lc('start_router') == 'true':
        actions.append(ExecuteProcess(cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'], output='screen'))
    sim_args = ['-n', str(n), '--layout', lc('layout'), '--spacing', lc('spacing'), '--connect', zenoh, '--backend', lc('backend')]
    if lc('viewer') == 'true':
        sim_args.append('--viewer')
    if lc('record'):
        sim_args += ['--record', lc('record')]
    actions.append(TimerAction(period=1.0, actions=[
        Node(package='e1_core', executable='e1-sim', name='e1_sim', arguments=sim_args, output='screen')]))
    actions.append(Node(package='e1_bridge', executable='e1_bridge', name='e1_bridge', output='screen',
                        parameters=[{'zenoh_connect': zenoh, 'ids': ids}]))
    actions.append(Node(package='e1_control', executable='swarm_controller', name='e1_swarm_controller', output='screen',
                        parameters=[{'ids': ids}]))
    actions += state_publishers(ids)
    if lc('rviz') == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_config(ids)], output='log'))
    m = lc('mission')
    if m == 'square':
        actions.append(TimerAction(period=6.0, actions=[Node(package='e1_control', executable='square_mission', output='screen')]))
    elif m == 'formation':
        actions.append(TimerAction(period=6.0, actions=[Node(package='e1_control', executable='formation_mission', output='screen',
                                                             parameters=[{'n': n}])]))
    return actions


def generate_launch_description():
    args = [('num', '1'), ('layout', 'grid'), ('spacing', '0.5'), ('backend', 'mujoco'), ('viewer', 'false'),
            ('rviz', 'true'), ('start_router', 'true'), ('zenoh', 'tcp/127.0.0.1:7447'), ('mission', 'none'), ('record', '')]
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args] + [OpaqueFunction(function=setup)])
