"""Locate the e1_description assets (ROS share dir, env var, or source checkout)."""
import os


def description_dir():
    env = os.environ.get('E1_DESCRIPTION_DIR')
    if env and os.path.isdir(env):
        return env
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('e1_description')
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    for up in range(1, 6):
        cand = os.path.normpath(os.path.join(here, *(['..'] * up), 'e1_description'))
        if os.path.isdir(os.path.join(cand, 'mjcf')):
            return cand
    raise FileNotFoundError('e1_description not found: set E1_DESCRIPTION_DIR or source the ROS workspace')


def find_mjcf():
    return os.path.join(description_dir(), 'mjcf', 'e1_nano.xml')


def find_urdf(kind='standalone'):
    return os.path.join(description_dir(), 'urdf', f'e1_nano_{kind}.urdf' if kind != 'ros' else 'e1_nano.urdf')
