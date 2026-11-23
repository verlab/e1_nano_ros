from glob import glob

from setuptools import setup

package_name = 'e1_mocap'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='VeRLab',
    maintainer_email='verlab@dcc.ufmg.br',
    description='OptiTrack NatNet motion capture for E1 Nano swarms',
    license='Apache-2.0',
    entry_points={'console_scripts': ['natnet = e1_mocap.natnet_node:main']},
)
