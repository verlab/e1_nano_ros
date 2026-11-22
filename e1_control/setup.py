from glob import glob

from setuptools import setup

package_name = 'e1_control'
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
    description='Position control and missions for E1 Nano swarms',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'swarm_controller = e1_control.swarm_controller:main',
        'square_mission = e1_control.missions:square_main',
        'formation_mission = e1_control.missions:formation_main',
    ]},
)
