from setuptools import find_packages, setup

package_name = 'e1_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'requirements.txt']),
    ],
    install_requires=['setuptools', 'numpy'],
    extras_require={'sim': ['mujoco>=3.2', 'imageio', 'imageio-ffmpeg'], 'zenoh': ['eclipse-zenoh==1.10.1'],
                    'genesis': ['genesis-world', 'torch']},
    zip_safe=True,
    maintainer='VeRLab',
    maintainer_email='verlab@dcc.ufmg.br',
    description='E1 Nano core: protocol, firmware model, control, Zenoh client, simulators',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'e1-sim = e1_core.sim.cli:main',
        'e1-cli = e1_core.cli:main',
    ]},
)
