from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'diff_robot_gr03'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name,'launch'), glob("launch/*.launch.py")),
        (os.path.join("share", package_name,'model'), glob("model/*.*")),
        (os.path.join("share", package_name,'config'), glob("config/*.*")),
        (os.path.join("share", package_name,'csv'), glob("config/*.csv")),
        (os.path.join("share", package_name,'dxf'), glob("config/*.dxf")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alejo',
    maintainer_email='alejo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_bridge_node=diff_robot_gr03.joy_bridge_node:main',
            'mr_sensors_test=diff_robot_gr03.mr_sensors_test:main',
            'mr_sensors_test_rozo=diff_robot_gr03.mr_sensors_test_rozo:main',
            'position_controller_node=diff_robot_gr03.position_controller:main',
            'sensor_bridge_node=diff_robot_gr03.sensor_bridge_node:main',
            'position_controller_node_v2=diff_robot_gr03.position_controller_v2:main',
            'path_publisher_node=diff_robot_gr03.path_publisher_node:main',
            'dxf_exporter_node_v2=diff_robot_gr03.dxf_exporter_node_v2:main',
            'trajectory_generator_node=diff_robot_gr03.trajectory_generator_node:main',
        ],
    },
)
