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
        (os.path.join("share", package_name,'csv'), glob("csv/*.csv")),
        (os.path.join("share", package_name,'dxf'), glob("dxf/*.dxf")),
        (os.path.join("share", package_name,'dxf'), glob("old_versions/*.*")),
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
            #Joystick related nodes
            'joy_bridge_node=diff_robot_gr03.joy_bridge_node:main',
            'autonomy_gate_node = diff_robot_gr03.autonomy_gate_node:main',
            #Sensor related nodes
            'sensor_bridge_node=diff_robot_gr03.sensor_bridge_node:main',
            'complementary_filter_node = diff_robot_gr03.complementary_filter_node:main',
            #Trajectory related nodes
            'dxf_exporter_node_v2=diff_robot_gr03.dxf_exporter_node_v2:main',
            'path_linear_interpolator = diff_robot_gr03.path_linear_interpolator:main',
            'pure_pursuit_controller = diff_robot_gr03.pure_pursuit_controller:main',
            'position_controller_node_v2=diff_robot_gr03.position_controller_v2:main',
            #Path publishing node 
            'path_publisher_node=diff_robot_gr03.path_publisher_node:main',
            # Metrics and analysis
            'pose_error_analyzer = diff_robot_gr03.pose_error_analyzer:main',
        ],
    },
)
