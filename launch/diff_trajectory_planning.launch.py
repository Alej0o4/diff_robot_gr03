from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Obtiene automáticamente la ruta correcta al paquete
    pkg_share = get_package_share_directory('diff_robot_gr03')
    config_file = os.path.join(pkg_share, 'config', 'trajectory_params.yaml')

    #  DXF Parser Node
    dxf_exporter_node = Node(
        package='diff_robot_gr03',
        executable='dxf_exporter_node_v2',
        name='dxf_exporter_node',
        output='screen',
        parameters=[config_file]
    )

    #  Trajectory Planner Node
    trajectory_node = Node(
        package='diff_robot_gr03',
        executable='trajectory_generator_node',
        name='trajectory_generator_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        trajectory_node,
        dxf_exporter_node
    ])
