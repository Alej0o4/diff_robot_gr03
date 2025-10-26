import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import xacro

pkg_folder = 'diff_robot_gr03'
rviz_file = 'rviz_config.rviz'

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_path = os.path.join(get_package_share_directory(pkg_folder))
    xacro_file = os.path.join(pkg_path, 'model', 'robot.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_path, 'model', rviz_file)

    # Procesar el URDF (ya incluye ambos robots)
    robot_description = xacro.process_file(xacro_file).toxml()

    # Configuración del twist_mux
    config_file_path = os.path.join(pkg_path, 'config', 'twist_mux.yaml')
    ekf_config_path = os.path.join(pkg_path, 'config','ekf.yaml')

    joy_params = os.path.join(get_package_share_directory(pkg_folder),'config','joystick.yaml')

    # RViz
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # Un solo robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}]
    )

    # Teleop keyboard
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[('/cmd_vel', '/teleop_vel')]
    )

    # Twist mux
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[config_file_path],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    mr_sensors_test_node = Node(
        package='diff_robot_gr03',
        executable='mr_sensors_test',
        name='mr_sensors_test',
        output='screen',
    )
    
    sensor_bridge = Node(
        package='diff_robot_gr03',
        executable='sensor_bridge_node',
        name='sensor_bridge_node',
        output='screen',
    )

    teleop_node_joy = Node(package='teleop_twist_joy', 
                    executable='teleop_node',
                    name="teleop_node",
                    parameters=[joy_params],
                    remappings=[('/cmd_vel','/cmd_vel_joy')]
    )
    
    joy_node = Node(package='diff_robot_gr03', 
                    executable='joy_bridge_node'
    )

    # Nodo del EKF (robot_localization)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
    )

    position_control = Node(
        package='diff_robot_gr03',
        executable='position_controller_node',
        name='position_controller_node',
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        rviz_arg,
        rviz_node,
        node_robot_state_publisher,
        #mr_sensors_test_node,
        sensor_bridge,
        ekf_node,
        #teleop_node,
        joy_node,
        teleop_node_joy,
        twist_mux_node,
        position_control
    ])
