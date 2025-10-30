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

    # Configuración de parametros
    config_file_path = os.path.join(pkg_path, 'config', 'twist_mux.yaml')
    ekf_config_path = os.path.join(pkg_path, 'config','ekf.yaml')
    joy_params = os.path.join(pkg_path,'config','joystick.yaml')
    controller_params_path = os.path.join(pkg_path, 'config', 'controller_params.yaml')

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

    # Twist mux
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[config_file_path],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
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
        executable='position_controller_node_v2',
        name='position_controller_node_v2',
        output='screen',
        parameters=[controller_params_path]
    )

    path_publisher_ekf = Node(
        package='diff_robot_gr03',
        executable='path_publisher_node',
        name='path_publisher_ekf',
        output='screen',
        parameters=[{
            'odom_topic': '/odometry/filtered', # <- Le decimos que dibuje el path del EKF
            'max_path_size': 1000 # Un buffer de 500 puntos
        }]
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
        sensor_bridge,
        ekf_node,
        path_publisher_ekf,
        joy_node,
        teleop_node_joy,
        twist_mux_node,
        #position_control
    ])
