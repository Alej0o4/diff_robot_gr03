from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('diff_robot_gr03')
    
    # --- [CAMBIO] ---
    # Ahora tenemos UN solo archivo de parámetros para todos los nodos
    params_file = os.path.join(pkg_share, 'config', 'trajectory_params.yaml')

    # Nodo 1: El "Proveedor"
    dxf_exporter_node = Node(
        package='diff_robot_gr03',
        executable='dxf_exporter_node_v2', 
        name='dxf_exporter_node',          
        output='screen',
        parameters=[params_file] # <-- Carga desde el archivo
    )

    # Nodo 2: El "Suavizador"
    path_smoother_node = Node(
        package='diff_robot_gr03',
        executable='path_smoother_node',
        name='path_smoother_node',
        output='screen',
        parameters=[params_file] # <-- Carga desde el archivo
    )
    
    # Nodo 3: El "Perseguidor"
    pure_pursuit_controller = Node(
        package='diff_robot_gr03',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[params_file] # <-- Carga desde el archivo
    )
    path_linear_interpolator_node = Node(
        package='diff_robot_gr03',
        executable='path_linear_interpolator',
        name='path_linear_interpolator',
        output='screen',
        parameters=[params_file] 
    )

    autonomy_gate_node = Node(
        package='diff_robot_gr03',
        executable='autonomy_gate_node',
        name='autonomy_gate_node',
        parameters=[
            # (Puedes sobreescribir parámetros aquí si lo deseas)
            # {'enable_button_idx': 0} # 0=A, 1=B, 2=X, 3=Y
        ],
        # Asegúrate de que los topics coincidan:
        remappings=[
            # Escucha a pure_pursuit
            ('cmd_vel_in_topic', '/nav_vel'), 
            # Escucha el joystick (asumiendo que joy_bridge publica en /joy)
            ('joy_topic', '/joy'),
            # Publica al twist_mux
            ('cmd_vel_out_topic', '/nav_vel_gated')
        ],
        output='screen'
    )

    return LaunchDescription([
        dxf_exporter_node,
        #path_smoother_node,
        autonomy_gate_node,
        path_linear_interpolator_node,
        pure_pursuit_controller
    ])