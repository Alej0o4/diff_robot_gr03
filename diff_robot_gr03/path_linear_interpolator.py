#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

def euler_to_quaternion(yaw, pitch=0.0, roll=0.0):
    """Convierte un ángulo Yaw (Euler) a un Cuaternión."""
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy; x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy; z = cr * cp * sy - sr * sp * cy
    return x, y, z, w

class PathLinearInterpolatorNode(Node):
    def __init__(self):
        super().__init__('path_linear_interpolator')
        
        # Parámetro: cuántos puntos por CADA segmento (ej. 50 puntos entre (0,0) y (X,0))
        self.declare_parameter('num_points_per_segment', 50)
        self.num_points = self.get_parameter('num_points_per_segment').get_parameter_value().integer_value
        
        latched_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Suscriptor al path "con esquinas" (del dxf_parser)
        self.raw_path_sub = self.create_subscription(
            Path,
            '/dxf_path_pub',
            self.raw_path_callback,
            latched_qos_profile)
            
        # Publicador para el nuevo path "lineal"
        self.linear_path_pub = self.create_publisher(
            Path,
            '/linear_path', # <-- Nuevo topic
            latched_qos_profile)
            
        self.get_logger().info(f"Nodo LinearInterpolator iniciado. {self.num_points} puntos por segmento.")
        self.get_logger().info("Escuchando en '/dxf_path_pub' y publicando en '/linear_path'")

    def raw_path_callback(self, msg: Path):
        """Recibe el path crudo y publica una versión lineal interpolada."""
        
        if len(msg.poses) < 2:
            self.get_logger().warn(f"Path recibido con {len(msg.poses)} puntos. Se necesitan al menos 2. Ignorando.")
            return

        x_raw = [pose.pose.position.x for pose in msg.poses]
        y_raw = [pose.pose.position.y for pose in msg.poses]
        
        # Listas para guardar los puntos interpolados
        x_interp = []
        y_interp = []
        
        # Iterar sobre cada SEGMENTO (de 0 a N-1)
        for i in range(len(x_raw) - 1):
            p_start = (x_raw[i], y_raw[i])
            p_end = (x_raw[i+1], y_raw[i+1])
            
            # Usar np.linspace para generar los puntos del segmento
            # endpoint=False -> No incluir el punto final (para evitar duplicados)
            x_segment = np.linspace(p_start[0], p_end[0], self.num_points, endpoint=False)
            y_segment = np.linspace(p_start[1], p_end[1], self.num_points, endpoint=False)
            
            x_interp.extend(x_segment)
            y_interp.extend(y_segment)

        # Añadir el último punto (que fue excluido por endpoint=False)
        x_interp.append(x_raw[-1])
        y_interp.append(y_raw[-1])
        
        # --- Re-empacar en un mensaje Path ---
        linear_path_msg = Path()
        linear_path_msg.header.stamp = self.get_clock().now().to_msg()
        linear_path_msg.header.frame_id = msg.header.frame_id
        
        total_points = len(x_interp)
        
        for i in range(total_points):
            pose = PoseStamped()
            pose.header = linear_path_msg.header
            pose.pose.position.x = x_interp[i]
            pose.pose.position.y = y_interp[i]
            pose.pose.position.z = 0.0
            
            # Calcular Orientación
            if i < total_points - 1:
                dx = x_interp[i+1] - x_interp[i]
                dy = y_interp[i+1] - y_interp[i]
            else:
                # Reusar la orientación anterior para el último punto
                dx = x_interp[i] - x_interp[i-1]
                dy = y_interp[i] - y_interp[i-1]
                
            yaw = math.atan2(dy, dx)
            qx, qy, qz, qw = euler_to_quaternion(yaw)
            
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            
            linear_path_msg.poses.append(pose)
            
        self.linear_path_pub.publish(linear_path_msg)
        self.get_logger().info(f"Path lineal con {total_points} puntos publicado en '/linear_path'")

def main(args=None):
    rclpy.init(args=args)
    node = PathLinearInterpolatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()