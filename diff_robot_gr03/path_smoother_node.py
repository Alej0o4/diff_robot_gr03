#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.interpolate import splprep, splev
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

class PathSmootherNode(Node):
    def __init__(self):
        super().__init__('path_smoother_node')
        
        # Parámetro para definir cuántos puntos tendrá el path suave
        self.declare_parameter('num_smooth_points', 200)
        self.declare_parameter('spline_smoothing_factor', 0.0) # 's'
            
        self.num_points = self.get_parameter('num_smooth_points').get_parameter_value().integer_value
        self.spline_smoothing = self.get_parameter('spline_smoothing_factor').get_parameter_value().double_value

        latched_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Suscriptor al path "con esquinas" (del dxf_parser)
        self.raw_path_sub = self.create_subscription(
            Path,
            '/dxf_path_pub',  # El topic que publica el dxf_parser_node
            self.raw_path_callback,
            latched_qos_profile)
            
        # Publicador para el nuevo path "suave"
        self.smooth_path_pub = self.create_publisher(
            Path,
            '/smooth_path', # El nuevo topic para RViz
            latched_qos_profile)
            
        self.get_logger().info(f"Nodo PathSmoother iniciado. Suavizando a {self.num_points} puntos.")
        self.get_logger().info(f"==> Factor de suavizado (s): {self.spline_smoothing}")
        self.get_logger().info("Escuchando en '/dxf_path_pub' y publicando en '/smooth_path'")

    def raw_path_callback(self, msg: Path):
        """Recibe el path crudo y publica una versión suave."""
        
        # 1. Verificar si el path es válido
        if len(msg.poses) < 3:
            self.get_logger().warn(f"Path recibido con {len(msg.poses)} puntos. Se necesitan al menos 3 para el spline. Ignorando.")
            return

        # 2. Desempacar los puntos (X, Y)
        x_raw = [pose.pose.position.x for pose in msg.poses]
        y_raw = [pose.pose.position.y for pose in msg.poses]
        
        # --- [CORRECCIÓN] ---
        # El bloque 'if' que añadía x_raw[1] y y_raw[1] ha sido eliminado.
        # Ya no es necesario y era la causa del bug.
        # --- [FIN CORRECCIÓN] ---

        # Convertir a un formato que SciPy entiende
        points = [x_raw, y_raw]
        
        try:
            # 3. Calcular el B-Spline Cúbico (k=3)
            # Con la lista de 5 puntos (0,0 -> ... -> 0,0),
            # splprep generará un spline que empieza y termina en (0,0).
            tck, u = splprep(points, k=3, s=self.spline_smoothing)
            
            # 4. Muestrear (samplear) el spline
            # Creamos los nuevos valores 't' (de 0.0 a 1.0)
            u_smooth = np.linspace(0, 1, self.num_points)
            
            # Evaluamos el spline en esos nuevos puntos
            # (x_smooth, y_smooth)
            x_smooth, y_smooth = splev(u_smooth, tck)

        except Exception as e:
            self.get_logger().error(f"No se pudo calcular el spline: {e}")
            return

        # 5. Re-empacar los puntos en un mensaje Path
        smooth_path_msg = Path()
        smooth_path_msg.header.stamp = self.get_clock().now().to_msg()
        smooth_path_msg.header.frame_id = msg.header.frame_id # Usar el mismo frame_id ("odom")
        
        for i in range(self.num_points):
            pose = PoseStamped()
            pose.header = smooth_path_msg.header
            
            pose.pose.position.x = x_smooth[i]
            pose.pose.position.y = y_smooth[i]
            pose.pose.position.z = 0.0 # Asumimos 2D
            
            # 6. Calcular la Orientación
            if i < self.num_points - 1:
                dx = x_smooth[i+1] - x_smooth[i]
                dy = y_smooth[i+1] - y_smooth[i]
            else:
                # Para el último punto, reusamos la orientación anterior
                dx = x_smooth[i] - x_smooth[i-1]
                dy = y_smooth[i] - y_smooth[i-1]
                
            yaw = math.atan2(dy, dx)
            qx, qy, qz, qw = euler_to_quaternion(yaw)
            
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            
            smooth_path_msg.poses.append(pose)
            
        # 7. Publicar el path suave
        self.smooth_path_pub.publish(smooth_path_msg)
        self.get_logger().info("Path suave publicado en '/smooth_path'")

def main(args=None):
    rclpy.init(args=args)
    node = PathSmootherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()