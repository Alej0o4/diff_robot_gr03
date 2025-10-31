#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math

# --- [NUEVA FUNCIÓN] ---
def euler_to_quaternion(yaw, pitch=0.0, roll=0.0):
    """Convierte un ángulo Yaw (Euler) a un Cuaternión."""
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy; x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy; z = cr * cp * sy - sr * sp * cy
    return x, y, z, w
# --- [FIN NUEVA FUNCIÓN] ---


class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__("trajectory_generator_node")
        
        # ... (parámetros) ...
        self.declare_parameter("desired_velocity", 0.1)
        self.declare_parameter("publish_rate", 10.0)
        
        self.waypoints = []
        self.current_waypoint_index = -1
        self.path_received = False

        # ... (definición de state_qos_profile y event_qos_profile) ...
        state_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        event_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # ... (Subscribers y Publishers no cambian) ...
        self.path_sub = self.create_subscription(Path, "dxf_path_pub", self.dxf_path_callback, state_qos_profile)
        self.waypoint_pub = self.create_publisher(PoseStamped, "goal_waypoint", state_qos_profile)
        self.rviz_path_pub = self.create_publisher(Path, "generated_trajectory_rviz", state_qos_profile)
        self.ready_sub = self.create_subscription(Bool, "controller_ready", self.controller_ready_callback, event_qos_profile)
        self.planner_ready_pub = self.create_publisher(Bool, 'planner_ready', state_qos_profile) 

        self.publish_planner_ready()
        self.get_logger().info(f"Generador de Waypoints iniciado. Esperando DXF en '/dxf_path_pub'...")

    def publish_planner_ready(self):
        self.get_logger().info("Publicando 'planner_ready = True' para el DXFParser.")
        msg = Bool()
        msg.data = True
        self.planner_ready_pub.publish(msg)

    # --- [CALLBACK MODIFICADO] ---
    def dxf_path_callback(self, msg: Path):
        """
        Recibe el path del DXFParser y calcula las orientaciones
        deseadas para cada waypoint.
        """
        if self.path_received or len(msg.poses) < 2:
            if len(msg.poses) < 2:
                self.get_logger().warn("Path recibido con menos de 2 puntos. No se puede procesar.")
            return 
        
        self.get_logger().info(f"¡Path DXF recibido con {len(msg.poses)} waypoints! Calculando orientaciones...")
        self.path_received = True
        
        poses = msg.poses # Obtenemos la lista de poses
        
        # 1. Calcular la orientación para todos los puntos EXCEPTO el último
        for i in range(len(poses) - 1):
            current_point = poses[i].pose.position
            next_point = poses[i+1].pose.position
            
            # Calcular ángulo hacia el siguiente punto
            dx = next_point.x - current_point.x
            dy = next_point.y - current_point.y
            yaw = math.atan2(dy, dx)
            
            # Convertir a cuaternión y guardarlo en la pose actual
            qx, qy, qz, qw = euler_to_quaternion(yaw)
            poses[i].pose.orientation.x = qx
            poses[i].pose.orientation.y = qy
            poses[i].pose.orientation.z = qz
            poses[i].pose.orientation.w = qw

        # 2. Manejar el último waypoint
        # Hacemos que el último punto tenga la misma orientación que el penúltimo
        poses[-1].pose.orientation = poses[-2].pose.orientation
        
        # 3. Guardar la lista de waypoints MODIFICADA
        self.waypoints = poses 
        
        # Publicar la lista de waypoints (con orientaciones) a RViz
        rviz_path_msg = Path()
        rviz_path_msg.header = msg.header
        rviz_path_msg.poses = self.waypoints
        self.rviz_path_pub.publish(rviz_path_msg)
        self.get_logger().info("Orientaciones calculadas. Trayectoria lista para RViz.")
        
        # Reseteamos el índice y solicitamos el primer punto
        self.current_waypoint_index = -1
        self.controller_ready_callback(Bool(data=True)) # Simulamos el primer "ready"
    # --- [FIN CALLBACK MODIFICADO] ---

    def controller_ready_callback(self, msg: Bool):
        """El controlador está listo, enviar el SIGUIENTE waypoint."""
        if not msg.data or not self.path_received:
            return
            
        self.current_waypoint_index += 1
        
        if self.current_waypoint_index < len(self.waypoints):
            # Tenemos más puntos, enviar el siguiente
            next_waypoint = self.waypoints[self.current_waypoint_index]
            next_waypoint.header.stamp = self.get_clock().now().to_msg()
            
            self.waypoint_pub.publish(next_waypoint)
            self.get_logger().info(f"Publicando waypoint {self.current_waypoint_index + 1} / {len(self.waypoints)}")
        else:
            self.get_logger().info("¡Seguimiento de trayectoria completado! No hay más waypoints.")

# ... (función main no cambia) ...
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()