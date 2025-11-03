#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# ... (funciones euler_from_quaternion y normalize_angle no cambian) ...
def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

class PurePursuitControllerNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # ... (lectura de parámetros no cambia) ...
        self.declare_parameter('lookahead_distance', 0.3)
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('controller_frequency', 20.0)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('path_topic', '/smooth_path')
        self.declare_parameter('max_linear_velocity', 0.15) 
        self.declare_parameter('max_angular_velocity', 1.0)



        self.ld = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.frequency = self.get_parameter('controller_frequency').get_parameter_value().double_value
        self.goal_tol = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        self.path_topic_name = self.get_parameter('path_topic').get_parameter_value().string_value
        self.v_max = self.get_parameter('max_linear_velocity').get_parameter_value().double_value
        self.w_max = self.get_parameter('max_angular_velocity').get_parameter_value().double_value

        # --- Variables de Estado ---
        self.path = None
        self.current_pose = None
        self.path_received = False
        self.pose_received = False
        
        # --- [CAMBIO 1] Añadir la bandera de estado ---
        self.trajectory_started = False
        # --- [FIN CAMBIO 1] ---
        self.last_closest_idx = 0

        # --- Publishers y Subscribers (con QoS) ---
        latched_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/nav_vel', 10)
        self.path_sub = self.create_subscription(
            Path, self.path_topic_name, self.path_callback, latched_qos_profile)
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10)
        self.lookahead_pub = self.create_publisher(PoseStamped, '/lookahead_point', 10)
        
        self.timer = self.create_timer(1.0 / self.frequency, self.control_loop)
        
        self.get_logger().info(f"==> Ld={self.ld}m, V_max={self.v_max}m/s, W_max={self.w_max}rad/s")
        self.get_logger().info(f"==> Escuchando pose en: '{self.odom_topic}'")
        self.get_logger().info(f"==> Escuchando path en: '{self.path_topic_name}'")

    def path_callback(self, msg: Path):
        """
        Almacena el path suave.
        Se protege contra "spam" de paths idénticos.
        """
        # --- [FIX ROBUSTO CONTRA SPAM] ---
        # Si ya estamos siguiendo un path (timer está corriendo), 
        # ignora cualquier path nuevo.
        if self.path_received and not self.timer.is_canceled():
            self.get_logger().warn("Recibido un path nuevo mientras se seguía uno. Ignorando 'spam'.")
            return
        # --- [FIN FIX] ---

        # Si llegamos aquí, es un path Genuinamente Nuevo (o el primero)
        if len(msg.poses) > 0:
            self.path = msg.poses
            self.path_received = True
            self.trajectory_started = False 
            
            # --- [CAMBIO 3] Resetear la memoria del índice ---
            self.last_closest_idx = 0 
            # --- [FIN CAMBIO 3] ---
            
            self.get_logger().info(f"Path de {len(self.path)} puntos (re)cargado. Listo para iniciar.")
            
            if self.timer.is_canceled():
                self.timer.reset()
                self.get_logger().info("Timer del controlador reactivado.")

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        self.pose_received = True

    # --- [CAMBIO 3] Lógica del bucle de control modificada ---
    def control_loop(self):
        """Bucle principal del controlador, con lógica de "inicio"."""
        
        if not self.path_received or not self.pose_received:
            self.get_logger().debug("Esperando path o pose...")
            return

        # 1. Encontrar el punto "zanahoria"
        lookahead_point, is_last_point = self.find_lookahead_point()

        if lookahead_point is None:
            self.get_logger().warn("No se encontró un lookahead point. Deteniendo.")
            self.stop_robot()
            return
            
        # 2. Revisar condición de PARADA FINAL
        if is_last_point:
            dist_to_goal = math.dist(
                (self.current_pose.position.x, self.current_pose.position.y),
                (lookahead_point.position.x, lookahead_point.position.y)
            )
            
            if dist_to_goal < self.goal_tol and self.trajectory_started:
                self.get_logger().info("¡Objetivo final alcanzado!")
                self.stop_robot()
                self.timer.cancel() # Detener el bucle de control
                return # Salir del bucle

        
        # --- [LÓGICA DE CONTROL RE-ORDENADA Y ADAPTATIVA] ---

        # 3. Comprobar primero la condición de "desatasco"
        # (Esto solo se activa al inicio, si estamos en el punto (0,0) del path)
        if is_last_point and not self.trajectory_started:
            self.get_logger().info("Iniciando trayectoria desde (0,0). 'Desatascando'...")
            self.trajectory_started = True
            v_final = self.v_max # Arrancar recto a máxima velocidad
            w_final = 0.0
        
        else:
            # --- LÓGICA NORMAL DE PURE PURSUIT ADAPTATIVO ---
            
            # 4. Calcular el ángulo de dirección (alpha)
            alpha = self.calculate_steering_angle(lookahead_point)
            
            # 5. Calcular velocidad angular 'raw' (usando v_max)
            w_raw = (2.0 * self.v_max * math.sin(alpha)) / self.ld
            
            # 6. Limitar (clampear) la velocidad angular por seguridad
            w_final = np.clip(w_raw, -self.w_max, self.w_max)
            
            # 7. Calcular la velocidad lineal dinámica (del nodo del profesor)
            v_final = self.v_max / (abs(w_final) + 1.0)
            
            # (Opcional: log si se clampea, con throttle para no hacer spam)
            if w_raw != w_final:
                 self.get_logger().warn(f"Velocidad angular limitada a {w_final:.2f} rad/s", throttle_duration_sec=1.0)
            
            # Marcar que hemos arrancado (si no lo hizo la lógica de desatasco)
            if not self.trajectory_started:
                self.trajectory_started = True

        # --- [FIN LÓGICA DE CONTROL] ---

        # 8. Publicar Comandos
        cmd_msg = Twist()
        cmd_msg.linear.x = v_final
        cmd_msg.angular.z = w_final
        self.cmd_vel_pub.publish(cmd_msg)

    def find_lookahead_point(self):
        """
        Encuentra el punto "zanahoria" en la trayectoria.
        [MODIFICADO] Busca el punto más cercano SOLO hacia adelante 
        del último índice conocido.
        """
        if not self.path: return None, False
        
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        
        # --- [CAMBIO 4] Lógica de búsqueda "Stateful" ---
        
        closest_dist = float('inf')
        closest_idx = self.last_closest_idx # Empezar desde la memoria

        # Buscar el nuevo punto más cercano, pero SOLO hacia adelante.
        # Esto evita "saltar" hacia atrás al inicio del path.
        for i in range(self.last_closest_idx, len(self.path)):
            point_x = self.path[i].pose.position.x
            point_y = self.path[i].pose.position.y
            dist = math.dist((robot_x, robot_y), (point_x, point_y))
            
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i
            
            # OPTIMIZACIÓN: Si la distancia empieza a crecer de nuevo,
            # es porque ya pasamos el punto más cercano.
            # (Ajusta '0.2' según sea necesario, es un "slop")
            if dist > closest_dist + 0.2:
                break 

        # Actualizar la memoria para la próxima iteración del control_loop
        self.last_closest_idx = closest_idx
        
        # --- [FIN CAMBIO 4] ---
        
        # (Tu "FIX SUTIL" y la búsqueda global se eliminan, ya no son necesarios)

        # Buscar hacia adelante desde el punto más cercano (esta parte es igual)
        for i in range(closest_idx, len(self.path)):
            point_x = self.path[i].pose.position.x
            point_y = self.path[i].pose.position.y
            dist_to_point = math.dist((robot_x, robot_y), (point_x, point_y))
            
            if dist_to_point >= self.ld:
                self.publish_lookahead_marker(self.path[i].pose)
                return self.path[i].pose, False

        # Si no encontramos ninguno (ej. estamos al final), la zanahoria es el último punto
        self.publish_lookahead_marker(self.path[-1].pose)
        return self.path[-1].pose, True

    def calculate_steering_angle(self, point):
        # ... (código no cambia) ...
        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y
        robot_yaw = euler_from_quaternion(
            self.current_pose.orientation.x, self.current_pose.orientation.y,
            self.current_pose.orientation.z, self.current_pose.orientation.w
        )
        angle_to_point = math.atan2(
            point.position.y - robot_y, point.position.x - robot_x
        )
        alpha = normalize_angle(angle_to_point - robot_yaw)
        return alpha

    def publish_lookahead_marker(self, pose: PoseStamped):
        # ... (código no cambia) ...
        marker_pose = PoseStamped()
        marker_pose.header.frame_id = "odom"
        marker_pose.header.stamp = self.get_clock().now().to_msg()
        marker_pose.pose = pose
        self.lookahead_pub.publish(marker_pose)

    def stop_robot(self):
        # ... (código no cambia) ...
        cmd_msg = Twist()
        cmd_msg.linear.x = 0.0
        cmd_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()