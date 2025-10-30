#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import math

# ... (función euler_from_quaternion no cambia) ...
def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

class PositionControllerNodeV2(Node):
    def __init__(self):
        super().__init__("position_controller_node_v2")

        # --- [CAMBIO 1] Declarar TODOS los parámetros ---
        # Le decimos al nodo qué parámetros puede recibir.
        self.declare_parameter("k_x", 0.3)
        self.declare_parameter("k_y", 0.3)
        self.declare_parameter("k_phi", 0.5)
        self.declare_parameter("tol_pos", 0.02)
        self.declare_parameter("tol_angle", 0.02)
        
        # Este es el nuevo parámetro clave para cambiar entre simulación y realidad
        # Por defecto, usará el robot real (/odometry/filtered)
        self.declare_parameter("odom_topic_source", "/odometry/filtered")
        
        # --- [CAMBIO 2] Leer los parámetros ---
        self.k_x = self.get_parameter("k_x").get_parameter_value().double_value
        self.k_y = self.get_parameter("k_y").get_parameter_value().double_value
        self.k_phi = self.get_parameter("k_phi").get_parameter_value().double_value
        self.tol_pos = self.get_parameter("tol_pos").get_parameter_value().double_value
        self.tol_angle = self.get_parameter("tol_angle").get_parameter_value().double_value
        
        # Leer el nuevo parámetro
        odom_topic = self.get_parameter("odom_topic_source").get_parameter_value().string_value
        
        self.get_logger().info(f"Controlador de Waypoints iniciado.")
        self.get_logger().info(f"==> Escuchando pose del robot en: '{odom_topic}'")
        
        # --- (Resto de variables de estado no cambian) ---
        self.current_pose = {'x': 0.0, 'y': 0.0, 'phi': 0.0}
        self.goal_pose = {'x': 0.0, 'y': 0.0, 'phi': 0.0}
        self.goal_received = False
        self.goal_reached = True

        # --- Publishers y Subscribers ---
        self.pose_subs = self.create_subscription(
            Odometry,
            odom_topic, 
            self.pose_callback,
            10)
        
        self.cmd_pub = self.create_publisher(
            Twist, 
            'nav_vel',
            10)

        # Subscriptor para "2D Goal Pose" de RViz
        self.goal_subs = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_pose_callback, 
            10)

        # --- [CAMBIO 2] Definir QoS para el publicador de waypoints ---
        # (Debe coincidir con el QoS del subscriptor en el Generador)
        state_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        self.traj_subs = self.create_subscription(
            PoseStamped,
            'goal_waypoint',
            self.goal_waypoint_callback,
            state_qos_profile # <-- Usar QoS de Estado
        )

        # --- [CAMBIO 3] Definir QoS para el publicador de "ready" ---
        # (Debe coincidir con el QoS del subscriptor en el Generador)
        event_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE, # <-- ¡LA CLAVE!
            depth=10
        )
        
        self.ready_pub = self.create_publisher(
            Bool,
            'controller_ready',
            event_qos_profile # <-- Usar QoS de Evento
        )

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.control_loop)
        
        self.request_next_waypoint()

    # --- (El resto del archivo .py no necesita cambios) ---
    # (request_next_waypoint, goal_pose_callback, goal_waypoint_callback, 
    #  pose_callback, control_loop, y main son idénticos a la versión anterior)

    def request_next_waypoint(self):
        self.get_logger().info("Solicitando siguiente waypoint...")
        msg = Bool()
        msg.data = True
        self.ready_pub.publish(msg)

    def goal_pose_callback(self, msg: PoseStamped):
        self.goal_pose['x'] = msg.pose.position.x
        self.goal_pose['y'] = msg.pose.position.y
        q = msg.pose.orientation
        self.goal_pose['phi'] = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.goal_received = True
        self.goal_reached = False
        self.get_logger().info(f"Nuevo objetivo SET-POINT (RViz): X={self.goal_pose['x']:.2f}, Y={self.goal_pose['y']:.2f}")

    def goal_waypoint_callback(self, msg: PoseStamped):
        self.goal_pose['x'] = msg.pose.position.x
        self.goal_pose['y'] = msg.pose.position.y
        q = msg.pose.orientation
        self.goal_pose['phi'] = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.goal_received = True
        self.goal_reached = False
        self.get_logger().info(f"Nuevo WAYPOINT recibido: X={self.goal_pose['x']:.2f}, Y={self.goal_pose['y']:.2f}")

    def pose_callback(self, msg: Odometry):
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_pose['phi'] = euler_from_quaternion(q.x, q.y, q.z, q.w)
        
    def control_loop(self):
        cmd_vel_msg = Twist()
        if not self.goal_received or self.goal_reached:
            self.cmd_pub.publish(cmd_vel_msg)
            return

        x_actual = self.current_pose['x']
        y_actual = self.current_pose['y']
        phi_actual = self.current_pose['phi']
        x_d = self.goal_pose['x']
        y_d = self.goal_pose['y']
        phi_final_d = self.goal_pose['phi']
        
        e_x = x_d - x_actual
        e_y = y_d - y_actual
        dist_error = math.sqrt(e_x**2 + e_y**2)

        if dist_error > self.tol_pos:
            phi_d_los = math.atan2(e_y, e_x)
            e_phi_los = math.atan2(math.sin(phi_d_los - phi_actual), 
                                   math.cos(phi_d_los - phi_actual))
            u_cmd = (self.k_x * e_x * math.cos(phi_actual) + 
                     self.k_y * e_y * math.sin(phi_actual))
            r_cmd = self.k_phi * e_phi_los
            cmd_vel_msg.linear.x = u_cmd
            cmd_vel_msg.angular.z = r_cmd
            self.get_logger().debug(f"[Etapa 1] Dist: {dist_error:.2f} | u: {u_cmd:.2f} | r: {r_cmd:.2f}")
        else:
            e_phi_final = math.atan2(math.sin(phi_final_d - phi_actual), 
                                     math.cos(phi_final_d - phi_actual))
            if abs(e_phi_final) > self.tol_angle:
                cmd_vel_msg.linear.x = 0.0
                r_cmd = self.k_phi * e_phi_final
                cmd_vel_msg.angular.z = r_cmd
                self.get_logger().debug(f"[Etapa 2] Orientando... Error: {math.degrees(e_phi_final):.2f}°")
            else:
                self.get_logger().info(f"¡Waypoint ({x_d:.2f}, {y_d:.2f}) Alcanzado!")
                self.goal_reached = True
                self.goal_received = False
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0
                self.request_next_waypoint()

        self.cmd_pub.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PositionControllerNodeV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()