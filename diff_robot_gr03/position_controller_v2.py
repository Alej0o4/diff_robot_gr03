#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry  # <-- [CAMBIO 1] Importamos Odometry
import numpy as np
import math

def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

class PositionControllerNodeV2(Node):
    def __init__(self):
        super().__init__("position_controller_node_v2")

        # ... (Parámetros y ganancias no cambian) ...
        self.declare_parameter("k_x", 0.3)
        self.declare_parameter("k_y", 0.3)
        self.declare_parameter("k_phi", 0.5)
        self.declare_parameter("tol_pos", 0.02)
        self.declare_parameter("tol_angle", 0.02)
        
        self.k_x = self.get_parameter("k_x").get_parameter_value().double_value
        self.k_y = self.get_parameter("k_y").get_parameter_value().double_value
        self.k_phi = self.get_parameter("k_phi").get_parameter_value().double_value
        self.tol_pos = self.get_parameter("tol_pos").get_parameter_value().double_value
        self.tol_angle = self.get_parameter("tol_angle").get_parameter_value().double_value
        
        # --- [CAMBIO 2] ---
        # Almacenamos por separado la pose deseada y la velocidad deseada
        self.current_pose = {'x': 0.0, 'y': 0.0, 'phi': 0.0}
        self.goal_pose = {'x': 0.0, 'y': 0.0, 'phi': 0.0}
        self.goal_vel = {'x_dot': 0.0, 'y_dot': 0.0, 'phi_dot': 0.0} # <-- NUEVO
        
        self.goal_received = False
        self.goal_reached = True

        # --- Publishers y Subscribers ---
        self.pose_subs = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.pose_callback,
            10)
        
        self.cmd_pub = self.create_publisher(
            Twist, 
            'nav_vel',
            10)

        # Subscriptor para "2D Goal Pose" de RViz (Set-Point)
        self.goal_subs = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_pose_callback, # <-- Renombrado
            10)

        # --- [CAMBIO 3] ---
        # NUEVO subscriptor para seguimiento de trayectoria
        self.traj_subs = self.create_subscription(
            Odometry, # <-- Usamos Odometry (Pose + Twist)
            'trajectory_point',
            self.trajectory_callback,
            10)

        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info("Controlador genérico iniciado. Escuchando en '/goal_pose' y '/trajectory_point'...")

    def goal_pose_callback(self, msg: PoseStamped):
        """Callback para objetivos Set-Point (de RViz). Velocidad deseada es 0."""
        
        self.goal_pose['x'] = msg.pose.position.x
        self.goal_pose['y'] = msg.pose.position.y
        
        q = msg.pose.orientation
        self.goal_pose['phi'] = euler_from_quaternion(q.x, q.y, q.z, q.w)
        
        # --- Importante: Definimos las velocidades deseadas como 0 ---
        self.goal_vel['x_dot'] = 0.0
        self.goal_vel['y_dot'] = 0.0
        self.goal_vel['phi_dot'] = 0.0
        
        self.goal_received = True
        self.goal_reached = False
        self.get_logger().info(
            f"Nuevo objetivo SET-POINT (frame='{msg.header.frame_id}'): "
            f"X={self.goal_pose['x']:.2f}, Y={self.goal_pose['y']:.2f}, "
            f"Phi={math.degrees(self.goal_pose['phi']):.2f}°"
        )

    # --- [CAMBIO 4] ---
    # NUEVO Callback para seguimiento de trayectoria
    def trajectory_callback(self, msg: Odometry):
        """Callback para objetivos de trayectoria. Incluye velocidad deseada."""
        
        # Extraer la Pose deseada
        self.goal_pose['x'] = msg.pose.pose.position.x
        self.goal_pose['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.goal_pose['phi'] = euler_from_quaternion(q.x, q.y, q.z, q.w)
        
        # Extraer la Velocidad deseada (Feedforward)
        # Asumimos que la velocidad viene en el marco 'odom'
        self.goal_vel['x_dot'] = msg.twist.twist.linear.x
        self.goal_vel['y_dot'] = msg.twist.twist.linear.y
        self.goal_vel['phi_dot'] = msg.twist.twist.angular.z
        
        self.goal_received = True
        self.goal_reached = False
        
        # Este log puede ser muy ruidoso, puedes comentarlo
        # self.get_logger().info(
        #     f"Nuevo punto de TRAYECTORIA: Xd={self.goal_pose['x']:.2f}, Vxd={self.goal_vel['x_dot']:.2f}"
        # )

    def pose_callback(self, msg: Odometry):
        # (Sin cambios)
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_pose['phi'] = euler_from_quaternion(q.x, q.y, q.z, q.w)
        
    def control_loop(self):
        cmd_vel_msg = Twist()
        
        if not self.goal_received or self.goal_reached:
            self.cmd_pub.publish(cmd_vel_msg)
            return

        # 1. Variables actuales
        x_actual = self.current_pose['x']
        y_actual = self.current_pose['y']
        phi_actual = self.current_pose['phi']

        # 2. Variables de pose deseada
        x_d = self.goal_pose['x']
        y_d = self.goal_pose['y']
        phi_final_d = self.goal_pose['phi']
        
        # --- [CAMBIO 5] ---
        # 2b. Variables de velocidad deseada (Feedforward)
        x_dot_d = self.goal_vel['x_dot']
        y_dot_d = self.goal_vel['y_dot']
        phi_dot_d = self.goal_vel['phi_dot']
        
        # 3. Errores de posición
        e_x = x_d - x_actual
        e_y = y_d - y_actual
        dist_error = math.sqrt(e_x**2 + e_y**2)

        if dist_error > self.tol_pos:
            # --- ETAPA 1: Ir al punto (x, y) usando Ecuación (21) COMPLETA ---
            
            # Ángulo al objetivo (Line of Sight, phi_d)
            phi_d_los = math.atan2(e_y, e_x)
            
            # Error de ángulo (phi_d - phi)
            e_phi_los = math.atan2(math.sin(phi_d_los - phi_actual), 
                                  math.cos(phi_d_los - phi_actual))

            # --- Aplicar Ley de Control COMPLETA (Feedforward + Feedback) ---
            
            # u(t) = cos(phi) * (x_dot_d + k_x*e_x) + sin(phi) * (y_dot_d + k_y*e_y)
            term_x = x_dot_d + self.k_x * e_x
            term_y = y_dot_d + self.k_y * e_y
            u_cmd = (term_x * math.cos(phi_actual) + 
                     term_y * math.sin(phi_actual))
            
            # r(t) = phi_dot_d + k_phi * e_phi
            # Usamos e_phi_los, ya que queremos que el robot apunte al 'line of sight'
            # mientras sigue la trayectoria.
            r_cmd = phi_dot_d + self.k_phi * e_phi_los
            
            # --- Fin de la Ley de Control ---

            cmd_vel_msg.linear.x = u_cmd
            cmd_vel_msg.angular.z = r_cmd
            
            self.get_logger().info(f"[Eq. 21] Dist: {dist_error:.2f} | u: {u_cmd:.2f} | r: {r_cmd:.2f}")

        else:
            # --- ETAPA 2: Orientarse en el punto final ---
            
            e_phi_final = math.atan2(math.sin(phi_final_d - phi_actual), 
                                    math.cos(phi_final_d - phi_actual))
            
            if abs(e_phi_final) > self.tol_angle:
                cmd_vel_msg.linear.x = 0.0 # Detenido
                
                # Aún aplicamos el feedforward de velocidad angular
                r_cmd = phi_dot_d + self.k_phi * e_phi_final
                cmd_vel_msg.angular.z = r_cmd
                
                self.get_logger().info(f"[Etapa 2] Orientando... Error: {math.degrees(e_phi_final):.2f}°")
            
            else:
                # --- ¡OBJETIVO ALCANZADO! ---
                self.get_logger().info("¡Objetivo (Posición y Orientación) Alcanzado!")
                self.goal_reached = True
                self.goal_received = False
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0 # Parada total

        # Publicar comandos
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