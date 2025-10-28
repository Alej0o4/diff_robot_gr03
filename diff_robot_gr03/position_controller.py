#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# [CAMBIO 1] Importamos PoseStamped en lugar de Pose2D
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import math

# (Asegúrate de que estas funciones de conversión están en tu nodo)
# (Las copiamos del sensor_bridge_node)
def euler_from_quaternion(x, y, z, w):
    """
    Convierte un cuaternión (w, x, y, z) a ángulos de Euler (roll, pitch, yaw)
    yaw es la rotación alrededor del eje Z.
    """
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z # Devuelve solo yaw

class PositionControllerNode(Node):
    def __init__(self):
        super().__init__("position_controller_node")

        # ... (Tus parámetros y ganancias no cambian) ...
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
        
        # ... (Variables de estado no cambian) ...
        self.current_pose = {'x': 0.0, 'y': 0.0, 'phi': 0.0}
        self.goal_pose = {'x': 0.0, 'y': 0.0, 'phi': 0.0}
        self.goal_received = False
        self.goal_reached = True

        # --- Publishers y Subscribers ---
        self.pose_subs = self.create_subscription(
            Odometry,
            '/odometry/filtered', # Correcto, debe escuchar al EKF
            self.pose_callback,
            10)
        
        self.cmd_pub = self.create_publisher(
            Twist, 
            'nav_vel',
            10)

        # --- [CAMBIO 2] ---
        # El subscriptor ahora espera un tipo PoseStamped
        self.goal_subs = self.create_subscription(
            PoseStamped,  # <-- TIPO ACTUALIZADO
            'goal_pose',
            self.goal_callback,
            10)

        # ... (Timer no cambia) ...
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info("Nodo de control de posición iniciado. Esperando objetivo '/goal_pose' (PoseStamped)...")


    # --- [CAMBIO 3] ---
    # Callback actualizado para "desempaquetar" un PoseStamped
    def goal_callback(self, msg: PoseStamped):
        """Actualiza el objetivo desde un mensaje PoseStamped."""
        
        # Un PoseStamped tiene un 'header' y una 'pose'
        # La 'pose' a su vez tiene 'position' y 'orientation'
        
        self.goal_pose['x'] = msg.pose.position.x
        self.goal_pose['y'] = msg.pose.position.y
        
        # La orientación es un cuaternión, debemos convertirla a Yaw
        q = msg.pose.orientation
        
        # Usamos la función de conversión
        # Ignoramos roll y pitch porque estamos en 2D
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        
        self.goal_pose['phi'] = yaw # Ángulo final deseado
        
        self.goal_received = True
        self.goal_reached = False
        
        # El frame_id nos dice en qué sistema de coordenadas viene el objetivo
        # Debería ser 'odom' (el Fixed Frame de RViz)
        self.get_logger().info(
            f"Nuevo objetivo (frame='{msg.header.frame_id}'): "
            f"X={self.goal_pose['x']:.2f}, Y={self.goal_pose['y']:.2f}, "
            f"Phi={math.degrees(self.goal_pose['phi']):.2f}°"
        )

    def pose_callback(self, msg: Odometry):
        # (Sin cambios)
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # ¡Asegúrate de que la función se llame euler_from_quaternion!
        self.current_pose['phi'] = euler_from_quaternion(q.x, q.y, q.z, q.w)


    def control_loop(self):
        cmd_vel_msg = Twist()
        
        if not self.goal_received or self.goal_reached:
            self.cmd_pub.publish(cmd_vel_msg) # Publica 0 si no hay objetivo
            return

        # 1. Variables actuales
        x_actual = self.current_pose['x']
        y_actual = self.current_pose['y']
        phi_actual = self.current_pose['phi']

        # 2. Variables deseadas (objetivo)
        x_d = self.goal_pose['x']
        y_d = self.goal_pose['y']
        phi_final_d = self.goal_pose['phi'] # Ángulo final
        
        # 3. Calcular errores de posición
        e_x = x_d - x_actual
        e_y = y_d - y_actual
        dist_error = math.sqrt(e_x**2 + e_y**2)

        if dist_error > self.tol_pos:
            # --- ETAPA 1: Ir al punto (x, y) usando Ecuación (21) ---
            
            # Ángulo al objetivo (Line of Sight, phi_d) [cite: 366]
            phi_d_los = math.atan2(e_y, e_x)
            
            # Error de ángulo (phi_d - phi) 
            e_phi_los = math.atan2(math.sin(phi_d_los - phi_actual), 
                                  math.cos(phi_d_los - phi_actual))

            # --- Aplicar Ley de Control (Ecuación 21 expandida) ---
            
            # u(t) = k_x*e_x*cos(phi) + k_y*e_y*sin(phi)
            u_cmd = (self.k_x * e_x * math.cos(phi_actual) + 
                     self.k_y * e_y * math.sin(phi_actual))
            
            # r(t) = k_phi * e_phi
            r_cmd = self.k_phi * e_phi_los
            
            # --- Fin de la Ley de Control ---

            cmd_vel_msg.linear.x = u_cmd
            cmd_vel_msg.angular.z = r_cmd
            
            self.get_logger().info(f"[Eq. 21] Dist: {dist_error:.2f} | u: {u_cmd:.2f} | r: {r_cmd:.2f}")

        else:
            # --- ETAPA 2: Orientarse en el punto ---
            # (Esta lógica se mantiene igual, para corregir el ángulo final)
            
            e_phi_final = math.atan2(math.sin(phi_final_d - phi_actual), 
                                    math.cos(phi_final_d - phi_actual))
            
            if abs(e_phi_final) > self.tol_angle:
                # Girar en el sitio
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = self.k_phi * e_phi_final
                self.get_logger().info(f"[Etapa 2] Orientando... Error: {math.degrees(e_phi_final):.2f}°")
            
            else:
                # --- ¡OBJETIVO ALCANZADO! ---
                self.get_logger().info("¡Objetivo (Posición y Orientación) Alcanzado!")
                self.goal_reached = True
                self.goal_received = False
                cmd_vel_msg.linear.x = 0.0
                cmd_vel_msg.angular.z = 0.0

        # Publicar comandos
        self.cmd_pub.publish(cmd_vel_msg)
        
    # def control_loop(self):
    #     # (Sin cambios)
    #     # La lógica de control ("align-then-move") no necesita cambiar,
    #     # ya que lee desde self.goal_pose, que ya hemos llenado
    #     # correctamente en el goal_callback.
        
    #     cmd_vel_msg = Twist()
        
    #     if not self.goal_received or self.goal_reached:
    #         self.cmd_pub.publish(cmd_vel_msg)
    #         return

    #     x = self.current_pose['x']
    #     y = self.current_pose['y']
    #     phi_actual = self.current_pose['phi']

    #     x_d = self.goal_pose['x']
    #     y_d = self.goal_pose['y']
    #     phi_final_d = self.goal_pose['phi']
        
    #     e_x = x_d - x
    #     e_y = y_d - y
    #     dist_error = math.sqrt(e_x**2 + e_y**2)

    #     if dist_error > self.tol_pos:
    #         # --- ETAPA 1: Ir al punto (x, y) ---
    #         phi_d_los = math.atan2(e_y, e_x)
    #         e_phi_los = math.atan2(math.sin(phi_d_los - phi_actual), math.cos(phi_d_los - phi_actual))

    #         if abs(e_phi_los) > self.tol_angle:
    #             # --- Sub-etapa 1a: ALINEAR ---
    #             cmd_vel_msg.linear.x = 0.0
    #             cmd_vel_msg.angular.z = self.k_phi * e_phi_los
    #             self.get_logger().info(f"[Etapa 1a] Alineando... Error: {math.degrees(e_phi_los):.2f}°")
    #         else:
    #             # --- Sub-etapa 1b: AVANZAR ---
    #             cmd_vel_msg.linear.x = self.k_x * dist_error
    #             cmd_vel_msg.angular.z = 0.0
    #             self.get_logger().info(f"[Etapa 1b] Avanzando... Dist: {dist_error:.2f} m")
    #     else:
    #         # --- ETAPA 2: Orientarse en el punto ---
    #         e_phi_final = math.atan2(math.sin(phi_final_d - phi_actual), math.cos(phi_final_d - phi_actual))
            
    #         if abs(e_phi_final) > self.tol_angle:
    #             cmd_vel_msg.linear.x = 0.0
    #             cmd_vel_msg.angular.z = self.k_phi * e_phi_final
    #             self.get_logger().info(f"[Etapa 2] Orientando... Error: {math.degrees(e_phi_final):.2f}°")
    #         else:
    #             # --- ¡OBJETIVO ALCANZADO! ---
    #             self.get_logger().info("¡Objetivo (Posición y Orientación) Alcanzado!")
    #             self.goal_reached = True
    #             self.goal_received = False
    #             cmd_vel_msg.linear.x = 0.0
    #             cmd_vel_msg.angular.z = 0.0

    #     self.cmd_pub.publish(cmd_vel_msg)

# ... (El resto del archivo, main(), etc., no cambia)
def main(args=None):
    rclpy.init(args=args)
    node = PositionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()