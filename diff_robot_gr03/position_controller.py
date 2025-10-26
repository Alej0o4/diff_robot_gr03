#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D  # <-- [CAMBIO 1] Importamos Pose2D
from nav_msgs.msg import Odometry
import numpy as np
import math

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

        # --- Parámetros y Ganancias del Controlador ---
        self.declare_parameter("k_x", 0.3)       # Ganancia para el error en X (lambda 1)
        self.declare_parameter("k_y", 0.3)       # Ganancia para el error en Y (lambda 2)
        self.declare_parameter("k_phi", 0.5)     # Ganancia para el error en Phi (lambda 3)
        self.declare_parameter("tol_pos", 0.02)  # Tolerancia de posición (metros)
        self.declare_parameter("tol_angle", 0.02) # Tolerancia de ángulo final (radianes)

        self.k_x = self.get_parameter("k_x").get_parameter_value().double_value
        self.k_y = self.get_parameter("k_y").get_parameter_value().double_value
        self.k_phi = self.get_parameter("k_phi").get_parameter_value().double_value
        self.tol_pos = self.get_parameter("tol_pos").get_parameter_value().double_value
        self.tol_angle = self.get_parameter("tol_angle").get_parameter_value().double_value
        
        # --- Variables de Estado ---
        self.current_pose = {'x': 0.0, 'y': 0.0, 'phi': 0.0}
        self.goal_pose = {'x': 0.0, 'y': 0.0, 'phi': 0.0}
        self.goal_received = False
        self.goal_reached = True

        # --- Publishers y Subscribers ---
        self.pose_subs = self.create_subscription(
            Odometry,
            '/odom_real',
            self.pose_callback,
            10)
        
        self.cmd_pub = self.create_publisher(
            Twist, 
            '/cmd_vel',
            10)

        # --- [CAMBIO 2] ---
        # Nuevo subscriptor para el objetivo
        self.goal_subs = self.create_subscription(
            Pose2D,
            '/goal_pose',
            self.goal_callback,
            10)

        # Timer para el bucle de control
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.control_loop)

        self.get_logger().info("Nodo de control de posición iniciado. Esperando objetivo en el topic '/goal_pose'...")

    # --- [CAMBIO 3] ---
    # Nuevo callback para recibir el objetivo
    def goal_callback(self, msg: Pose2D):
        """Actualiza el objetivo cuando se publica en /goal_pose."""
        self.goal_pose['x'] = msg.x
        self.goal_pose['y'] = msg.y
        self.goal_pose['phi'] = msg.theta # Ángulo final deseado
        
        self.goal_received = True
        self.goal_reached = False
        self.get_logger().info(f"Nuevo objetivo recibido: X={msg.x:.2f}, Y={msg.y:.2f}, Phi={math.degrees(msg.theta):.2f}°")

    def pose_callback(self, msg: Odometry):
        # (Sin cambios)
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_pose['phi'] = euler_from_quaternion(q.x, q.y, q.z, q.w)


    # ============ LÓGICA Movimiento continuo =============

    def control_loop(self):
        cmd_vel_msg = Twist()
        
        if not self.goal_received or self.goal_reached:
            self.cmd_pub.publish(cmd_vel_msg) # Publica 0 si no hay objetivo
            return

        # --- [CAMBIO 4: Lógica de control de 2 etapas] ---
        
        # 1. Variables actuales
        x = self.current_pose['x']
        y = self.current_pose['y']
        phi = self.current_pose['phi']

        # 2. Variables deseadas (objetivo)
        x_d = self.goal_pose['x']
        y_d = self.goal_pose['y']
        phi_final_d = self.goal_pose['phi'] # Ángulo final
        
        # Velocidad deseada (para set-point, es 0)
        x_vel_d = 0.0
        y_vel_d = 0.0
        phi_vel_d = 0.0
        
        # 3. Calcular errores de posición
        e_x = x_d - x
        e_y = y_d - y
        dist_error = math.sqrt(e_x**2 + e_y**2)

        # 4. Decidir etapa (Ir a punto vs. Orientarse)
        if dist_error > self.tol_pos:
            # --- ETAPA 1: Ir al punto (x, y) ---
            
            # Ángulo al objetivo (Line of Sight)
            phi_d_los = math.atan2(e_y, e_x)
            e_phi = math.atan2(math.sin(phi_d_los - phi), math.cos(phi_d_los - phi))

            # Definir la matriz de ganancias (Lambda)
            Lambda = np.diag([self.k_x, self.k_y, self.k_phi])
            
            # Definir vector de error (eta_tilde)
            error_vector = np.array([[e_x], [e_y], [e_phi]])
            
            # Definir velocidad deseada (eta_dot_d)
            eta_dot_d = np.array([[x_vel_d], [y_vel_d], [phi_vel_d]])
            
            # Calcular la pseudoinversa Jacobiana (J_plus de Eq. 21)
            J_plus = np.array([
                [math.cos(phi), math.sin(phi), 0],
                [0, 0, 1]
            ])

            # Calcular u y r (Eq. 21)
            v_control = eta_dot_d + (Lambda @ error_vector)
            xi_cmd = J_plus @ v_control
            
            u = xi_cmd[0, 0]  # Velocidad lineal
            r = xi_cmd[1, 0]  # Velocidad angular
            
            cmd_vel_msg.linear.x = u
            cmd_vel_msg.angular.z = r
            self.get_logger().info(f"[Etapa 1] Dist: {dist_error:.2f} | u: {u:.2f} | r: {r:.2f}")

        else:
            # --- ETAPA 2: Orientarse en el punto ---
            
            # Calcular error de ángulo final
            e_phi_final = math.atan2(math.sin(phi_final_d - phi), math.cos(phi_final_d - phi))
            
            if abs(e_phi_final) > self.tol_angle:
                # Girar en el sitio
                u = 0.0
                r = self.k_phi * e_phi_final # Usamos solo el control P angular
                
                cmd_vel_msg.linear.x = u
                cmd_vel_msg.angular.z = r
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

# ============= LÓGICA Tortuga (COMENTADA) =============

    # def control_loop(self):
    #     cmd_vel_msg = Twist()
        
    #     if not self.goal_received or self.goal_reached:
    #         self.cmd_pub.publish(cmd_vel_msg) # Publica 0 si no hay objetivo
    #         return

    #     # 1. Variables actuales
    #     x = self.current_pose['x']
    #     y = self.current_pose['y']
    #     phi_actual = self.current_pose['phi'] # Renombrado para claridad

    #     # 2. Variables deseadas (objetivo)
    #     x_d = self.goal_pose['x']
    #     y_d = self.goal_pose['y']
    #     phi_final_d = self.goal_pose['phi'] # Ángulo final
        
    #     # 3. Calcular errores de posición
    #     e_x = x_d - x
    #     e_y = y_d - y
    #     dist_error = math.sqrt(e_x**2 + e_y**2)

    #     # --- [CAMBIO A LÓGICA DE TORTUGA "ALIGN-THEN-MOVE"] ---

    #     if dist_error > self.tol_pos:
    #         # --- ETAPA 1: Ir al punto (x, y) ---

    #         # a. Calcular el ángulo hacia el objetivo (Line of Sight)
    #         phi_d_los = math.atan2(e_y, e_x)
            
    #         # b. Calcular el error de ángulo para apuntar al objetivo
    #         e_phi_los = math.atan2(math.sin(phi_d_los - phi_actual), math.cos(phi_d_los - phi_actual))

    #         # c. Decidir si alinear o avanzar
    #         if abs(e_phi_los) > self.tol_angle:
    #             # --- Sub-etapa 1a: ALINEAR ---
    #             # Girar en el sitio para apuntar al objetivo
    #             cmd_vel_msg.linear.x = 0.0
    #             cmd_vel_msg.angular.z = self.k_phi * e_phi_los # k_phi controla la vel. de giro
    #             self.get_logger().info(f"[Etapa 1a] Alineando... Error: {math.degrees(e_phi_los):.2f}°")
            
    #         else:
    #             # --- Sub-etapa 1b: AVANZAR ---
    #             # Ya estamos alineados, avanzar en línea recta
    #             cmd_vel_msg.linear.x = self.k_x * dist_error # k_x controla la vel. de avance
    #             cmd_vel_msg.angular.z = 0.0
    #             self.get_logger().info(f"[Etapa 1b] Avanzando... Dist: {dist_error:.2f} m")

    #     else:
    #         # --- ETAPA 2: Orientarse en el punto ---
    #         # (Esta lógica es idéntica a la que ya teníamos)
            
    #         # Calcular error de ángulo final
    #         e_phi_final = math.atan2(math.sin(phi_final_d - phi_actual), math.cos(phi_final_d - phi_actual))
            
    #         if abs(e_phi_final) > self.tol_angle:
    #             # Girar en el sitio para alcanzar la orientación final
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

    #     # Publicar comandos
    #     self.cmd_pub.publish(cmd_vel_msg)

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