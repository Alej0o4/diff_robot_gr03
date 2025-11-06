#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np
import math
import csv
import os
from ament_index_python.packages import get_package_share_directory

# --- (Funciones auxiliares euler_from_quaternion y normalize_angle no cambian) ---
def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

class PoseErrorAnalyzerNode(Node):
    def __init__(self):
        super().__init__('pose_error_analyzer_node')

        # --- Parámetros ---
        self.declare_parameter('truth_topic', '/odom_ideal')
        self.declare_parameter('estimate_topic', '/odom_real')
        self.declare_parameter('plan_topic', '/linear_path') # [NUEVO]
        self.declare_parameter('sampling_frequency', 10.0)
        self.declare_parameter('report_frequency', 1.0)
        self.declare_parameter('output_csv_file', 'pose_error_log.csv') 

        truth_topic = self.get_parameter('truth_topic').get_parameter_value().string_value
        estimate_topic = self.get_parameter('estimate_topic').get_parameter_value().string_value
        plan_topic = self.get_parameter('plan_topic').get_parameter_value().string_value # [NUEVO]
        sampling_hz = self.get_parameter('sampling_frequency').get_parameter_value().double_value
        report_hz = self.get_parameter('report_frequency').get_parameter_value().double_value
        csv_basename = self.get_parameter('output_csv_file').get_parameter_value().string_value

        # --- Estado ---
        self.truth_pose = None      # /odom_ideal
        self.estimate_pose = None   # /odom_real
        self.stored_path = None     # /linear_path (lista de poses) [NUEVO]
        
        self.squared_errors_pos = [] # Error de Estimación (Posición)
        self.squared_errors_yaw = [] # Error de Estimación (Yaw)
        self.squared_errors_cte = [] # Error de Control (CTE) [NUEVO]

        # --- (Lógica del archivo CSV no cambia) ---
        package_share_dir = get_package_share_directory('diff_robot_gr03')
        csv_dir_path = os.path.join(package_share_dir, 'csv')
        csv_filename = os.path.join(csv_dir_path, csv_basename)
        try:
            os.makedirs(csv_dir_path, exist_ok=True) 
            self.csv_file = open(csv_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # [MODIFICADO] Nuevo encabezado del CSV
            self.csv_writer.writerow(['timestamp_sec', 'error_est_pos_m', 'error_est_yaw_rad', 'error_cte_m'])
            self.get_logger().info(f"Guardando log de error en: {csv_filename}")
        except IOError as e:
            self.get_logger().error(f"No se pudo abrir el archivo CSV: {e}")
            self.csv_file = None

        # --- Subscribers ---
        self.truth_sub = self.create_subscription(
            Odometry, truth_topic, self.truth_callback, 10)
        
        self.estimate_sub = self.create_subscription(
            Odometry, estimate_topic, self.estimate_callback, 10)

        # [NUEVO] Suscriptor al Path (con QoS Latched)
        latched_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.path_sub = self.create_subscription(
            Path, plan_topic, self.path_callback, latched_qos_profile)

        # --- Timers (no cambian) ---
        self.sampling_timer = self.create_timer(1.0 / sampling_hz, self.sample_error)
        self.report_timer = self.create_timer(1.0 / report_hz, self.report_rmse)

        self.get_logger().info("Nodo Analizador de Error de Pose iniciado.")
        self.get_logger().info(f"==> Verdad (Truth): '{truth_topic}'")
        self.get_logger().info(f"==> Estimación (Estimate): '{estimate_topic}'")
        self.get_logger().info(f"==> Plan: '{plan_topic}'")

    # [NUEVO] Callback para almacenar el path
    def path_callback(self, msg: Path):
        if len(msg.poses) > 1:
            self.stored_path = msg.poses
            self.get_logger().info(f"Path del plan de {len(self.stored_path)} puntos almacenado.")

    def close_csv(self):
        if self.csv_file:
            self.get_logger().info("Cerrando archivo CSV...")
            self.csv_file.close()

    def truth_callback(self, msg: Odometry):
        self.truth_pose = msg.pose.pose

    def estimate_callback(self, msg: Odometry):
        self.estimate_pose = msg.pose.pose

    def sample_error(self):
        """Timer de alta frecuencia para capturar todos los errores."""
        current_time = self.get_clock().now()
        
        # Necesitamos todos los datos para continuar
        if self.truth_pose is None or self.estimate_pose is None or self.stored_path is None:
            return 

        # --- 1. Calcular Error de Estimación (el que ya teníamos) ---
        e_x = self.truth_pose.position.x - self.estimate_pose.position.x
        e_y = self.truth_pose.position.y - self.estimate_pose.position.y
        e_est_pos = math.sqrt(e_x**2 + e_y**2)

        truth_q = self.truth_pose.orientation
        truth_yaw = euler_from_quaternion(truth_q.x, truth_q.y, truth_q.z, truth_q.w)
        est_q = self.estimate_pose.orientation
        est_yaw = euler_from_quaternion(est_q.x, est_q.y, est_q.z, est_q.w)
        e_est_yaw = normalize_angle(truth_yaw - est_yaw)

        # Almacenar para el reporte en vivo
        self.squared_errors_pos.append(e_est_pos**2)
        self.squared_errors_yaw.append(e_est_yaw**2)

        # --- 2. [NUEVO] Calcular Error de Control (Cross-Track Error) ---
        # Comparamos la "Verdad" (/odom_ideal) con el "Plan" (/linear_path)
        robot_x = self.truth_pose.position.x
        robot_y = self.truth_pose.position.y
        
        # Encontrar la distancia más corta al path
        cte_sq = self.find_min_distance_to_path_sq(robot_x, robot_y)
        e_cte = math.sqrt(cte_sq)
        
        # Almacenar para el reporte en vivo
        self.squared_errors_cte.append(cte_sq)

        # --- 3. [MODIFICADO] Escribir la fila de datos en el CSV ---
        if self.csv_file:
            time_sec = current_time.nanoseconds / 1e9
            self.csv_writer.writerow([time_sec, e_est_pos, e_est_yaw, e_cte])

    def find_min_distance_to_path_sq(self, rx, ry):
        """
        Calcula el cuadrado de la distancia perpendicular más corta
        desde el robot (rx, ry) a cualquier segmento en self.stored_path.
        """
        min_dist_sq = float('inf')

        # Iterar sobre cada segmento de línea en el path
        for i in range(len(self.stored_path) - 1):
            p1 = self.stored_path[i].pose.position
            p2 = self.stored_path[i+1].pose.position
            
            # Vector del segmento de línea
            seg_vx = p2.x - p1.x
            seg_vy = p2.y - p1.y
            
            # Vector desde el inicio del segmento al robot
            rob_vx = rx - p1.x
            rob_vy = ry - p1.y
            
            seg_len_sq = seg_vx**2 + seg_vy**2
            
            if seg_len_sq == 0.0:
                # El segmento es un punto (p1 == p2)
                dist_sq = rob_vx**2 + rob_vy**2
            else:
                # Proyectar el robot sobre la línea del segmento
                # t = [ (rx-p1x)(p2x-p1x) + (ry-p1y)(p2y-p1y) ] / |p2-p1|^2
                t = (rob_vx * seg_vx + rob_vy * seg_vy) / seg_len_sq
                
                # 't' es el factor de proyección.
                # Si t < 0, el punto más cercano es p1.
                # Si t > 1, el punto más cercano es p2.
                # Si 0 <= t <= 1, el punto más cercano está en el segmento.
                
                if t < 0.0:
                    dist_sq = (rx - p1.x)**2 + (ry - p1.y)**2 # Distancia a p1
                elif t > 1.0:
                    dist_sq = (rx - p2.x)**2 + (ry - p2.y)**2 # Distancia a p2
                else:
                    # El punto más cercano está en medio del segmento
                    proj_x = p1.x + t * seg_vx
                    proj_y = p1.y + t * seg_vy
                    dist_sq = (rx - proj_x)**2 + (ry - proj_y)**2
            
            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                
        return min_dist_sq

    def report_rmse(self):
        """Timer de baja frecuencia para calcular y mostrar el RMSE acumulado."""
        n_est = len(self.squared_errors_pos)
        n_cte = len(self.squared_errors_cte)

        if n_est == 0 or n_cte == 0:
            self.get_logger().info("Calculando RMSE... (esperando datos de todos los tópicos)")
            return
        
        # --- 1. RMSE de Estimación (el que ya teníamos) ---
        rmse_pos = math.sqrt(sum(self.squared_errors_pos) / n_est)
        rmse_yaw = math.sqrt(sum(self.squared_errors_yaw) / n_est)
        
        # --- 2. [NUEVO] RMSE de Control (CTE) ---
        rmse_cte = math.sqrt(sum(self.squared_errors_cte) / n_cte)

        # --- 3. [MODIFICADO] Reportar ---
        self.get_logger().info(
            f"--- Reporte de Error (N={n_est} muestras) ---\n"
            f"\t[ESTIMACIÓN] RMSE Pos (Real vs Verdad): {rmse_pos * 100.0:.2f} cm\n"
            f"\t[ESTIMACIÓN] RMSE Yaw (Real vs Verdad): {math.degrees(rmse_yaw):.3f} grados\n"
            f"\t[CONTROL]    RMSE CTE (Verdad vs Plan): {rmse_cte * 100.0:.2f} cm"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PoseErrorAnalyzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_csv() 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()