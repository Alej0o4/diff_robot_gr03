#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
import math
import numpy as np
from tf2_ros import TransformBroadcaster

# --- Funciones auxiliares de conversión (las mismas de tu Pure Pursuit) ---
def euler_from_quaternion(x, y, z, w):
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    return math.atan2(t3, t4)

def normalize_angle(angle):
    while angle > math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle

class ComplementaryFilterNode(Node):
    def __init__(self):
        super().__init__('complementary_filter_node')

        # --- Parámetros ---
        self.declare_parameter('odom_topic', '/odom_real')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('output_topic', '/odometry/complementary')
        self.declare_parameter('gain_orientation', 0.05) # 5% de corrección de la IMU
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_link_frame', 'real_base_link')

        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.gain = self.get_parameter('gain_orientation').get_parameter_value().double_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_link_frame = self.get_parameter('base_link_frame').get_parameter_value().string_value

        # --- Estado del Filtro ---
        self.current_pose_x = 0.0
        self.current_pose_y = 0.0
        self.current_pose_yaw = 0.0
        self.last_imu_yaw = None
        self.last_odom_time = None

        # --- Publishers y Subscribers ---
        self.odom_pub = self.create_publisher(Odometry, output_topic, 10)
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_callback, 10)
        
        # El callback de Odom es el bucle principal del filtro
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        
        # --- Publicador de TF ---
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"Filtro Complementario iniciado. Ganancia={self.gain*100}%")
        self.get_logger().info(f"==> Escuchando Odom: '{odom_topic}'")
        self.get_logger().info(f"==> Escuchando IMU: '{imu_topic}'")
        self.get_logger().info(f"==> Publicando Pose en: '{output_topic}'")

    def imu_callback(self, msg: Imu):
        # Solo almacena la orientación (Yaw) absoluta de la IMU
        q = msg.orientation
        self.last_imu_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def odom_callback(self, msg: Odometry):
        # 1. Verificar que tenemos datos de la IMU
        if self.last_imu_yaw is None:
            self.get_logger().warn("Esperando datos de la IMU...", throttle_duration_sec=2.0)
            return

        current_time = self.get_clock().now()
        
        # 2. Calcular dt (delta de tiempo)
        if self.last_odom_time is None:
            self.last_odom_time = current_time
            return # Saltar la primera iteración
            
        dt = (current_time - self.last_odom_time).nanoseconds * 1e-9
        self.last_odom_time = current_time

        # 3. Obtener velocidades de la Odometría
        vx = msg.twist.twist.linear.x
        vyaw = msg.twist.twist.angular.z # Velocidad angular de encoders

        # --- 4. PREDICT STEP (Dead-Reckoning) ---
        # Estimar el nuevo Yaw usando la velocidad angular de los encoders
        predicted_yaw = self.current_pose_yaw + vyaw * dt
        
        # Estimar la nueva posición usando la velocidad lineal y el Yaw predicho
        self.current_pose_x += vx * math.cos(predicted_yaw) * dt
        self.current_pose_y += vx * math.sin(predicted_yaw) * dt

        # --- 5. CORRECT STEP (Filtro Complementario) ---
        # Corregir la deriva de Yaw usando la IMU
        
        # Calcular el error entre la predicción y la "verdad" de la IMU
        yaw_error = normalize_angle(self.last_imu_yaw - predicted_yaw)
        
        # Aplicar la corrección (aquí está la magia)
        # Tomamos nuestra predicción y le sumamos un pequeño % del error
        self.current_pose_yaw = normalize_angle(predicted_yaw + self.gain * yaw_error)

        # --- 6. PUBLICAR RESULTADOS ---
        
        # 6a. Publicar el mensaje Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_link_frame
        
        odom_msg.pose.pose.position.x = self.current_pose_x
        odom_msg.pose.pose.position.y = self.current_pose_y
        
        # Convertir Yaw (Euler) a Cuaternión
        q = euler_to_quaternion(self.current_pose_yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        # También copiamos las velocidades (no las filtramos, solo las pasamos)
        odom_msg.twist.twist = msg.twist.twist
        
        self.odom_pub.publish(odom_msg)

        # 6b. Publicar la Transformación TF (¡Esto arregla RViz!)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_link_frame
        
        t.transform.translation.x = self.current_pose_x
        t.transform.translation.y = self.current_pose_y
        t.transform.translation.z = 0.0 # Asumimos 2D
        
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

# Función auxiliar para convertir Euler a Cuaternión
def euler_to_quaternion(yaw, pitch=0.0, roll=0.0):
    cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy; x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy; z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)

def main(args=None):
    rclpy.init(args=args)
    node = ComplementaryFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()