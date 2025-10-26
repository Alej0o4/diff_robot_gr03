#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import numpy as np
from sensor_msgs.msg import Imu, Range
import math  # <-- [NUEVO] Necesario para las conversiones

# --- [NUEVO] ---
# Funciones copiadas de tu nodo de prueba MrImuOrTFRozo
def quaternion_to_euler(x, y, z, w):
    """
    Convierte un cuaternión (x, y, z, w) a ángulos de Euler (roll, pitch, yaw).
    Retorna los ángulos en radianes.
    """
    # Roll (rotación sobre X)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (rotación sobre Y)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # saturación en ±90°
    else:
        pitch = math.asin(sinp)

    # Yaw (rotación sobre Z)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convierte ángulos de Euler (roll, pitch, yaw) a cuaternión (x, y, z, w).
    Los ángulos deben estar en radianes.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w
# --- [FIN DE LO NUEVO] ---


class SensorBridgeNode(Node):
    def __init__(self):
        super().__init__("sensor_bridge_node")

        qos = QoSProfile(depth=10)
        
        # --- Publishers (Topics "Limpios" para EKF) ---
        self.imu_pub = self.create_publisher(
            msg_type=Imu,
            topic="imu/data",
            qos_profile=qos
        )
        self.range_pub = self.create_publisher(
            msg_type=Range,
            topic="sensor/range",
            qos_profile=qos
        )

        # --- Subscribers (Topics "Crudos" desde la Pico) ---
        self.imu_sub = self.create_subscription(
            msg_type=Imu,
            topic="imu",
            callback=self.imu_callback,
            qos_profile=qos
        )
        self.ultrasonic_sub = self.create_subscription(
            msg_type=Range,
            topic="ultrasonic",
            callback=self.ultrasonic_callback,
            qos_profile=qos
        )
        
        self.get_logger().info("Sensor Bridge iniciado. Preparando datos para EKF...")

        # --- Matrices de Covarianza (Sin cambios) ---
        self.imu_orientation_cov = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.01
        ]
        self.imu_angular_vel_cov = [
            0.02, 0.0,  0.0,
            0.0,  0.02, 0.0,
            0.0,  0.0,  0.02
        ]
        self.imu_linear_accel_cov = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1
        ]

    def imu_callback(self, msg: Imu):
        
        # --- [NUEVO] LÓGICA DE INVERSIÓN DEL EJE Z ---
        
        # 1. Invertir la velocidad angular en Z
        msg.angular_velocity.z = -msg.angular_velocity.z
        
        # 2. Invertir la orientación (Yaw) del cuaternión
        q = msg.orientation
        roll, pitch, yaw = quaternion_to_euler(q.x, q.y, q.z, q.w)
        
        # Negamos solo el Yaw
        new_yaw = -yaw
        
        # Convertimos de vuelta a cuaternión
        new_x, new_y, new_z, new_w = euler_to_quaternion(roll, pitch, new_yaw)
        
        msg.orientation.x = new_x
        msg.orientation.y = new_y
        msg.orientation.z = new_z
        msg.orientation.w = new_w
        
        # --- [FIN DE LÓGICA DE INVERSIÓN] ---
        
        # 3. Asignar el frame_id correcto (con prefijo)
        msg.header.frame_id = "real_imu_link" 
        
        # 4. Asignar las covarianzas
        msg.orientation_covariance = self.imu_orientation_cov
        msg.angular_velocity_covariance = self.imu_angular_vel_cov
        msg.linear_acceleration_covariance = self.imu_linear_accel_cov
        
        # 5. Re-publicar el mensaje "limpio" y "corregido"
        self.imu_pub.publish(msg)

    def ultrasonic_callback(self, msg: Range):
        # (Sin cambios)
        msg.header.frame_id = "real_ultrasonic" 
        self.range_pub.publish(msg)

def main():
    rclpy.init()
    node = SensorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()