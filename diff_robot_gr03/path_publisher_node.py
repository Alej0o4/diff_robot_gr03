#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import rcl_interfaces.msg

class PathPublisherNode(Node):
    def __init__(self):
        super().__init__("path_publisher_node")

        # --- Parámetros ---
        # Declara parámetros para que sean configurables desde el launch file
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('max_path_size', 200) # Tu "buffer"

        # Lee los parámetros
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.max_size = self.get_parameter('max_path_size').get_parameter_value().integer_value

        # --- Publicador ---
        self.path_pub = self.create_publisher(Path, 'robot_path', 10)
        
        # --- Subscriptor ---
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic, # Escucha el topic que le pasamos por parámetro
            self.odom_callback,
            10)

        # --- Inicializar el mensaje Path ---
        self.path_msg = Path()
        # El Path existe en el frame 'odom', que es nuestro frame fijo
        self.path_msg.header.frame_id = 'odom' 
        
        self.get_logger().info(f"Publicador de Path iniciado. Escuchando Odometría en: '{odom_topic}'")
        self.get_logger().info(f"Tamaño máximo del buffer de path: {self.max_size} poses")

    def odom_callback(self, msg: Odometry):
        # 1. Crear un nuevo mensaje PoseStamped
        # El mensaje Path es una lista de Poses "estampadas" (con tiempo y frame)
        pose_stamped = PoseStamped()
        
        # 2. Copiar la información del mensaje de Odometría
        pose_stamped.header = msg.header  # Copia el stamp y el frame_id ('odom')
        pose_stamped.pose = msg.pose.pose # Copia la pose (posición y orientación)

        # 3. Añadir la nueva pose a la lista
        self.path_msg.poses.append(pose_stamped)

        # 4. Implementar el "buffer"
        # Si la lista es más larga que el tamaño máximo, borra el elemento más antiguo
        if len(self.path_msg.poses) > self.max_size:
            self.path_msg.poses.pop(0) # Elimina la primera pose (la más vieja)

        # 5. Actualizar el timestamp del Path (para que RViz sepa que es nuevo)
        self.path_msg.header.stamp = self.get_clock().now().to_msg()

        # 6. Publicar el mensaje Path completo
        self.path_pub.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()