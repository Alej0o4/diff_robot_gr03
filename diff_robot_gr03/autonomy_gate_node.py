#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class AutonomyGateNode(Node):
    def __init__(self):
        super().__init__('autonomy_gate_node')

        # --- Parámetros ---
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('cmd_vel_in_topic', '/nav_vel')
        self.declare_parameter('cmd_vel_out_topic', '/nav_vel_gated')
        
        # Botón 'A' en un control de Xbox es el índice 0
        self.declare_parameter('enable_button_idx', 0) 

        # --- Leer Parámetros ---
        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        cmd_in = self.get_parameter('cmd_vel_in_topic').get_parameter_value().string_value
        cmd_out = self.get_parameter('cmd_vel_out_topic').get_parameter_value().string_value
        self.enable_btn = self.get_parameter('enable_button_idx').get_parameter_value().integer_value

        # --- Variables de Estado ---
        self.autonomy_enabled = False
        self.stop_msg = Twist() # Mensaje de 'stop' pre-calculado

        # --- Publishers y Subscribers ---
        self.cmd_out_pub = self.create_publisher(Twist, cmd_out, 10)
        
        self.joy_sub = self.create_subscription(
            Joy, joy_topic, self.joy_callback, 10)
        
        self.cmd_in_sub = self.create_subscription(
            Twist, cmd_in, self.cmd_in_callback, 10)

        self.get_logger().info("Nodo 'AutonomyGate' iniciado.")
        self.get_logger().info(f"==> Botón de activación: índice {self.enable_btn}")
        self.get_logger().info(f"==> Escuchando comandos en: '{cmd_in}'")
        self.get_logger().info(f"==> Publicando comandos 'gated' en: '{cmd_out}'")

    def joy_callback(self, msg: Joy):
        """Actualiza el estado de 'autonomía habilitada' basado en el botón."""
        
        # Lógica de "Hold-to-Run" (Dead Man's Switch)
        # Comprueba si el botón está presionado
        if len(msg.buttons) > self.enable_btn:
            new_state = (msg.buttons[self.enable_btn] == 1)
            
            if new_state != self.autonomy_enabled:
                self.autonomy_enabled = new_state
                if self.autonomy_enabled:
                    self.get_logger().info("AUTONOMÍA HABILITADA (Botón presionado)")
                else:
                    self.get_logger().warn("AUTONOMÍA PAUSADA (Botón suelto)")
        
        # Si la autonomía está deshabilitada, publica 'stop' inmediatamente
        # Esto asegura que el robot se detenga tan pronto como se suelte el botón
        if not self.autonomy_enabled:
             self.cmd_out_pub.publish(self.stop_msg)


    def cmd_in_callback(self, msg: Twist):
        """
        Recibe el comando del 'pure_pursuit' y solo lo re-publica
        si la autonomía está habilitada.
        """
        if self.autonomy_enabled:
            # Botón presionado: dejar pasar el comando
            self.cmd_out_pub.publish(msg)
        else:
            # Botón suelto: publicar 'stop' (aunque joy_callback ya lo hace)
            self.cmd_out_pub.publish(self.stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomyGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()