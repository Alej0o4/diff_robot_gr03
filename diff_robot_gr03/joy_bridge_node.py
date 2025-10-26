import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import socket
import json

class JoyBridge(Node):
    def __init__(self):
        super().__init__('joy_bridge_node')
        self.publisher_ = self.create_publisher(Joy, 'joy', 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(('127.0.0.1', 5005))
        self.sock.listen(1)
        self.get_logger().info("Waiting for joystick stream...")
        self.conn, _ = self.sock.accept()
        self.get_logger().info("Joystick stream connected.")
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        try:
            line = self.conn.recv(4096).decode()
            for packet in line.strip().split('\n'):
                if not packet: continue
                data = json.loads(packet)
                msg = Joy()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.axes = data['axes']
                msg.buttons = data['buttons']
                self.publisher_.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Stream error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JoyBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
