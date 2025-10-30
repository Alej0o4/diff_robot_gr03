#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry

class MrImuOrTF(Node): # <--- CHANGE ME
    def __init__(self):
        super().__init__("mr_imu_or_tf") # <--- CHANGE ME

                # publishers and subscribers
        qos = QoSProfile(depth=10)
        
        self.ultrasonic_sub = self.create_subscription(
            msg_type=Range,
            topic="ultrasonic",
            callback=self.ultrasonic_callback,
            qos_profile=qos
        )
        
        self.range_pub = self.create_publisher(
            msg_type=Range,
            topic="range",
            qos_profile=qos
        )
        self.ultrasonic = Range()
        self.range = Range()
        
        # tf broadcaster
        # Create a TF broadcaster to publish TransformStamped messages onto /tf
        # Used to maintain coordinate frame relationships in ROS 2
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.t = TransformStamped()
        self.t.header.frame_id = 'real_base_link'
        self.t.child_frame_id = 'real_ultrasonic'

        # time variables for integration
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)
            
    def ultrasonic_callback(self, msg):
        self.get_logger().info(f"Ultrasonic Range: {msg.range} m")
        self.ultrasonic = msg

    def timer_callback(self):
                
        # publish tf message
        self.or_tf_broadcaster()
        
    def or_tf_broadcaster(self):
        
        # header
        now = self.get_clock().now()
        self.t.header.stamp = now.to_msg()
        
        # Position the ultrasonic sensor at the front of the robot
        # Based on your URDF: lx/2+(lx/2 - wheel_position) ≈ 0.125 meters forward
        self.t.transform.translation.x = 0.125  # Front of robot
        self.t.transform.translation.y = 0.0    # Center
        self.t.transform.translation.z = 0.057  # Height (lz)
        
        # No rotation needed
        self.t.transform.rotation.x = 0.0
        self.t.transform.rotation.y = 0.0
        self.t.transform.rotation.z = 0.0
        self.t.transform.rotation.w = 1.0
               
        # send the transformation
        self.tf_broadcaster.sendTransform(self.t)
        
        self.range.header.stamp = now.to_msg()
        self.range.header.frame_id = "real_ultrasonic"
        self.range.radiation_type = self.ultrasonic.radiation_type
        self.range.field_of_view = self.ultrasonic.field_of_view
        self.range.min_range = self.ultrasonic.min_range
        self.range.max_range = self.ultrasonic.max_range
        self.range.range = self.ultrasonic.range

        self.range_pub.publish(self.range)

def main():
    rclpy.init()
    node = MrImuOrTF() # <--- CHANGE ME
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()