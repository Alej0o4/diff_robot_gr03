#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile,ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool
import ezdxf, pathlib, math, csv 
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

class DXFParserNode(Node):
    def __init__(self):
        super().__init__('dxf_exporter_node')

        self.ENTITY_TYPE_MAP = {
            "LINE": 1, "LWPOLYLINE": 2, "POLYLINE": 3,
            "ARC": 4, "CIRCLE": 5, "SPLINE": 6,
            "RECTANGLE": 7 
        }
        
        self.waypoints = []

        # --- Parámetros ---
        self.declare_parameter('trajectory_mode', 'dxf')
        self.declare_parameter('dxf_file','/home/alejo/diff_robot/src/diff_robot_gr03/dxf/30X30.dxf')
        self.declare_parameter('rectangle.width', 0.5)
        self.declare_parameter('rectangle.height', 0.3)
        self.declare_parameter('num_laps', 1)
        self.declare_parameter('rectangle.start_x', 0.0)
        self.declare_parameter('rectangle.start_y', 0.0)

        self.num_laps = self.get_parameter('num_laps').get_parameter_value().integer_value


        # --- Publishers ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.path_pub = self.create_publisher(Path, 'dxf_path_pub', qos) 
        self.pc_pub = self.create_publisher(PointCloud, 'dxf_pointcloud', qos)
        
        # --- Proceso ---
        self.waypoints = self.load_trajectory() 
        
        if self.waypoints:
            self.export_to_csv()
            self.publish_pointcloud() 
        else:
            self.get_logger().error("No se generaron waypoints. El nodo no publicará.")
        
        # El timer revisará cada 1s si ya hay alguien suscrito
        self.timer = self.create_timer(1.0, self.publish_waypoints)
        
        self.get_logger().info(f"==> Generando {self.num_laps} vuelta(s).")


    def load_trajectory(self):
        mode = self.get_parameter('trajectory_mode').get_parameter_value().string_value
        self.get_logger().info(f"Modo de trayectoria seleccionado: '{mode}'")
        
        base_path = []

        # 1. Obtener el path base (para 1 vuelta)
        if mode == 'dxf':
            dxf_path = self.get_parameter('dxf_file').get_parameter_value().string_value
            self.get_logger().info(f"Leyendo archivo DXF: {dxf_path}")
            base_path = self.parse_dxf(dxf_path)
            # ... (advertencia de DXF) ...
        
        elif mode == 'rectangle':
            width = self.get_parameter('rectangle.width').get_parameter_value().double_value
            height = self.get_parameter('rectangle.height').get_parameter_value().double_value
            start_x = self.get_parameter('rectangle.start_x').get_parameter_value().double_value
            start_y = self.get_parameter('rectangle.start_y').get_parameter_value().double_value

            self.get_logger().info(f"Generando rectángulo base de {width}m x {height}m")
            self.get_logger().info(f"==> Punto de inicio en: ({start_x}, {start_y})")

            base_path = self.generate_rectangle_waypoints(width, height, start_x, start_y)
        
        else:
            self.get_logger().error(f"Modo '{mode}' no reconocido. No se generaron waypoints.")
            return []

        # 2. Lógica de Repetición
        if self.num_laps <= 1:
            return base_path # Retornar solo 1 vuelta

        if len(base_path) < 2:
            return base_path

        if mode == 'rectangle':
            self.get_logger().info(f"Repitiendo la trayectoria del rectángulo {self.num_laps} veces.")
            # base_path ya es [s, pX, ..., pY, s]
            # Para la siguiente vuelta, omitimos el 's' inicial
            lap_path = base_path[1:] 
            final_path = list(base_path) # Empezar con la Vuelta 1
            for i in range(self.num_laps - 1):
                final_path.extend(lap_path) # Añadir las vueltas 2..N
            self.get_logger().info(f"Path final generado con {len(final_path)} waypoints (para {self.num_laps} vueltas).")
            return final_path
        
        else: # Lógica simple de repetición para DXF
            final_path = []
            for _ in range(self.num_laps):
                final_path.extend(base_path)
            return final_path
    
    def generate_rectangle_waypoints(self, width, height, start_x, start_y):
        """
        Genera los waypoints de un rectángulo, permitiendo
        un punto de inicio arbitrario Y TRASLADANDO el path
        para que ese inicio sea (0,0).
        """
        waypoints = [] # Esta será la lista final de waypoints
        entity_type = "RECTANGLE"
        entity_id = 0
        
        # 1. Definir los 4 vértices (lógicos)
        p1 = (0.0, 0.0)
        p2 = (width, 0.0)
        p3 = (width, height)
        p4 = (0.0, height)
        
        # 2. Definir el punto de inicio (lógico)
        s = (start_x, start_y)
        
        # 3. Construir el path lógico (lista de tuplas (x,y))
        raw_path_tuples = [] 
        
        # (Esta lógica de 'if math.isclose...' es idéntica a la anterior)
        # Caso 1: 's' está en el lado inferior (entre p1 y p2)
        if math.isclose(start_y, 0.0) and (0.0 <= start_x < width):
            self.get_logger().info("Punto de inicio en LADO INFERIOR.")
            raw_path_tuples = [s, p2, p3, p4, p1, s]
            
        # Caso 2: 's' está en el lado derecho (entre p2 y p3)
        elif math.isclose(start_x, width) and (0.0 <= start_y < height):
            self.get_logger().info("Punto de inicio en LADO DERECHO.")
            raw_path_tuples = [s, p3, p4, p1, p2, s]

        # Caso 3: 's' está en el lado superior (entre p3 y p4)
        elif math.isclose(start_y, height) and (0.0 < start_x <= width):
            self.get_logger().info("Punto de inicio en LADO SUPERIOR.")
            raw_path_tuples = [s, p4, p1, p2, p3, s]
            
        # Caso 4: 's' está en el lado izquierdo (entre p4 y p1)
        elif math.isclose(start_x, 0.0) and (0.0 < start_y <= height):
            self.get_logger().info("Punto de inicio en LADO IZQUIERDO.")
            raw_path_tuples = [s, p1, p2, p3, p4, s]

        # Caso 5: 's' es (0,0) (o por defecto)
        elif math.isclose(start_x, 0.0) and math.isclose(start_y, 0.0):
             self.get_logger().info("Punto de inicio en ESQUINA (0,0).")
             raw_path_tuples = [p1, p2, p3, p4, p1]
             # 's' ya es (0.0, 0.0)
             
        else:
            self.get_logger().warn(
                f"Punto de inicio ({start_x}, {start_y}) está fuera del perímetro. "
                f"Usando (0,0) por defecto."
            )
            raw_path_tuples = [p1, p2, p3, p4, p1]
            s = (0.0, 0.0) # Forzar el offset a (0,0)

        # --- [NUEVA LÓGICA DE TRASLACIÓN] ---
        # El offset es el propio punto de inicio 's'
        offset_x = s[0]
        offset_y = s[1]
        self.get_logger().info(f"Aplicando traslación de (-{offset_x:.2f}, -{offset_y:.2f}) a todos los puntos.")

        # 4. Empaquetar los waypoints TRASLADADOS
        for p in raw_path_tuples:
            # Aplicar el offset (restar el punto de inicio)
            translated_x = p[0] - offset_x
            translated_y = p[1] - offset_y
            
            # Añadir a la lista final
            waypoints.append((translated_x, translated_y, 0.0, entity_type, entity_id))
            
        self.get_logger().info(f"Generados {len(waypoints)} waypoints base (trasladados).")
        return waypoints

    def parse_dxf(self, path):
        self.get_logger().warn("Función parse_dxf no implementada completamente en este snippet.")
        return []

    def _approximate_arc(self, cx, cy, radius, start_angle_deg, end_angle_deg, num_points=30):
        return []

    def publish_waypoints(self):
        """
        Publica el path UNA SOLA VEZ cuando detecta un suscriptor.
        """
        # Comprobar si hay suscriptores Y si tenemos un path
        if self.path_pub.get_subscription_count() > 0 and self.waypoints:
            
            self.get_logger().info(" Hay suscriptor(es), publicando trayectoria...")
            path_msg = Path()
            path_msg.header.frame_id = "odom" 
            path_msg.header.stamp = self.get_clock().now().to_msg()

            for wp in self.waypoints:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = wp[0]
                pose.pose.position.y = wp[1]
                pose.pose.position.z = wp[2]
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0 
                path_msg.poses.append(pose)

            # Publicar
            self.path_pub.publish(path_msg)
            self.get_logger().info(f"Publishing path con {len(path_msg.poses)} poses (crudo).")
            
            # --- [CORRECCIÓN CLAVE] ---
            # Detener el timer para que no se ejecute de nuevo (detiene el spam)
            self.timer.cancel()
            self.get_logger().info("Timer de publicación cancelado. El path crudo no se publicará más.")
            # --- [FIN CORRECCIÓN] ---
        
        else:
            self.get_logger().debug("Esperando suscriptores para '/dxf_path_pub'...")


    def publish_pointcloud(self):
        if not self.waypoints: return
        pc = PointCloud()
        pc.header.frame_id = "odom"
        pc.header.stamp = self.get_clock().now().to_msg()
        for wp in self.waypoints:
            pc.points.append(Point32(x=float(wp[0]), y=float(wp[1]), z=float(wp[2])))
        self.pc_pub.publish(pc)
        self.get_logger().info("PointCloud para RViz publicada (unidades en METERS).")


    def export_to_csv(self, filename='/home/alejo/diff_robot/src/diff_robot_gr03/csv/dxf_waypoints_m.csv'):
        if not self.waypoints: return
        try:
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['x_m', 'y_m', 'z_m', 'entity_type', 'entity_id']) 
                for point in self.waypoints:
                    writer.writerow([point[0], point[1], point[2], point[3], point[4]]) 
            self.get_logger().info(f"Waypoints (en METERS) exported to CSV at: {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to write CSV: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DXFParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()