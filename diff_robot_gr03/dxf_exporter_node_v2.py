
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile,ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool

import ezdxf
import pathlib 
import math
import csv 

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

class DXFParserNode(Node):
    def __init__(self):
        super().__init__('dxf_parser_node')

        self.ENTITY_TYPE_MAP = {
            "LINE": 1, "LWPOLYLINE": 2, "POLYLINE": 3,
            "ARC": 4, "CIRCLE": 5, "SPLINE": 6,
            "RECTANGLE": 7 # <-- [CAMBIO] Nuevo tipo para nuestra figura
        }
        self.flag = False
        self.waypoints = []

        # ------------------ [CAMBIO] Parámetros ------------------
        # Declaramos todos los parámetros que usará el nodo
        self.declare_parameter('trajectory_mode', 'dxf')
        self.declare_parameter('dxf_file','/home/alejo/diff_robot/src/diff_robot_gr03/dxf/xxx.dxf')
        self.declare_parameter('rectangle.width', 0.5)
        self.declare_parameter('rectangle.height', 0.3)
        # ------------------ [FIN CAMBIO] ------------------
        
        # ------------------ Publishers ------------------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        self.path_pub = self.create_publisher(Path, 'dxf_path_pub', qos) 
        self.pc_pub = self.create_publisher(PointCloud, 'dxf_pointcloud', qos)
        self.create_subscription(Bool, 'planner_ready', self.ready_callback, qos)
        
        # ------------------ [CAMBIO] Proceso ------------------
        # Llamamos a la nueva función "cerebro" que decide qué cargar
        self.waypoints = self.load_trajectory()
        
        if self.waypoints:
            self.export_to_csv()
            self.publish_pointcloud() # Publicar la nube de puntos inmediatamente
        else:
            self.get_logger().error("No se generaron waypoints. El nodo no publicará.")
        
        # El timer solo publicará el Path cuando el planner esté listo
        self.timer = self.create_timer(1.0, self.publish_waypoints)
        # ------------------ [FIN CAMBIO] ------------------

    # --- [NUEVA FUNCIÓN] ---
    def load_trajectory(self):
        """
        Lee el parámetro 'trajectory_mode' y llama a la función 
        adecuada para generar los waypoints.
        """
        mode = self.get_parameter('trajectory_mode').get_parameter_value().string_value
        self.get_logger().info(f"Modo de trayectoria seleccionado: '{mode}'")

        if mode == 'dxf':
            dxf_path = self.get_parameter('dxf_file').get_parameter_value().string_value
            self.get_logger().info(f"Leyendo archivo DXF: {dxf_path}")
            return self.parse_dxf(dxf_path)
        
        elif mode == 'rectangle':
            width = self.get_parameter('rectangle.width').get_parameter_value().double_value
            height = self.get_parameter('rectangle.height').get_parameter_value().double_value
            self.get_logger().info(f"Generando rectángulo de {width}m (ancho) x {height}m (alto)")
            return self.generate_rectangle_waypoints(width, height)
        
        else:
            self.get_logger().error(f"Modo '{mode}' no reconocido. No se generaron waypoints.")
            return []
    
    # --- [NUEVA FUNCIÓN] ---
    def generate_rectangle_waypoints(self, width, height):
        """
        Genera una lista de 4 waypoints para un rectángulo.
        Sigue la lógica: (X,0) -> (X,Y) -> (0,Y) -> (0,0)
        """
        waypoints = []
        entity_type = "RECTANGLE"
        entity_id = 0 # Todos los puntos pertenecen a la misma entidad
        
        # El robot está en (0,0) y apunta a +X
        
        # Waypoint 1: (X, 0)
        waypoints.append((width, 0.0, 0.0, entity_type, entity_id))
        # Waypoint 2: (X, Y)
        waypoints.append((width, height, 0.0, entity_type, entity_id))
        # Waypoint 3: (0, Y)
        waypoints.append((0.0, height, 0.0, entity_type, entity_id))
        # Waypoint 4: (0, 0) - Regreso a casa
        waypoints.append((0.0, 0.0, 0.0, entity_type, entity_id))
        
        self.get_logger().info(f"Generados {len(waypoints)} waypoints para el rectángulo.")
        return waypoints

    def ready_callback(self, msg: Bool):
        if msg.data:
            self.flag = True
            self.get_logger().info(" Planner indicó que está listo (planner_ready=True).")

    # --- (Esta función no se modifica en su lógica interna) ---
    def parse_dxf(self, path):
        if not pathlib.Path(path).exists():
            self.get_logger().error(f"File does not exist: {path}")
            return []
        try:
            doc = ezdxf.readfile(path)
        except Exception as e:
            self.get_logger().error(f"Failed to read DXF file: {e}")
            return []
            
        msp = doc.modelspace()
        all_entity_data = []
        entity_id_counter = 0
        
        self.get_logger().info("Extracting DXF entities (assuming METERS)...")
        for e in msp:
            entity_type = e.dxftype()
            current_entity_points = []
            
            if entity_type == 'LINE':
                current_entity_points.append((e.dxf.start.x, e.dxf.start.y, 0.0))
                current_entity_points.append((e.dxf.end.x, e.dxf.end.y, 0.0))
            elif entity_type == 'LWPOLYLINE':
                points = list(e.get_points())
                for x, y, *_ in points: current_entity_points.append((x, y, 0.0))
                if e.is_closed and len(points) > 0:
                    current_entity_points.append((points[0][0], points[0][1], 0.0))
            elif entity_type == 'POLYLINE':
                points = list(e.points())
                for p in points: current_entity_points.append((p.x, p.y, 0.0))
                if e.is_closed and len(points) > 0:
                    current_entity_points.append((points[0].x, points[0].y, 0.0))
            elif entity_type == 'ARC':
                arc_points = self._approximate_arc(e.dxf.center.x, e.dxf.center.y, e.dxf.radius, e.dxf.start_angle, e.dxf.end_angle)
                for point in arc_points: current_entity_points.append((point.x, point.y, point.z))
            elif entity_type == 'CIRCLE':
                circle_points = self._approximate_arc(e.dxf.center.x, e.dxf.center.y, e.dxf.radius, 0.0, 360.0)
                for point in circle_points: current_entity_points.append((point.x, point.y, point.z))
                if len(circle_points) > 0:
                    current_entity_points.append((circle_points[0].x, circle_points[0].y, circle_points[0].z))
            elif entity_type == 'SPLINE':
                for vec in e.flattening(0.5): current_entity_points.append((vec.x, vec.y, 0.0))
            else:
                self.get_logger().warn(f"Entity type '{entity_type}' not handled. Skipping.")
            
            if current_entity_points:
                all_entity_data.append( (current_entity_points, entity_type, entity_id_counter) )
                entity_id_counter += 1

        if not all_entity_data: return []

        # --- Lógica de ORDENAMIENTO (TSP simple) ---
        def calculate_distance(p1, p2):
            return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

        sorted_entities = []
        remaining_entities = all_entity_data.copy()
        if not remaining_entities: return []
        
        sorted_entities.append(remaining_entities.pop(0))

        while remaining_entities:
            last_point = sorted_entities[-1][0][-1]
            best_match_idx = -1
            min_dist = float('inf')
            reverse_path = False
            for i, (entity_pts, _, _) in enumerate(remaining_entities):
                dist_to_start = calculate_distance(last_point, entity_pts[0])
                dist_to_end = calculate_distance(last_point, entity_pts[-1])
                if dist_to_start < min_dist:
                    min_dist = dist_to_start; best_match_idx = i; reverse_path = False
                if dist_to_end < min_dist:
                    min_dist = dist_to_end; best_match_idx = i; reverse_path = True
            
            if best_match_idx != -1:
                next_entity = remaining_entities.pop(best_match_idx)
                if reverse_path: next_entity[0].reverse()
                sorted_entities.append(next_entity)
            else: break # No se encontró conexión

        # --- Reconstrucción de waypoints ---
        raw_waypoints = []
        for points, entity_type, entity_id in sorted_entities:
            for p in points:
                raw_waypoints.append((p[0], p[1], p[2], entity_type, entity_id))

        if not raw_waypoints:
            self.get_logger().error("No waypoints generated after sorting.")
            return []
            
        # Dejamos la normalización comentada (como la tenías)
        final_waypoints = raw_waypoints
        self.get_logger().info(f"Extracted and sorted {len(final_waypoints)} waypoints (No normalization).")

        return final_waypoints

    def _approximate_arc(self, cx, cy, radius, start_angle_deg, end_angle_deg, num_points=30):
        # ... (Esta función no cambia) ...
        start_angle = math.radians(start_angle_deg)
        end_angle = math.radians(end_angle_deg)
        if end_angle < start_angle: end_angle += 2 * math.pi
        arc_points = []
        for i in range(num_points + 1):
            angle = start_angle + i * (end_angle - start_angle) / num_points
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            arc_points.append(Point(x=x, y=y, z=0.0))
        return arc_points

    def publish_waypoints(self):
        if not self.flag:
            self.get_logger().debug("⏳ Esperando planner_ready...")
            return
        
        if self.path_pub.get_subscription_count() > 0 and self.waypoints:
            self.get_logger().info(" Hay suscriptor, publicando trayectoria...")
            path_msg = Path()
            path_msg.header.frame_id = "odom"
            path_msg.header.stamp = self.get_clock().now().to_msg()

            for wp in self.waypoints:
                pose = PoseStamped()
                pose.header = path_msg.header
                
                # Posición (corregida sin el '-' que teníamos antes)
                pose.pose.position.x = wp[0] # Asumimos metros
                pose.pose.position.y = wp[1] # Asumimos metros
                pose.pose.position.z = wp[2] # Asumimos metros

                # --- [CORRECCIÓN DEL BUG] ---
                # Ya no guardamos metadata aquí.
                # Enviamos una orientación neutral (0 grados).
                # El 'trajectory_generator' calculará la orientación correcta.
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0 
                # --- [FIN CORRECCIÓN] ---

                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)
            self.get_logger().info(f"Publishing path with {len(path_msg.poses)} poses (units in METERS).")
            self.timer.cancel() # Publica solo una vez

    def publish_pointcloud(self):
        if not self.waypoints: return
        pc = PointCloud()
        pc.header.frame_id = "odom"
        pc.header.stamp = self.get_clock().now().to_msg()

        for wp in self.waypoints:
            # --- [BUG CORREGIDO] ---
            # Eliminados los signos negativos.
            pc.points.append(Point32(
                x = float(wp[0]), # Asumimos metros
                y = float(wp[1]), # Asumimos metros
                z = float(wp[2])  # Asumimos metros
            ))
            # --- [FIN CORRECCIÓN] ---
            
        self.pc_pub.publish(pc)
        self.get_logger().info("PointCloud para RViz publicada (unidades en METERS).")

    def export_to_csv(self, filename='/home/alejo/diff_robot/src/diff_robot_gr03/csv/dxf_waypoints_m.csv'):
        if not self.waypoints: return
        try:
            with open(filename, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['x_m', 'y_m', 'z_m', 'entity_type', 'entity_id']) 
                for point in self.waypoints:
                    # --- [BUG CORREGIDO] ---
                    # Eliminados los signos negativos.
                    writer.writerow([point[0], point[1], point[2], point[3], point[4]]) 
                    # --- [FIN CORRECCIÓN] ---
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