#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import math

class CollisionAvoiderJetbot(Node):
    def __init__(self):
        super().__init__('collision_avoidance_node_jetbot')

        # --- Publicador y suscriptores ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.draw_sub = self.create_subscription(Twist, '/draw_vel', self.draw_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # --- Clientes de servicio ---
        self.pause_client = self.create_client(Empty, 'pause')
        self.resume_client = self.create_client(Empty, 'resume')

        while not self.pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /pause...')
        while not self.resume_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /resume...')

        self.get_logger().info('Clientes /pause y /resume disponibles.')

        # --- Variables internas ---
        self.last_draw_vel = Twist()
        self.stop_vel = Twist()
        self.stop_vel.linear.x = 0.0
        self.stop_vel.angular.z = 0.0

        self.obstacle_detected = False
        self.obstacle_paused_drawing = False
        self.avoid_threshold = 0.20  # Umbral de detección (20 cm)

        # --- Variables del watchdog LiDAR ---
        self.last_scan_time = self.get_clock().now()
        self.lidar_timeout_sec = 2.0
        self.lidar_active = False

        # Temporizador del watchdog
        self.create_timer(1.0, self.watchdog_callback)

        self.get_logger().info("Nodo de evitación de colisiones (Jetbot) listo.")
        self.get_logger().warn("Esperando datos del LiDAR...")

    # --- Funciones principales ---

    def call_service(self, client):
        if client.service_is_ready():
            client.call_async(Empty.Request())

    def draw_callback(self, msg: Twist):
        """Recibe las velocidades deseadas y las reenvía si no hay obstáculo."""
        self.last_draw_vel = msg
        if not self.obstacle_detected:
            self.cmd_pub.publish(msg)

    def scan_callback(self, msg: LaserScan):
        """Procesa los datos del LiDAR y gestiona la pausa/reanudación."""
        # --- Watchdog: marca último mensaje recibido ---
        self.last_scan_time = self.get_clock().now()
        if not self.lidar_active:
            self.lidar_active = True
            self.get_logger().info("LiDAR conectado y enviando datos.")

        if not msg.ranges:
            return

        # --- Detección de obstáculos en el frente ---
        try:
            indices_per_degree = 1.0 / math.degrees(msg.angle_increment)
            check_range = int(15 * indices_per_degree)
        except ZeroDivisionError:
            check_range = 10

        front_positive = msg.ranges[0:check_range]
        front_negative = msg.ranges[-check_range:]
        front_distances = front_positive + front_negative
        valid = [d for d in front_distances if d > msg.range_min and not math.isinf(d) and not math.isnan(d)]

        if not valid:
            current_obstacle_state = False
        else:
            min_dist = min(valid)
            current_obstacle_state = (min_dist < self.avoid_threshold)

        # --- Lógica de control (igual que en el TurtleBot) ---
        if current_obstacle_state and not self.obstacle_paused_drawing:
            self.obstacle_paused_drawing = True
            self.call_service(self.pause_client)
            self.cmd_pub.publish(self.stop_vel)
            self.get_logger().warn(f"¡Obstáculo a {min_dist:.2f} m! Pausando dibujo.")

        elif not current_obstacle_state and self.obstacle_paused_drawing:
            self.obstacle_paused_drawing = False
            self.call_service(self.resume_client)
            self.get_logger().info("Obstáculo despejado. Reanudando dibujo.")

        self.obstacle_detected = current_obstacle_state

        if self.obstacle_detected:
            self.cmd_pub.publish(self.stop_vel)

    def watchdog_callback(self):
        """Verifica que el LiDAR siga activo."""
        if not self.lidar_active:
            return
        now = self.get_clock().now()
        time_diff = (now - self.last_scan_time).nanoseconds / 1e9
        if time_diff > self.lidar_timeout_sec:
            self.force_stop_due_to_lidar_failure(time_diff)

    def force_stop_due_to_lidar_failure(self, diff):
        """Detiene el robot si se pierde la conexión del LiDAR."""
        self.get_logger().error(f"¡Fallo del LiDAR! {diff:.1f}s sin datos.")
        if not self.obstacle_paused_drawing:
            self.obstacle_paused_drawing = True
            self.call_service(self.pause_client)
        self.cmd_pub.publish(self.stop_vel)
        self.obstacle_detected = True

def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoiderJetbot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
