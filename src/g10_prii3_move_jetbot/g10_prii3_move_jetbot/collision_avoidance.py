#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import math
import rclpy.time # <-- AÑADIDO: Para el watchdog

class CollisionAvoiderJetbot(Node):
    def __init__(self):
        super().__init__('collision_avoidance_node_jetbot')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.draw_sub = self.create_subscription(Twist, '/draw_vel', self.draw_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.pause_client = self.create_client(Empty, 'pause')
        self.resume_client = self.create_client(Empty, 'resume')
        
        while not self.pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio /pause no disponible, esperando...')
        while not self.resume_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio /resume no disponible, esperando...')
        
        self.get_logger().info('Clientes de servicio /pause y /resume listos.')

        self.obstacle_detected = False
        self.obstacle_paused_drawing = False
        self.last_draw_vel = Twist()
        self.stop_vel = Twist()
        self.stop_vel.linear.x = 0.0
        self.stop_vel.angular.z = 0.0
        
        # --- AÑADIDO: Variables del Watchdog ---
        self.last_scan_time = self.get_clock().now()
        self.lidar_timeout_sec = 2.0 # 2 segundos de tolerancia
        self.lidar_active = False # Flag para el primer mensaje
        
        # El watchdog comprueba el estado del LiDAR cada 1 segundo
        self.watchdog_timer = self.create_timer(1.0, self.watchdog_callback)
        # --- FIN AÑADIDO ---

        self.get_logger().info("Nodo 'collision_avoidance' (Jetbot) listo.")
        self.get_logger().warn("Esperando datos del LiDAR en /scan...")

    def call_service(self, client):
        if not client.service_is_ready():
            return
        client.call_async(Empty.Request())

    def draw_callback(self, msg: Twist):
        self.last_draw_vel = msg
        if not self.obstacle_detected:
            self.cmd_pub.publish(self.last_draw_vel)

    # --- AÑADIDO: Función del Watchdog ---
    def watchdog_callback(self):
        # Comprueba si el LiDAR está activo
        if not self.lidar_active:
            # Si no está activo y no ha forzado la parada, no hagas nada
            # (El log de "Esperando datos" ya se mostró en __init__)
            return

        # Si el LiDAR SÍ estaba activo, comprueba el tiempo
        now = self.get_clock().now()
        time_diff_sec = (now - self.last_scan_time).nanoseconds / 1e9

        if time_diff_sec > self.lidar_timeout_sec:
            # ¡Fallo del LiDAR! Forzar parada.
            self.get_logger().error(f"¡Se perdió la conexión del LiDAR! ({time_diff_sec:.1f}s sin datos).")
            self.force_stop_due_to_lidar_failure()
    
    def force_stop_due_to_lidar_failure(self):
        """Función de seguridad que para el robot y pausa el dibujo."""
        self.obstacle_detected = True # Finge un obstáculo
        self.cmd_pub.publish(self.stop_vel)
        
        if not self.obstacle_paused_drawing:
            self.obstacle_paused_drawing = True
            self.call_service(self.pause_client)
            self.get_logger().warn("Pausando dibujo por seguridad (Fallo de LiDAR).")
    # --- FIN AÑADIDO ---

    def scan_callback(self, msg: LaserScan):
        # --- AÑADIDO: Comprobación de conexión ---
        # Resetea el temporizador del watchdog
        self.last_scan_time = self.get_clock().now()
        
        # Mensaje de "Conectado y leyendo" (se ejecuta solo una vez)
        if not self.lidar_active:
            self.lidar_active = True
            self.get_logger().info("¡Conexión con LiDAR establecida! Leyendo datos.")
        # --- FIN AÑADIDO ---
        
        if not msg.ranges:
            return

        try:
            indices_per_degree = 1.0 / math.degrees(msg.angle_increment)
            check_range = int(15 * indices_per_degree)
        except ZeroDivisionError:
            check_range = 10 

        front_positive = msg.ranges[0:check_range]
        front_negative = msg.ranges[-check_range:]
        
        front_distances = front_positive + front_negative
        
        valid_distances = [d for d in front_distances if d > msg.range_min]

        if not valid_distances:
            current_obstacle_state = False
        else:
            min_dist = min(valid_distances)
            current_obstacle_state = (min_dist < 0.20) # Tu umbral de 20cm

        # --- Lógica de Pausa/Resume (Casi idéntica) ---
        
        # Transición: Detectar obstáculo
        if current_obstacle_state and not self.obstacle_paused_drawing:
            self.obstacle_paused_drawing = True
            self.call_service(self.pause_client)
            self.cmd_pub.publish(self.stop_vel)
            self.get_logger().warn("¡Obstáculo detectado! Pausando dibujo.")

        # Transición: Obstáculo desaparece
        elif not current_obstacle_state and self.obstacle_paused_drawing:
            self.obstacle_paused_drawing = False
            self.call_service(self.resume_client)
            self.get_logger().info("Obstáculo despejado. Reanudando dibujo.")
        
        self.obstacle_detected = current_obstacle_state

        if self.obstacle_detected:
             self.cmd_pub.publish(self.stop_vel)

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