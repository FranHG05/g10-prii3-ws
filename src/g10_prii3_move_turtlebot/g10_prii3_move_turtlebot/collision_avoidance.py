#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import math

class CollisionAvoider(Node):
    def __init__(self):
        super().__init__('collision_avoidance_node')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.draw_sub = self.create_subscription(
            Twist,
            '/draw_vel',
            self.draw_callback,
            10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        # --- Clientes de Servicio ---
        self.pause_client = self.create_client(Empty, 'pause')
        self.resume_client = self.create_client(Empty, 'resume')
        
        while not self.pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio /pause no disponible, esperando...')
        while not self.resume_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio /resume no disponible, esperando...')
        
        self.get_logger().info('Clientes de servicio /pause y /resume listos.')

        # --- Variables internas ---
        self.obstacle_detected = False
        self.obstacle_paused_drawing = False
        self.last_draw_vel = Twist()
        self.stop_vel = Twist()
        self.stop_vel.linear.x = 0.0
        self.stop_vel.angular.z = 0.0
        self.avoid_threshold = 0.35  # ← Añadido: distancia mínima de detección

        self.get_logger().info("Nodo 'collision_avoidance' listo.")

    def call_service(self, client):
        if not client.service_is_ready():
            self.get_logger().error(f"Servicio no disponible.")
            return
        client.call_async(Empty.Request())

    def draw_callback(self, msg: Twist):
        self.last_draw_vel = msg
        if not self.obstacle_detected:
            self.cmd_pub.publish(self.last_draw_vel)

    def scan_callback(self, msg: LaserScan):
        # Detectar objetos en el frente (-10 a +10 grados)
        front_angles = range(-10, 11)
        front_distances = []
        for i in front_angles:
            index = i % 360
            dist = msg.ranges[index]
            if not math.isinf(dist) and not math.isnan(dist) and dist > 0.01:
                front_distances.append(dist)

        # Calcular si hay obstáculo
        if not front_distances:
            current_obstacle_state = False
        else:
            min_dist = min(front_distances)
            current_obstacle_state = (min_dist < self.avoid_threshold)

        # --- Lógica de Pausa/Resume ---
        if current_obstacle_state and not self.obstacle_paused_drawing:
            self.obstacle_paused_drawing = True
            self.call_service(self.pause_client)
            self.cmd_pub.publish(self.stop_vel)
            self.get_logger().warn("¡Obstáculo detectado! Pausando dibujo.")

        elif not current_obstacle_state and self.obstacle_paused_drawing:
            self.obstacle_paused_drawing = False
            self.call_service(self.resume_client)
            self.get_logger().info("Obstáculo despejado. Reanudando dibujo.")
        
        self.obstacle_detected = current_obstacle_state

        if self.obstacle_detected:
            self.cmd_pub.publish(self.stop_vel)

def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoider()
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
