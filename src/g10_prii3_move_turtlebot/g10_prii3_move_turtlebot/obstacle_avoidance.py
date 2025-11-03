#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import math
import time

# --- Definimos los estados del robot ---
STATE_DRAWING = 0
STATE_AVOIDING = 1

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.draw_sub = self.create_subscription(Twist, '/draw_vel', self.draw_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.pause_client = self.create_client(Empty, 'pause')
        self.resume_client = self.create_client(Empty, 'resume')
        while not self.pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio /pause no disponible, esperando...')
        while not self.resume_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio /resume no disponible, esperando...')

        self.robot_state = STATE_DRAWING
        self.drawer_paused = False
        self.last_draw_vel = Twist()
        self.obstacle_detected = False

        self.avoid_threshold = 0.35  # Distancia para detectar el obstáculo

        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Nodo 'obstacle_avoidance' listo.")

    def call_service(self, client):
        if not client.service_is_ready():
            return
        client.call_async(Empty.Request())

    def draw_callback(self, msg: Twist):
        self.last_draw_vel = msg

    def scan_callback(self, msg: LaserScan):
        # Detectar objetos estrictamente en el frente (-10 a +10 grados)
        front_angles = range(-10, 11)  # Ajustado para un rango más estrecho
        front_distances = []
        for i in front_angles:
            index = i % 360  # Maneja ángulos (0-359)
            dist = msg.ranges[index]
            if not math.isinf(dist) and not math.isnan(dist) and dist > 0.01:
                front_distances.append(dist)

        if not front_distances:
            self.obstacle_detected = False
        else:
            min_dist = min(front_distances)
            self.obstacle_detected = (min_dist < self.avoid_threshold)  # True si hay obstáculo cerca

    def control_loop(self):
        vel_cmd = Twist()

        if self.robot_state == STATE_DRAWING:
            vel_cmd = self.last_draw_vel

            if self.obstacle_detected:
                self.get_logger().warn("¡Obstáculo detectado! Estado -> AVOIDING")
                self.robot_state = STATE_AVOIDING
                if not self.drawer_paused:
                    self.call_service(self.pause_client)
                    self.drawer_paused = True

        elif self.robot_state == STATE_AVOIDING:
            self.obstacle_detected = False  # Desactivar detección temporalmente
            self.avoid_obstacle()
            self.robot_state = STATE_DRAWING
            if self.drawer_paused:
                self.call_service(self.resume_client)
                self.drawer_paused = False

        self.cmd_pub.publish(vel_cmd)

    def avoid_obstacle(self):
        # Secuencia de movimientos para esquivar el obstáculo
        self.get_logger().info("Esquivando obstáculo con movimientos predefinidos.")
        self.stop()
        self.turn(90)
        self.forward(2.5)
        self.turn(-90)
        self.forward(5.0)
        self.turn(-90)
        self.forward(2.5)
        self.turn(90)
        self.obstacle_detected = False  # Asegurar que no se detecten obstáculos residuales

    def stop(self):
        # Detener el robot
        self.execute_movement(0.0, 0.0, duration=0.5)

    def forward(self, duration, speed=0.15):
        # Avanzar hacia adelante
        self.execute_movement(speed, 0.0, duration)

    def turn(self, angle_deg, angular_speed=0.4):
        # Girar un ángulo específico
        angle_rad = math.radians(angle_deg)
        duration = abs(angle_rad) / angular_speed
        direction = 1 if angle_rad > 0 else -1
        self.execute_movement(0.0, direction * angular_speed, duration)

    def execute_movement(self, linear_speed, angular_speed, duration):
        vel_cmd = Twist()
        vel_cmd.linear.x = linear_speed
        vel_cmd.angular.z = angular_speed

        start_time = time.time()
        while time.time() - start_time < duration and rclpy.ok():
            self.cmd_pub.publish(vel_cmd)
            time.sleep(0.1)

        # Detener el robot después del movimiento
        self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
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