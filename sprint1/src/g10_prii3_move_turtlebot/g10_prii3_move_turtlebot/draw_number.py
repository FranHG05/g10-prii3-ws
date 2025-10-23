#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import time
import math
import threading

class TurtleNumber(Node):
    def __init__(self):
        super().__init__('turtle_number')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vel = Twist()
        self.paused = False       # Pausa manual
        self.stop_flag = False    # Reinicio
        self.obstacle = False     # Obstáculo detectado

        # Servicios
        self.create_service(Empty, 'pause', self.pause_callback)
        self.create_service(Empty, 'resume', self.resume_callback)
        self.create_service(Empty, 'reset', self.reset_callback)
        self.get_logger().info("Servicios disponibles: /pause, /resume, /reset")

        # Suscripción al LIDAR
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

    # --- Callbacks de servicios ---
    def pause_callback(self, request, response):
        self.paused = True
        self.get_logger().info("⏸️  Simulación pausada (manual).")
        return response

    def resume_callback(self, request, response):
        self.paused = False
        self.get_logger().info("▶️  Simulación reanudada (manual).")
        return response

    def reset_callback(self, request, response):
        self.stop_flag = True
        self.paused = False
        self.get_logger().info("🔄  Reiniciando dibujo.")
        return response

    # --- Callback del LIDAR ---
    def lidar_callback(self, msg: LaserScan):
        # Detectar obstáculos frente al robot
        front_angles = range(-10, 11)  # grados frontales
        front_distances = []
        for i in front_angles:
            index = (i - int(math.degrees(msg.angle_min))) % len(msg.ranges)
            front_distances.append(msg.ranges[index])
        # Objeto a menos de 0.3 metros
        self.obstacle = any(d < 0.3 for d in front_distances if d > 0)

    # --- Movimiento ---
    def stop(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.pub.publish(self.vel)

    def forward(self, duration, speed=0.15):
        elapsed = 0.0
        dt = 0.05
        while elapsed < duration and rclpy.ok():
            if self.stop_flag:
                break
            if not self.paused and not self.obstacle:
                self.vel.linear.x = speed
                self.vel.angular.z = 0.0
                elapsed += dt
            else:
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0
            self.pub.publish(self.vel)
            time.sleep(dt)
        self.stop()

    def turn(self, angle_deg, angular_speed=0.5):
        angle_rad = math.radians(angle_deg)
        duration = abs(angle_rad) / angular_speed
        direction = 1 if angle_rad > 0 else -1
        elapsed = 0.0
        dt = 0.05
        while elapsed < duration and rclpy.ok():
            if self.stop_flag:
                break
            if not self.paused and not self.obstacle:
                self.vel.linear.x = 0.0
                self.vel.angular.z = direction * angular_speed
                elapsed += dt
            else:
                self.vel.linear.x = 0.0
                self.vel.angular.z = 0.0
            self.pub.publish(self.vel)
            time.sleep(dt)
        self.stop()

    # --- Dibujo del número 10 ---
    def draw_number(self):
        steps = [
            (self.forward, 3.0),
            (self.turn, 90),
            (self.forward, 1.0),
            (self.turn, 90),
            (self.forward, 3.0),
            (self.turn, -90),
            (self.forward, 1.5),
            (self.turn, -90),
            (self.forward, 3.0),
            (self.turn, -90),
            (self.forward, 1.5),
        ]

        while rclpy.ok():
            step_index = 0
            while step_index < len(steps):
                func, value = steps[step_index]
                func(value)
                if self.stop_flag:
                    self.stop_flag = False
                    step_index = 0
                    continue
                step_index += 1
            while not self.stop_flag and rclpy.ok():
                time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNumber()
    thread = threading.Thread(target=node.draw_number, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
