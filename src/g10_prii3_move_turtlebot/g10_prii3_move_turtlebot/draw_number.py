#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import time
import math
import threading

class TurtleNumber(Node):
    def __init__(self):
        super().__init__('draw_number_node')
        self.pub = self.create_publisher(Twist, '/draw_vel', 10)
        self.vel = Twist()
        self.paused = False
        self.stop_flag = False

        # Servicios
        self.create_service(Empty, 'pause', self.pause_callback)
        self.create_service(Empty, 'resume', self.resume_callback)
        self.create_service(Empty, 'reset', self.reset_callback)

        self.get_logger().info("Nodo de dibujo listo.")
        self.get_logger().info("Publicando intenciones en /draw_vel")

    # --- Callbacks de servicios ---
    def pause_callback(self, request, response):
        self.paused = True
        self.get_logger().info("Dibujo pausado.")
        return response

    def resume_callback(self, request, response):
        self.paused = False
        self.get_logger().info("Dibujo reanudado.")
        return response

    def reset_callback(self, request, response):
        self.stop_flag = True
        self.paused = False
        self.get_logger().info("Reiniciando dibujo.")
        return response

    # --- Movimiento ---
    def stop(self):
        self.publish_velocity(0.0, 0.0)

    def forward(self, duration, speed=0.15):
        self.move(duration, linear_speed=speed)

    def turn(self, angle_deg, angular_speed=0.4):
        angle_rad = math.radians(angle_deg)
        duration = abs(angle_rad) / angular_speed
        direction = 1 if angle_rad > 0 else -1
        self.move(duration, angular_speed=direction * angular_speed)

    def move(self, duration, linear_speed=0.0, angular_speed=0.0):
        elapsed = 0.0
        dt = 0.05

        while elapsed < duration and rclpy.ok():
            if self.stop_flag:
                break

            if self.paused:
                self.stop()
                time.sleep(dt)
                continue

            self.publish_velocity(linear_speed, angular_speed)
            elapsed += dt
            time.sleep(dt)

        self.stop()

    def publish_velocity(self, linear, angular):
        self.vel.linear.x = linear
        self.vel.angular.z = angular
        self.pub.publish(self.vel)

    # --- Dibujo del número ---
    def draw_number(self):
        steps = self.get_drawing_steps()

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

    def get_drawing_steps(self):
        return [
            (self.forward, 8.0),
            (self.turn, -90),
            (self.forward, 2.0),
            (self.turn, -90),
            (self.forward, 8.0),
            (self.turn, 90),
            (self.forward, 4.0),
            (self.turn, 90),
            (self.forward, 8.0),
            (self.turn, 90),
            (self.forward, 4.0),
        ]

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