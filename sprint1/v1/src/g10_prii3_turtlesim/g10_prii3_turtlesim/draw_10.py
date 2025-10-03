#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class TurtleNumber(Node):
    def __init__(self):
        super().__init__('turtle_number')
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.vel = Twist()

    def forward(self, duration, speed=2.0):
        """Avanza recto durante X segundos"""
        self.vel.linear.x = speed
        self.vel.angular.z = 0.0
        end_time = time.time() + duration
        while time.time() < end_time:
            self.pub.publish(self.vel)
            time.sleep(0.05)
        self.stop()

    def turn(self, angle_deg, speed=1.5):
        """Gira en grados (positivo = izquierda, negativo = derecha)"""
        angle_rad = math.radians(angle_deg)
        # tiempo = ángulo / velocidad
        duration = abs(angle_rad) / speed
        self.vel.linear.x = 0.0
        self.vel.angular.z = speed if angle_rad > 0 else -speed
        end_time = time.time() + duration
        while time.time() < end_time:
            self.pub.publish(self.vel)
            time.sleep(0.05)
        self.stop()

    def stop(self):
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.pub.publish(self.vel)
        time.sleep(0.1)

    def draw_number(self):
        self.turn(90)
        self.forward(1.5)
        self.turn(-90)
        self.forward(1)
        self.turn(-90)
        self.forward(3)
        self.turn(-90)
        self.forward(1)
        self.turn(-90)
        self.forward(1.5)
        self.turn(90)
        self.forward(1)
        self.turn(-90)
        self.forward(1.5)
        self.turn(180)
        self.forward(3)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNumber()
    time.sleep(1)  # pequeña pausa antes de empezar
    node.draw_number()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()