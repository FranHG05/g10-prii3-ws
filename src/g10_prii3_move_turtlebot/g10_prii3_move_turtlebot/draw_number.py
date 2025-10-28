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
        # Publica en /draw_vel
        self.pub = self.create_publisher(Twist, '/draw_vel', 10)
        self.vel = Twist()
        self.paused = False
        self.stop_flag = False

        # Servicios
        self.create_service(Empty, 'pause', self.pause_callback)
        self.create_service(Empty, 'resume', self.resume_callback)
        self.create_service(Empty, 'reset', self.reset_callback)
        
        self.get_logger().info("Nodo de dibujo (draw_number) v1 (Basado en Tiempo) listo.")
        self.get_logger().info("Publicando intenciones en /draw_vel")

    # --- Callbacks de servicios ---
    def pause_callback(self, request, response):
        self.paused = True
        self.get_logger().info("⏸️  Dibujo pausado.")
        return response

    def resume_callback(self, request, response):
        self.paused = False
        self.get_logger().info("▶️  Dibujo reanudado.")
        return response

    def reset_callback(self, request, response):
        self.stop_flag = True
        self.paused = False
        self.get_logger().info("🔄  Reiniciando dibujo.")
        return response

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
            
            if self.paused:
                self.stop() # Publica 0.0
                time.sleep(dt)
                continue # Se salta el resto del bucle y no suma tiempo
            
            # Si no está pausado, se mueve
            self.vel.linear.x = speed
            self.vel.angular.z = 0.0
            self.pub.publish(self.vel)
            
            elapsed += dt
            time.sleep(dt)
            
        self.stop()

    def turn(self, angle_deg, angular_speed=0.4):
        angle_rad = math.radians(angle_deg)
        duration = abs(angle_rad) / angular_speed
        direction = 1 if angle_rad > 0 else -1
        
        elapsed = 0.0
        dt = 0.05
        
        while elapsed < duration and rclpy.ok():
            if self.stop_flag:
                break

            if self.paused:
                self.stop() # Publica 0.0
                time.sleep(dt)
                continue # Se salta el resto del bucle y no suma tiempo
            
            # Si no está pausado, gira
            self.vel.linear.x = 0.0
            self.vel.angular.z = direction * angular_speed
            self.pub.publish(self.vel)
            
            elapsed += dt
            time.sleep(dt)

        self.stop()

    # --- Dibujo del número 10 (Basado en Tiempo) ---
    def draw_number(self):
        # Ajusta estas duraciones (en segundos)
        steps = [
            (self.forward, 5.0),  # Dibuja el '1' (largo)
            (self.turn, 90),
            (self.forward, 1.0),
            (self.turn, 90),
            (self.forward, 5.0),
            (self.turn, -90),     # Se mueve para empezar el '0'
            (self.forward, 2.5),
            (self.turn, -90),     # Dibuja el '0'
            (self.forward, 5.0),
            (self.turn, -90),
            (self.forward, 2.5),
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