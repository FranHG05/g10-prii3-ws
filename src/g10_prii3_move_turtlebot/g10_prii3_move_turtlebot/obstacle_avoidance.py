#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import math

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
        
        self.wall_dist = 999.0
        
        # --- Parámetros (Calibración Suave) ---
        self.target_wall_distance = 0.45
        self.wall_follow_speed = 0.08
        self.kp = 1.0
        self.max_angular_correction = 0.4
        self.avoid_threshold = 0.7
        self.wall_lost_threshold = 1.2

        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Nodo 'obstacle_avoidance' (v11 - Lógica Unificada) listo.")

    def call_service(self, client):
        if not client.service_is_ready():
            return
        client.call_async(Empty.Request())

    def draw_callback(self, msg: Twist):
        self.last_draw_vel = msg

    def get_min_distance(self, ranges, start_angle, end_angle):
        min_dist = float('inf')
        for i in range(start_angle, end_angle + 1):
            index = i % 360
            dist = ranges[index]
            if not math.isinf(dist) and not math.isnan(dist) and dist > 0.01:
                if dist < min_dist:
                    min_dist = dist
        return min_dist

    def scan_callback(self, msg: LaserScan):
        # Sensor único: Cuadrante frontal-derecho (-90 a +20)
        self.wall_dist = min(
            self.get_min_distance(msg.ranges, 270, 359),
            self.get_min_distance(msg.ranges, 0, 20)
        )

    def control_loop(self):
        vel_cmd = Twist()
        
        if self.robot_state == STATE_DRAWING:
            vel_cmd = self.last_draw_vel
            
            if self.wall_dist < self.avoid_threshold:
                self.get_logger().warn("¡Obstáculo detectado! Estado -> AVOIDING")
                self.robot_state = STATE_AVOIDING
                if not self.drawer_paused:
                    self.call_service(self.pause_client)
                    self.drawer_paused = True
        
        elif self.robot_state == STATE_AVOIDING:
            
            if self.wall_dist < self.wall_lost_threshold:
                # Prioridad 1: Hay un objeto, seguirlo
                self.get_logger().info(f"Siguiendo pared. Dist: {self.wall_dist:.2f}", throttle_duration_sec=1)
                vel_cmd.linear.x = self.wall_follow_speed
                
                error = self.target_wall_distance - self.wall_dist
                correction = self.kp * error
                
                if correction > self.max_angular_correction: correction = self.max_angular_correction
                if correction < -self.max_angular_correction: correction = -self.max_angular_correction
                
                vel_cmd.angular.z = correction
            
            else:
                # Prioridad 2: PARED PERDIDA
                self.get_logger().info("Pared perdida (Obstáculo superado). Estado -> DRAWING")
                self.robot_state = STATE_DRAWING
                if self.drawer_paused:
                    self.call_service(self.resume_client)
                    self.drawer_paused = False
                
        self.cmd_pub.publish(vel_cmd)

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