#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import math
import time


class GoToPose(Node):
    def __init__(self):
        super().__init__('multi_goal_client')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Lista de puntos (x, y, theta)
        self.goals = [
            (1.0, 2.0, 180),
            (0.0, 0.0, 90),
            (2.5, -1.0, 0)
        ]

        self.current_goal_index = 0
        self.tolerance = 0.25  # metros
        self.last_feedback_pose = None

    def send_next_goal(self):
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info('✅ Todos los puntos alcanzados.')
            rclpy.shutdown()
            return

        x, y, theta_deg = self.goals[self.current_goal_index]
        self.get_logger().info(
            f'🟢 Enviando punto {self.current_goal_index + 1}/{len(self.goals)}: ({x}, {y}, {theta_deg}°)'
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        theta = math.radians(theta_deg)
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ Objetivo rechazado.')
            return

        self.get_logger().info('🚀 Objetivo aceptado. Navegando...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        pose = feedback_msg.feedback.current_pose.pose.position
        self.last_feedback_pose = pose
        self.get_logger().info(f'→ Avanzando: x={pose.x:.2f}, y={pose.y:.2f}')

    def get_result_callback(self, future):
        self.get_logger().info(
            f'🟡 Punto {self.current_goal_index + 1} completado por Nav2. Verificando distancia...'
        )

        # Esperar un poco para estabilizar la pose final
        time.sleep(1.0)

        if self.last_feedback_pose is None:
            self.get_logger().warn('⚠️ No se recibió feedback de posición, continuando igualmente...')
        else:
            goal_x, goal_y, _ = self.goals[self.current_goal_index]
            dist = math.sqrt(
                (self.last_feedback_pose.x - goal_x) ** 2 +
                (self.last_feedback_pose.y - goal_y) ** 2
            )
            self.get_logger().info(f'📏 Distancia al objetivo: {dist:.2f} m')

            if dist > self.tolerance:
                self.get_logger().warn('⚠️ El robot aún está lejos. Reintentando este punto...')
                self.send_next_goal()  # Reintenta el mismo punto
                return

        self.get_logger().info(f'✅ Punto {self.current_goal_index + 1} alcanzado con éxito.')
        self.current_goal_index += 1
        time.sleep(1.0)
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = GoToPose()
    node.send_next_goal()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
