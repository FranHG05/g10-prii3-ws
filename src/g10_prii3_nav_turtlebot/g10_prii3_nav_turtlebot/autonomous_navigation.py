#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import math
import time

def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - \
         math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - \
         math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + \
         math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return qx, qy, qz, qw

class AutoNav(Node):
    def __init__(self):
        super().__init__('autonomous_navigation')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.bridge = CvBridge()
        self.aruco_detected = False
        self.image_received = None

        # Subscribirse a la cámara
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Esperando al servidor de Navigation2...')
        self.get_logger().info('Servidor de Navigation2 activo')

        # Pose 1
        self.pose1 = (3.1, 1.5, math.pi/2)
        self.pose2 = (-3.0, 6.0, math.pi)          # Si NO hay ArUco
        self.pose3 = (-3.0, 15.0, math.pi)         # Si hay ArUco

        self.go_to_pose(self.pose1, self.after_pose1)

    def camera_callback(self, msg):
        self.image_received = msg

    def detect_aruco(self):
        if self.image_received is None:
            return False
        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.image_received, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error convirtiendo la imagen: {e}")
            return False

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            self.get_logger().info(f"Código ArUco detectado: {ids.flatten()}")
            return True
        return False

    def go_to_pose(self, pose, callback=None):
        x, y, yaw = pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = euler_to_quaternion(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f'Enviando goal: x={x}, y={y}, yaw={yaw}')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        if callback:
            send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, callback))
        else:
            send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, lambda _: None))

    def goal_response_callback(self, future, callback):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rechazado')
            return
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(lambda fut: callback())

    def after_pose1(self):
        time.sleep(1.0)  # Espera breve para estabilizar la cámara
        self.aruco_detected = self.detect_aruco()
        if self.aruco_detected:
            self.get_logger().info("ArUco detectado → yendo a pose3")
            self.go_to_pose(self.pose3, self.finish)
        else:
            self.get_logger().info("No se detectó ArUco → yendo a pose2")
            self.go_to_pose(self.pose2, self.finish)

    def finish(self):
        self.get_logger().info("Trayectoria finalizada")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = AutoNav()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
