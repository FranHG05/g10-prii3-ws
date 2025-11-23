#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math
import time


def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) - math.cos(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
    qy = math.cos(roll/2)*math.sin(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.cos(pitch/2)*math.sin(yaw/2)
    qz = math.cos(roll/2)*math.cos(pitch/2)*math.sin(yaw/2) - math.sin(roll/2)*math.sin(pitch/2)*math.cos(yaw/2)
    qw = math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
    return qx, qy, qz, qw


class AutoNavAruco(Node):

    def __init__(self):
        super().__init__('autonav_aruco')

        # --- NAVIGATION2 CLIENT ---
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # --- ARUCO ---
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
        self.parameters = cv2.aruco.DetectorParameters_create()

        self.looking_for_aruco = False
        self.detected_id = None

        self.get_logger().info("Nodo iniciado. Preparando navegación…")

        # Esperar al servidor de Nav2
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Esperando al servidor de Navigation2...')

        self.first_pose = (3.20, 2.30, math.pi/2)

        self.get_logger().info("Moviéndose a la posición inicial para detectar ArUco…")
        self.send_goal(self.first_pose, callback=self.after_first_pose)

    # --- ARUCO CALLBACK ---
    def image_callback(self, msg):
        if not self.looking_for_aruco:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)

        if ids is not None and self.detected_id is None:
            self.detected_id = ids[0][0]
            self.get_logger().info(f"ArUco detectado: ID = {self.detected_id}")
            self.looking_for_aruco = False
            self.define_waypoints_and_start()

    # --- NAVEGACIÓN ---
    def send_goal(self, pose_tuple, callback=None):
        x, y, yaw = pose_tuple

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        qx, qy, qz, qw = euler_to_quaternion(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(f"Enviando goal: x={x}, y={y}, yaw={yaw:.2f}")

        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_callback(f, callback))

    def goal_response_callback(self, future, callback):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rechazado.")
            return

        self.get_logger().info("Goal aceptado.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.goal_result_callback(f, callback))

    def goal_result_callback(self, future, callback):
        self.get_logger().info("Goal alcanzado.")
        time.sleep(1.0)
        if callback:
            callback()

    # --- TRAS LLEGAR A LA PRIMERA POSE ---
    def after_first_pose(self):
        self.get_logger().info("En posición inicial. Buscando ArUco…")
        self.looking_for_aruco = True

    # --- DEFINIR RUTA SEGÚN EL ID ---
    def define_waypoints_and_start(self):
        id = self.detected_id

        if id == 5:
            self.waypoints = [(-3.0, 6.0, math.pi)]
        elif id == 17:
            self.waypoints = [
                (3.10, 15.0, math.pi/2),
                (4.0, 15.0, 0.0)
            ]
        elif id == 6:
            self.waypoints = [
                (3.10, 15.0, math.pi/2),
                (-3.0, 15.0, math.pi)
            ]
        else:
            self.get_logger().warn(f"ID {id} no reconocido. No se ha definido ruta.")
            self.waypoints = []

        self.get_logger().info(f"Ruta definida para ID {id}. Iniciando navegación…")
        self.current_wp = 0
        self.go_to_next_wp()

    # --- SIGUIENTE WAYPOINT ---
    def go_to_next_wp(self):
        if self.current_wp >= len(self.waypoints):
            self.get_logger().info("Ruta completada.")
            rclpy.shutdown()
            return

        pose = self.waypoints[self.current_wp]
        self.current_wp += 1
        self.send_goal(pose, callback=self.go_to_next_wp)


def main(args=None):
    rclpy.init(args=args)
    node = AutoNavAruco()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
