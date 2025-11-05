#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
import math
import time

# Estados del robot
STATE_DRAWING = 0
STATE_AVOIDING = 1

class ObstacleAvoiderJetbot(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node_jetbot')

        # Publicador y suscriptores
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.draw_sub = self.create_subscription(Twist, '/draw_vel', self.draw_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Clientes de servicio (control del nodo que dibuja)
        self.pause_client = self.create_client(Empty, 'pause')
        self.resume_client = self.create_client(Empty, 'resume')

        # Esperar servicios
        while not self.pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /pause...')
        while not self.resume_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /resume...')

        self.get_logger().info('Clientes /pause y /resume disponibles.')

        # Variables de estado
        self.robot_state = STATE_DRAWING
        self.drawer_paused = False
        self.last_draw_vel = Twist()
        self.obstacle_detected = False

        # Parámetros ajustables (tunea para tu JetBot)
        self.avoid_threshold = 0.30      # metros: distancia a partir de la cual se considera obstáculo (ajustable)
        self.front_angle_deg = 15       # grados a cada lado para considerar "frontal"
        self.linear_speed = 0.10        # m/s para avanzar durante la maniobra
        self.angular_speed = 0.4        # rad/s para giros
        self.move_dt = 0.05             # ciclo de publicación de movimientos (s)

        # Watchdog LiDAR
        self.last_scan_time = self.get_clock().now()
        self.lidar_timeout_sec = 2.0
        self.lidar_active = False

        # Temporizadores
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.watchdog_timer = self.create_timer(1.0, self.watchdog_callback)

        self.get_logger().info("Nodo 'obstacle_avoidance' (JetBot) listo.")

    def call_service(self, client):
        if client.service_is_ready():
            client.call_async(Empty.Request())

    def draw_callback(self, msg: Twist):
        # Guardamos la intención de movimiento del dibujador
        self.last_draw_vel = msg

    def scan_callback(self, msg: LaserScan):
        # Watchdog: actualizamos tiempo del último mensaje
        self.last_scan_time = self.get_clock().now()
        if not self.lidar_active:
            self.lidar_active = True
            self.get_logger().info("LiDAR activo: recibiendo datos de /scan")

        # Protección por si ranges está vacío
        if not msg.ranges:
            return

        # Calcular índices dinámicamente según angle_increment
        try:
            indices_per_degree = 1.0 / math.degrees(msg.angle_increment)
            check_range = int(self.front_angle_deg * indices_per_degree)
        except Exception:
            check_range = int(self.front_angle_deg)  # fallback

        # Obtener distancias frontales (positivas y negativas alrededor del índice 0)
        # Nota: msg.ranges[0] corresponde a angle_min; algunos LiDARs centran 0 en frente.
        # Para robustez intentamos indexar alrededor del centro si angle_min no es 0:
        total_points = len(msg.ranges)
        # Índice central aproximado
        mid_index = int((0.0 - msg.angle_min) / msg.angle_increment) if msg.angle_increment != 0 else total_points // 2
        start = max(0, mid_index - check_range)
        end = min(total_points, mid_index + check_range + 1)
        front_slice = msg.ranges[start:end]

        # Filtrar valores válidos
        valid = [d for d in front_slice if (d is not None) and (not math.isinf(d)) and (not math.isnan(d)) and (d >= msg.range_min)]

        if not valid:
            self.obstacle_detected = False
        else:
            min_dist = min(valid)
            self.obstacle_detected = (min_dist < self.avoid_threshold)

    def control_loop(self):
        # Bucle principal de control: si dibujando, pasar velocidades; si esquivando, ejecutar maniobra
        vel_cmd = Twist()

        if self.robot_state == STATE_DRAWING:
            vel_cmd = self.last_draw_vel

            if self.obstacle_detected:
                self.get_logger().warn("Obstáculo detectado: entrando en modo AVOIDING")
                self.robot_state = STATE_AVOIDING
                if not self.drawer_paused:
                    self.call_service(self.pause_client)
                    self.drawer_paused = True

        elif self.robot_state == STATE_AVOIDING:
            # Ejecutar maniobra de esquiva
            self.obstacle_detected = False  # ignorar nuevas detecciones temporales
            self.get_logger().info("Ejecutando maniobra de esquiva.")
            self.avoid_obstacle()
            # Volver a modo dibujo
            self.robot_state = STATE_DRAWING
            if self.drawer_paused:
                self.call_service(self.resume_client)
                self.drawer_paused = False

        # Publicar el comando (si estamos en AVOIDING, avoid_obstacle ya publica sus propios comandos)
        self.cmd_pub.publish(vel_cmd)

    def avoid_obstacle(self):
        """
        Secuencia de movimientos diseñada para JetBot:
        - pararse
        - girar a la derecha  (90°)
        - avanzar lateralmente (moverse "al lado" del obstáculo)
        - girar izquierda (volver orientación original)
        - avanzar un tramo para rebasar
        - girar izquierda (preparar retorno)
        - avanzar lateralmente (volver a la trayectoria)
        - girar derecha (alinear a la orientación inicial)
        Ajusta duraciones/velocidades según comportamiento real.
        """
        # Parar y asegurar
        self.execute_movement(0.0, 0.0, duration=0.3)
        # Girar 90 grados (dirección escogida según preferencia; aquí giramos +90)
        self.turn(90)
        # Avanzar para salir lateralmente del obstáculo
        self.forward(1.5)
        # Girar -90 para volver paralelos al camino original
        self.turn(-90)
        # Avanzar para rebasar el obstáculo
        self.forward(2.5)
        # Girar -90 para regresar a la línea original
        self.turn(-90)
        # Avanzar lateralmente de nuevo
        self.forward(1.5)
        # Girar 90 para recuperar orientación inicial
        self.turn(90)
        # Pequeña pausa y asegurar parada
        self.execute_movement(0.0, 0.0, duration=0.2)

    def stop(self):
        self.execute_movement(0.0, 0.0, duration=0.2)

    def forward(self, duration, speed=None):
        s = self.linear_speed if speed is None else speed
        self.execute_movement(s, 0.0, duration)

    def turn(self, angle_deg, angular_speed=None):
        w = self.angular_speed if angular_speed is None else angular_speed
        angle_rad = math.radians(angle_deg)
        duration = abs(angle_rad) / w if w != 0 else 0.0
        direction = 1 if angle_rad > 0 else -1
        self.execute_movement(0.0, direction * w, duration)

    def execute_movement(self, linear_speed, angular_speed, duration):
        """
        Publica comandos durante 'duration' segundos y luego para.
        Usa un bucle con sleep breve (compatible con JetBot).
        """
        vel_cmd = Twist()
        vel_cmd.linear.x = linear_speed
        vel_cmd.angular.z = angular_speed

        start_time = time.time()
        while (time.time() - start_time) < duration and rclpy.ok():
            self.cmd_pub.publish(vel_cmd)
            time.sleep(self.move_dt)

        # Parada segura al terminar
        self.cmd_pub.publish(Twist())

    def watchdog_callback(self):
        # Si el LiDAR estuvo activo alguna vez, comprobar tiempo desde última lectura
        if not self.lidar_active:
            return
        now = self.get_clock().now()
        time_diff_sec = (now - self.last_scan_time).nanoseconds / 1e9
        if time_diff_sec > self.lidar_timeout_sec:
            self.get_logger().error("Fallo del LiDAR: no se reciben datos. Forzando parada y pausa.")
            # Forzar parada y pausar el dibujador por seguridad
            self.cmd_pub.publish(Twist())
            if not self.drawer_paused:
                self.call_service(self.pause_client)
                self.drawer_paused = True
            # Mantener estado de obstáculo para evitar reanudar hasta que haya lecturas
            self.obstacle_detected = True

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoiderJetbot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Parada final
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
