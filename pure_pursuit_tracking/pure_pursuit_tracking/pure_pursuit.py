import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from transforms3d.euler import quat2euler
import math

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.declare_parameter('lookahead_distance', 0.5)
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value

        self.declare_parameter('goal_tolerance', 0.3)
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value

        self.declare_parameter('max_speed', 0.6)  # Velocidad máxima
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value

        self.declare_parameter('k_p', 0.05)  # Ganancia proporcional
        self.k_p = self.get_parameter('k_p').get_parameter_value().double_value

        self.path_subscriber = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10)

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel_ctrl',
            10)

        self.current_path = []

        # Inicializar tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_loop)

    def path_callback(self, msg):
        self.current_path = msg.poses

    def control_loop(self):
        if not self.current_path:
            return

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'No se pudo obtener la transformación: {e}')
            return

        # Obtener la posición actual del robot
        robot_x = trans.transform.translation.x
        robot_y = trans.transform.translation.y

        # Convertir la orientación del cuaternión a yaw
        q = trans.transform.rotation
        # transforms3d espera [w, x, y, z]
        roll, pitch, robot_yaw = quat2euler([q.w, q.x, q.y, q.z], axes='sxyz')

        # Encontrar el punto de seguimiento (lookahead point)
        lookahead_point = self.find_lookahead_point(robot_x, robot_y)

        if lookahead_point is None:
            return
        
        # Verificar si se alcanzó el objetivo
        goal_pose = self.current_path[-1].pose
        goal_distance = math.hypot(goal_pose.position.x - robot_x,
                                goal_pose.position.y - robot_y)

        if goal_distance < self.goal_tolerance:
            self.get_logger().info('Objetivo alcanzado. Deteniendo el robot.')
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            self.current_path = []  # Vaciar ruta para detener el control
            return

        # Calcular el ángulo hacia el punto de seguimiento
        dx = lookahead_point.pose.position.x - robot_x
        dy = lookahead_point.pose.position.y - robot_y
        theta = robot_yaw
        gx = math.cos(theta) * dx + math.sin(theta) * dy
        gy = -math.sin(theta) * dx + math.cos(theta) * dy

        # Calcular curvatura (alpha)
        L = math.sqrt(gx**2 + gy**2)
        if L == 0:
            alpha = 0.0
        else:
            alpha = (2 * gy) / (L ** 2)

        # Convertir curvatura a ángulo de dirección (Ackermann)
        wheelbase = 0.26  # metros (ajusta según tu robot)
        delta = math.atan(wheelbase * alpha)  # radianes

        # Limitar entre [-max_steer, +max_steer]
        max_steer = 0.6458  # radianes
        steering_angle = max(-max_steer, min(max_steer, delta))

        # Calcular la velocidad proporcional
        robot_to_goal_distance = math.hypot(goal_pose.position.x - robot_x,
                                            goal_pose.position.y - robot_y)
        
        # Velocidad proporcional (ajustar según el valor de k_p)
        linear_velocity = min(self.k_p * robot_to_goal_distance, self.max_speed)


        # Comando de movimiento
        cmd = Twist()
        cmd.linear.x = linear_velocity
        cmd.angular.z = steering_angle
        self.cmd_vel_publisher.publish(cmd)

    def find_lookahead_point(self, robot_x, robot_y):
        for pose in self.current_path:
            dx = pose.pose.position.x - robot_x
            dy = pose.pose.position.y - robot_y
            distance = math.hypot(dx, dy)
            if distance >= self.lookahead_distance:
                return pose
        return None

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
