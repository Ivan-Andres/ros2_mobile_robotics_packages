import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from transforms3d.euler import quat2euler
import math, numpy as np

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.declare_parameter('lookahead_distance', 0.9) #para vida real 0.5m, simulacion 3.0m
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value

        self.declare_parameter('goal_tolerance', 0.3)
        self.goal_tolerance = self.get_parameter('goal_tolerance').get_parameter_value().double_value

        self.declare_parameter('max_speed', 1.5)  # Velocidad máxima 0.5 para real, sim 1.5
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value

        self.declare_parameter('k_p', 0.05)  # Ganancia proporcional 0.5 para real, sim 0.05
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
        
        self.pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            10)

        self.current_pose = None  # Guardará la última pose recibida

        self.current_path = []

        self.timer = self.create_timer(0.1, self.control_loop)

    def path_callback(self, msg):
        self.current_path = msg.poses

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose


    def control_loop(self):
        if not self.current_path:
            return

        if self.current_pose is None:
            return

        robot_x = self.current_pose.position.x
        robot_y = self.current_pose.position.y

        q = self.current_pose.orientation
        _, _, robot_yaw = quat2euler([q.w, q.x, q.y, q.z], axes='sxyz')

        current_pos = np.array([robot_x, robot_y])
        waypoints_list = [(p.pose.position.x, p.pose.position.y) 
                          for p in self.current_path]
        waypoints_xy = np.array(waypoints_list)

        nearest_idx = np.argmin(np.sum((current_pos-waypoints_xy)**2,axis=1))
        pruned_waypoints = waypoints_xy[nearest_idx:] 
        self.current_path = self.current_path[nearest_idx:]

        waypoints_global = pruned_waypoints
        diffs = waypoints_global - current_pos 
        cos_t = math.cos(robot_yaw)
        sin_t = math.sin(robot_yaw)
        R = np.array([[ cos_t, sin_t],
                    [-sin_t, cos_t]])
        robot_frame = diffs @ R.T

        x_r_arr = robot_frame[:, 0] 
        y_r_arr = robot_frame[:, 1]
        dists = np.hypot(x_r_arr, y_r_arr)
        mask_valid = (dists >= self.lookahead_distance) & (x_r_arr > 0)
        valid_indices = np.nonzero(mask_valid)[0]

        if valid_indices.size == 0:
            return

        lookahead_idx = valid_indices[0]
        lookahead_pose = self.current_path[lookahead_idx]
            
        goal_pose = self.current_path[-1].pose
        goal_distance = math.hypot(goal_pose.position.x - robot_x,
                                goal_pose.position.y - robot_y)

        if goal_distance < self.goal_tolerance:
            self.get_logger().info('Objetivo alcanzado. Deteniendo el robot.')
            stop_cmd = Twist()
            self.cmd_vel_publisher.publish(stop_cmd)
            self.current_path = [] 
            return

        dx = lookahead_pose.pose.position.x - robot_x
        dy = lookahead_pose.pose.position.y - robot_y
        self.get_logger().info(f'punto x: {lookahead_pose.pose.position.x:.2f}, y: {lookahead_pose.pose.position.y:.2f}')
        theta = robot_yaw
        self.get_logger().info(f'Robot en x: {robot_x:.2f}, y: {robot_y:.2f}, theta: {theta:.2f}')
        gx = math.cos(theta) * dx + math.sin(theta) * dy
        gy = -math.sin(theta) * dx + math.cos(theta) * dy
        self.get_logger().info(f'diferencia1 x: {dx:.2f}, y: {dy:.2f}')
        self.get_logger().info(f'diferencia2 x: {dx:.2f}, y: {dy:.2f}')

        L = math.sqrt(gx**2 + gy**2)
        self.get_logger().info(f'distancia al punto: {L:.2f}')
        if L == 0:
            alpha = 0.0
        else:
            alpha = (2 * gy) / (L ** 2)
        
        self.get_logger().info(f'angulo de curvatura: {alpha:.2f}')

        wheelbase = 0.26 
        delta = math.atan(wheelbase * alpha)  # 
        self.get_logger().info(f'angulo de stering: {delta:.2f}')

        max_steer = 0.785 # radianes 0.5 para sim, real 0.785
        if delta > max_steer:
            delta = max_steer
        elif delta < -max_steer:
            delta = -max_steer

        normalized_steering = delta / max_steer

        cmd = Twist()
        cmd.linear.x = 0.5
        # cmd.angular.z = delta
        cmd.angular.z = normalized_steering
        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
