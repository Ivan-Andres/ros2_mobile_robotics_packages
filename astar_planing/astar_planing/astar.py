#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Header
import heapq, math
import numpy as np
from collections import deque

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')
        self.map = None
        self.goal_queue = deque()
        self.executing = False
        self.current_pose = None
        self.current_path = [] 

        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_received = False

        self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos_profile)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.slam_pose_cb, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.declare_parameter('threshold_distance', 1.0)
        self.threshold_distance = self.get_parameter('threshold_distance').value
        self.timer = self.create_timer(0.1, self.timer_cb) 

    def slam_pose_cb(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def timer_cb(self):
        if self.current_path:
            self.publish_path(self.current_path)

    def dilate_map(self, map_data, width, height, radius=1):
        dilated_map = np.array(map_data).reshape((height, width))
        new_map = np.copy(dilated_map)
        for x in range(width):
            for y in range(height):
                if dilated_map[y, x] >= 50:
                    for dx in range(-radius, radius + 1):
                        for dy in range(-radius, radius + 1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < width and 0 <= ny < height:
                                if new_map[ny, nx] != -1:
                                    new_map[ny, nx] = 100
        return new_map.flatten().tolist()

    def map_cb(self, msg):
        self.get_logger().warn('mapa recivido.')
        self.map = msg
        self.map_received = True
        dilated = self.dilate_map(self.map.data, self.map.info.width, self.map.info.height, radius=3) #real 1, sim 15
        self.map.data = dilated
        self.get_logger().warn('mapa leido.')

    def goal_cb(self, msg):
        point = (msg.pose.position.x, msg.pose.position.y)
        self.goal_queue.append(point)
        self.get_logger().info(f'Objetivo añadido a la cola: {point}')

    def clicked_cb(self, msg):
        if not self.executing and self.goal_queue:
            self.executing = True
            self.execute_path_through_all_goals()
            self.get_logger().info(f'ejecutar puntos')

    def execute_path_through_all_goals(self):
        if not self.goal_queue:
            self.get_logger().warn('no hay objetivos.')
            self.executing = False
            return
        
        if not self.map :
            self.get_logger().warn('Mapa no disponible')
            self.executing = False
            return

        if self.current_pose is None:
            self.get_logger().warn('Pose actual no disponible.')
            self.executing = False
            return

        current_pos = (self.current_pose.position.x, self.current_pose.position.y)

        full_path = []
        goals = list(self.goal_queue)
        self.goal_queue.clear()

        for goal in goals:
            segment = self.a_star_plan(current_pos, goal)
            if segment:
                if full_path:
                    segment = segment[1:]
                full_path.extend(segment)
                current_pos = goal
            else:
                self.get_logger().warn(f'Se falló al planear hacia: {goal}')

        if full_path:
            self.current_path = full_path 
        else:
            self.get_logger().warn('No se pudo construir un camino completo.')

        self.executing = False

    def a_star_plan(self, start, goal):
        origin = self.map.info.origin.position
        res = self.map.info.resolution
        width = self.map.info.width
        height = self.map.info.height
        data = self.map.data

        start_idx = (int((start[0] - origin.x) / res), int((start[1] - origin.y) / res))
        goal_idx = (int((goal[0] - origin.x) / res), int((goal[1] - origin.y) / res))

        def valid(nb):
            x, y = nb
            return (0 <= x < width and 0 <= y < height) and data[y * width + x] == 0

        def neighbors(node):
            x, y = node
            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,1),(-1,1),(1,-1)]:
                nb = (x + dx, y + dy)
                if valid(nb):
                    yield nb, math.hypot(dx, dy)

        def h(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        open_set = [(h(start_idx, goal_idx), 0, start_idx, None)]
        came_from = {}
        cost_so_far = {start_idx: 0}

        while open_set:
            f, g, current, parent = heapq.heappop(open_set)
            if current in came_from:
                continue
            came_from[current] = parent
            if current == goal_idx:
                break
            for nb, w in neighbors(current):
                new_cost = g + w
                if nb not in cost_so_far or new_cost < cost_so_far[nb]:
                    cost_so_far[nb] = new_cost
                    heapq.heappush(open_set, (new_cost + h(nb, goal_idx), new_cost, nb, current))

        path = []
        node = goal_idx
        while node:
            path.append(node)
            node = came_from.get(node)
        path.reverse()
        self.get_logger().warn(f'path calculado {path}')

        return [(ix * res + origin.x, iy * res + origin.y) for ix, iy in path] if path else None

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.map.header.frame_id)
        for x, y in path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    while not node.map_received and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
