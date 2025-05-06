#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
import heapq, math
import numpy as np

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')
        self.map = None
        self.goal = None
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        # Suscripciones
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        # Publicador de Path
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.create_timer(1.0, self.plan_path)

        # Umbral de distancia para detenerse
        self.declare_parameter('threshold_distance', 0.3)
        self.threshold_distance = self.get_parameter('threshold_distance').value

    def dilate_map(self, map_data, width, height, radius=1):
        """
        Aplica una dilatación a los obstáculos en el mapa, creando una burbuja de seguridad.
        Cambia las celdas alrededor de obstáculos (valores >=50) a 100 (ocupadas).
        """
        dilated_map = np.array(map_data).reshape((height, width))
        new_map = np.copy(dilated_map)
        for x in range(width):
            for y in range(height):
                if dilated_map[y, x] >= 50:
                    for dx in range(-radius, radius + 1):
                        for dy in range(-radius, radius + 1):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < width and 0 <= ny < height:
                                if new_map[ny, nx] != -1:  # no marcar como ocupado si es desconocido
                                    new_map[ny, nx] = 100
        return new_map.flatten().tolist()

    def map_cb(self, msg):
        self.map = msg
         # Aplica dilatación en el mapa
        dilated_map = self.dilate_map(self.map.data, self.map.info.width, self.map.info.height, radius=8)
        self.map.data = dilated_map

    def goal_cb(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

    def plan_path(self):
        if not self.map or not self.goal:
            return

        # Obtener pose actual desde TF
        try:
            t = self.tf_buffer.lookup_transform(
                self.map.header.frame_id, 'base_link', rclpy.time.Time())
            start = (t.transform.translation.x,
                     t.transform.translation.y)
        except Exception as e:
            self.get_logger().warn(f'TF no disponible: {e}')
            return

        # Grid indices
        origin = self.map.info.origin.position
        res = self.map.info.resolution
        start_idx = (int((start[0]-origin.x)/res),
                     int((start[1]-origin.y)/res))
        goal_idx  = (int((self.goal[0]-origin.x)/res),
                     int((self.goal[1]-origin.y)/res))
        width = self.map.info.width
        data = self.map.data

        # Verificar si el robot está cerca del objetivo (umbral)
        if self.is_at_goal(start[0], start[1]):
            self.get_logger().info('El robot ya está en el objetivo, no generando nuevo path.')
            return

        # Vecinos válidos
        def valid(nb):
            x,y = nb
            if not (0<=x<width and 0<=y<self.map.info.height): return False
            return data[y*width+x] == 0

        def neighbors(node):
            x,y = node
            for dx,dy in [(-1,0),(1,0),(0,-1),(0,1),
                          (-1,-1),(1,1),(-1,1),(1,-1)]:
                nb=(x+dx,y+dy)
                if valid(nb):
                    yield nb, math.hypot(dx,dy)

        def h(a,b):
            return math.hypot(a[0]-b[0], a[1]-b[1])

        # A*
        open_set = [(h(start_idx, goal_idx),0, start_idx, None)]
        came_from = {}
        cost_so_far = {start_idx:0}

        while open_set:
            f,g,cur,parent = heapq.heappop(open_set)
            if cur in came_from: continue
            came_from[cur] = parent
            if cur==goal_idx: break
            for nb,w in neighbors(cur):
                ng = g + w
                if nb not in cost_so_far or ng<cost_so_far[nb]:
                    cost_so_far[nb]=ng
                    heapq.heappush(open_set,(ng+h(nb,goal_idx),ng,nb,cur))

        # Reconstruir
        path=[]
        node = goal_idx
        while node:
            path.append(node)
            node = came_from.get(node)
        path = path[::-1]

        # Publicar
        path_msg=Path()
        path_msg.header=Header(stamp=self.get_clock().now().to_msg(),
                               frame_id=self.map.header.frame_id)
        for ix,iy in path:
            ps=PoseStamped(header=path_msg.header)
            ps.pose.position.x = ix*res + origin.x
            ps.pose.position.y = iy*res + origin.y
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Path publicada: {len(path_msg.poses)} puntos')
    
    def is_at_goal(self, robot_x, robot_y):
        # Verificar si el robot está lo suficientemente cerca del objetivo
        goal_distance = math.hypot(self.goal[0] - robot_x, self.goal[1] - robot_y)
        return goal_distance < self.threshold_distance

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
