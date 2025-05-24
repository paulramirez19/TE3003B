import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import numpy as np
import math

class OccupancyGridMappingNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_mapping')

        # Parámetros
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('width', 200)
        self.declare_parameter('height', 200)
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)

        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.origin_x = self.get_parameter('origin_x').get_parameter_value().double_value
        self.origin_y = self.get_parameter('origin_y').get_parameter_value().double_value

        # Mapa con log-odds y mapa visual (-1: desconocido, 0: libre, 100: ocupado)
        self.log_odds_map = np.zeros((self.height, self.width), dtype=np.float32)
        self.occupancy_map = np.full((self.height, self.width), -1, dtype=np.int8)

        self.l0 = 0.0
        self.l_occ = 0.85
        self.l_free = -0.4
        self.clamp_min = -2.0
        self.clamp_max = 3.5

        # Pose inicial del robot
        self.robot_pose = Pose()

        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)
        self.timer = self.create_timer(1.0, self.publish_map)

    def odom_callback(self, msg: Odometry):
        self.robot_pose = msg.pose.pose


    def world_to_map(self, x, y):
        mx = int((x - self.origin_x) / self.resolution)
        my = int((y - self.origin_y) / self.resolution)
        return mx, my

    def bresenham(self, x0, y0, x1, y1):
        #Algoritmo de Bresenham para marcar espacio libre """
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = -1 if x0 > x1 else 1
        sy = -1 if y0 > y1 else 1

        if dx > dy:
            err = dx / 2.0
            while x != x1:
                points.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1:
                points.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        points.append((x1, y1))
        return points

    def update_cell(self, mx, my, logodds):
        if 0 <= mx < self.width and 0 <= my < self.height:
            self.log_odds_map[my, mx] += logodds
            self.log_odds_map[my, mx] = np.clip(self.log_odds_map[my, mx], self.clamp_min, self.clamp_max)
            l = self.log_odds_map[my, mx]
            if l > 0.6:
                self.occupancy_map[my, mx] = 100
            elif l < -0.6:
                self.occupancy_map[my, mx] = 0
            else:
                self.occupancy_map[my, mx] = -1

    def scan_callback(self, msg: LaserScan):
        angle = msg.angle_min
        # Pose del robot
        rx = self.robot_pose.position.x
        ry = self.robot_pose.position.y
        _, _, yaw = self.get_yaw_from_quaternion(self.robot_pose.orientation)
        for r in msg.ranges:
            if np.isinf(r) or np.isnan(r):
                angle += msg.angle_increment
                continue

            # Punto final del rayo en coordenadas del mundo
            global_angle = yaw + angle
            x_end = rx + r * math.cos(global_angle)
            y_end = ry + r * math.sin(global_angle)

            mx0, my0 = self.world_to_map(rx, ry)
            mx1, my1 = self.world_to_map(x_end, y_end)

            # Marcar espacio libre
            free_cells = self.bresenham(mx0, my0, mx1, my1)
            for (fx, fy) in free_cells[:-1]:  # excluye el último
                self.update_cell(fx, fy, self.l_free)

            # Última celda (ocupada)
            self.update_cell(mx1, my1, self.l_occ)

            angle += msg.angle_increment

    def get_yaw_from_quaternion(self, q):
        #Devuelve (roll, pitch, yaw) desde un quaternion 
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(sinp) if abs(sinp) <= 1 else math.copysign(math.pi / 2, sinp)

        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        return roll, pitch, yaw

    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = 'map'

        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.orientation.w = 1.0

        grid.data = self.occupancy_map.flatten().tolist()
        self.map_pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridMappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
