import heapq
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from your_package_name.srv import SetGoal

class Node:
    def __init__(self, x, y, cost, heuristic):
        self.x = x
        self.y = y
        self.cost = cost
        self.heuristic = heuristic
        self.estimated_total_cost = cost + heuristic
        self.parent = None

    def __lt__(self, other):
        return self.estimated_total_cost < other.estimated_total_cost

class DStarPlanner:
    def __init__(self, grid_size):
        self.grid_size = grid_size
        self.grid = [[0.0 for _ in range(grid_size)] for _ in range(grid_size)]
        self.resolution = 1.0
        self.origin_x = 0.0
        self.origin_y = 0.0
    def set_occupancy(self, x, y, prob):
        if 0 <= x < self.grid_size and 0 <= y < self.grid_size:
            self.grid[x][y] = prob
        else:
            pass

    def set_grid_info(self, resolution, width, height, origin_x, origin_y):
        self.resolution = resolution
        self.grid_size = width
        self.grid = [[0.0 for _ in range(height)] for _ in range(width)]
        self.origin_x = origin_x
        self.origin_y = origin_y

    def heuristic(self, start, goal):
        return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

    def get_neighbors(self, node):
        x, y = node.x, node.y
        neighbors = []
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0),
                       (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            new_x, new_y = x + dx, y + dy
            if 0 <= new_x < self.grid_size and 0 <= new_y < self.grid_size:
                cost_multiplier = 1.414 if abs(dx) + abs(dy) == 2 else 1.0
                neighbors.append((new_x, new_y, cost_multiplier))
        return neighbors

    def plan_path(self, start_coord, goal_coord):
        start_grid = self.world_to_grid(start_coord[0], start_coord[1])
        goal_grid = self.world_to_grid(goal_coord[0], goal_coord[1])

        self.get_logger().info(f"Planning path from grid {start_grid} to grid {goal_grid}")

        start_node = Node(start_grid[0], start_grid[1], 0, self.heuristic(start_grid, goal_grid))
        goal_node_coords = (goal_grid[0], goal_grid[1])

        open_set = [start_node]
        closed_set = set()

        while open_set:
            current_node = heapq.heappop(open_set)

            if (current_node.x, current_node.y) == goal_node_coords:
                self.get_logger().info("Path found!")
                return self.reconstruct_path(current_node)

            if (current_node.x, current_node.y) in closed_set:
                continue

            closed_set.add((current_node.x, current_node.y))

            for neighbor in self.get_neighbors(current_node):
                neighbor_x, neighbor_y, cost_multiplier = neighbor

                if (neighbor_x, neighbor_y) in closed_set:
                    continue

                occupancy_prob = self.grid[neighbor_x][neighbor_y]
                if occupancy_prob > 0.6:
                    continue

                new_cost = current_node.cost + (occupancy_prob * cost_multiplier * self.resolution)

                is_in_open_set = False
                for existing_node in open_set:
                    if existing_node.x == neighbor_x and existing_node.y == neighbor_y:
                        is_in_open_set = True
                        if new_cost < existing_node.cost:
                            existing_node.cost = new_cost
                            existing_node.parent = current_node
                            heapq.heapify(open_set)
                        break

                if not is_in_open_set:
                    neighbor_heuristic = self.heuristic((neighbor_x, neighbor_y), goal_grid)
                    neighbor_node = Node(neighbor_x, neighbor_y, new_cost, neighbor_heuristic)
                    neighbor_node.parent = current_node
                    heapq.heappush(open_set, neighbor_node)

        self.get_logger().warning("No path found!")
        return None

    def reconstruct_path(self, node):
        path = []
        while node:
            world_x, world_y = self.grid_to_world(node.x, node.y)
            path.append((world_x, world_y))
            node = node.parent
        return path[::-1]

    def world_to_grid(self, world_x, world_y):
        grid_x = int((world_x - self.origin_x) / self.resolution)
        grid_y = int((world_y - self.origin_y) / self.resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        world_x = grid_x * self.resolution + self.origin_x + self.resolution / 2.0
        world_y = grid_y * self.resolution + self.origin_y + self.resolution / 2.0
        return world_x, world_y

class DStarPlannerROS2(Node):
    def __init__(self):
        super().__init__('dstar_planner_node')
        self.get_logger().info("DStar Planner ROS2 Node initialized.")

        self.grid_map = None
        self.current_pose = None

        self.planner = DStarPlanner(1)

        self.path_publisher = self.create_publisher(Path, '/path', 10)

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.get_logger().info("Subscribed to /map topic.")

        self.goal_service = self.create_service(SetGoal, 'set_goal', self.set_goal_callback)
        self.get_logger().info("Created 'set_goal' service.")

    def map_callback(self, msg):
        self.get_logger().info("Received new OccupancyGrid map.")
        self.grid_map = msg
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        self.planner.set_grid_info(resolution, width, height, origin_x, origin_y)

        for i in range(height):
            for j in range(width):
                occupancy_value = msg.data[i * width + j]
                if occupancy_value == -1:
                    self.planner.set_occupancy(j, i, 0.0)
                else:
                    self.planner.set_occupancy(j, i, float(occupancy_value) / 100.0)
        self.get_logger().info(f"Map updated. Grid size: {self.planner.grid_size}x{self.planner.grid_size}, Resolution: {self.planner.resolution}")

    def set_goal_callback(self, request, response):
        if self.grid_map is None:
            self.get_logger().warn("Map not received yet. Cannot plan path.")
            response.success = False
            response.message = "Map not available."
            return response

        start_pose = request.start_pose.pose.position
        goal_pose = request.goal_pose.pose.position

        start_coords = (start_pose.x, start_pose.y)
        goal_coords = (goal_pose.x, goal_pose.y)

        self.get_logger().info(f"Received goal request: Start({start_coords[0]:.2f}, {start_coords[1]:.2f}), Goal({goal_coords[0]:.2f}, {goal_coords[1]:.2f})")

        path_coords = self.planner.plan_path(start_coords, goal_coords)

        if path_coords:
            path_msg = Path()
            path_msg.header.frame_id = self.grid_map.header.frame_id
            path_msg.header.stamp = self.get_clock().now().to_msg()

            for x, y in path_coords:
                pose_stamped = PoseStamped()
                pose_stamped.header = path_msg.header
                pose_stamped.pose.position.x = x
                pose_stamped.pose.position.y = y
                pose_stamped.pose.position.z = 0.0
                path_msg.poses.append(pose_stamped)

            self.path_publisher.publish(path_msg)
            self.get_logger().info(f"Published path with {len(path_msg.poses)} poses.")
            response.success = True
            response.message = "Path planned successfully."
        else:
            self.get_logger().warn("Failed to find a path.")
            response.success = False
            response.message = "No path found."

        return response

def main(args=None):
    rclpy.init(args=args)
    dstar_planner_node = DStarPlannerROS2()
    rclpy.spin(dstar_planner_node)
    dstar_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
