import heapq
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import math # Needed for ceil in inflation calculation

# The 'SetGoal' service is no longer needed if the goal is received via a topic.
# If you still want to offer a service for setting goals in addition to the topic,
# you would keep this import and the service creation.
# For this request, assuming topic is the sole input for goal.
# from your_package_name.srv import SetGoal

# Queda pendiente suscribirnos al servicio

class Node:
    """
    Represents a single cell/state in the grid for pathfinding.
    """
    def __init__(self, x, y, cost, heuristic):
        self.x = x
        self.y = y
        self.cost = cost # g(n): Actual cost from start to this node
        self.heuristic = heuristic # h(n): Estimated cost from this node to goal
        self.estimated_total_cost = cost + heuristic # f(n) = g(n) + h(n)
        self.parent = None # Parent node for path reconstruction

    def __lt__(self, other):
        """
        Comparison method for use with heapq (priority queue).
        Nodes with lower estimated_total_cost are prioritized.
        """
        return self.estimated_total_cost < other.estimated_total_cost

class DStarPlanner:
    """
    Implements a pathfinding algorithm (A* in this case) on an occupancy grid.
    Handles grid conversions, neighbor finding, and path reconstruction.
    """
    def __init__(self, logger, occupied_threshold_prob):
        # The logger from the ROS2 node for printing messages
        self.logger = logger
        self.occupied_threshold_prob = occupied_threshold_prob # Store the threshold

        self.grid_width = 0
        self.grid_height = 0
        self.grid = [] # Stores the inflated occupancy probabilities for planning
        self.resolution = 0.05 # Default cell size: 5 cm (0.05 meters)
        self.origin_x = 0.0
        self.origin_y = 0.0

    def get_logger(self):
        return self.logger

    def set_occupancy(self, x, y, prob):
        """
        Sets the occupancy probability of a specific grid cell.
        This function might be less used directly if map_callback handles bulk grid population.
        """
        if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
            self.grid[x][y] = prob
        # else: pass (ignore out-of-bounds attempts)

    def set_grid_info(self, resolution, width, height, origin_x, origin_y):
        """
        Updates the planner's grid parameters based on received map info.
        Re-initializes the internal grid structure.
        """
        self.resolution = resolution
        self.grid_width = width
        self.grid_height = height
        # Initialize grid with zeros (free space) based on new dimensions
        self.grid = [[0.0 for _ in range(self.grid_height)] for _ in range(self.grid_width)]
        self.origin_x = origin_x
        self.origin_y = origin_y

    def heuristic(self, start, goal):
        """
        Calculates the Manhattan distance heuristic between two grid points.
        This is an admissible heuristic for grid-based A*.
        """
        return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

    def get_neighbors(self, node):
        """
        Returns a list of valid neighboring grid cells for a given node.
        Considers 8-directional movement (orthogonal and diagonal).
        """
        x, y = node.x, node.y
        neighbors = []
        # Define 8 possible movements (dx, dy)
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0), # Orthogonal
                       (1, 1), (1, -1), (-1, 1), (-1, -1)]: # Diagonal
            new_x, new_y = x + dx, y + dy
            # Check if neighbor is within grid boundaries
            if 0 <= new_x < self.grid_width and 0 <= new_y < self.grid_height:
                # Assign different costs for orthogonal vs. diagonal moves
                cost_multiplier = 1.414 if abs(dx) + abs(dy) == 2 else 1.0 # 1.414 approx sqrt(2)
                neighbors.append((new_x, new_y, cost_multiplier))
        return neighbors

    def plan_path(self, start_coord, goal_coord):
        """
        Plans a path from start to goal using the A* algorithm.
        Returns a list of world coordinates representing the path, or None if no path found.
        """
        start_grid = self.world_to_grid(start_coord[0], start_coord[1])
        goal_grid = self.world_to_grid(goal_coord[0], goal_coord[1])

        self.get_logger().info(f"Planning path from grid {start_grid} to grid {goal_grid}")

        # Check if start or goal are in occupied/inflated cells or out of bounds
        if not (0 <= start_grid[0] < self.grid_width and 0 <= start_grid[1] < self.grid_height) or \
           self.grid[start_grid[0]][start_grid[1]] > self.occupied_threshold_prob:
            self.get_logger().warning(f"Start point {start_grid} is in an obstacle or out of bounds.")
            return None
        if not (0 <= goal_grid[0] < self.grid_width and 0 <= goal_grid[1] < self.grid_height) or \
           self.grid[goal_grid[0]][goal_grid[1]] > self.occupied_threshold_prob:
            self.get_logger().warning(f"Goal point {goal_grid} is in an obstacle or out of bounds.")
            return None


        start_node = Node(start_grid[0], start_grid[1], 0, self.heuristic(start_grid, goal_grid))
        goal_node_coords = (goal_grid[0], goal_grid[1])

        # Priority queue for nodes to be evaluated, ordered by estimated_total_cost
        open_set = [start_node]
        # Set to store already evaluated nodes (grid coordinates)
        closed_set = set()
        # Dictionary to store the actual cost (g-score) to a node, for efficient updates
        g_scores = {(start_grid[0], start_grid[1]): 0}


        while open_set:
            current_node = heapq.heappop(open_set)

            # If current node is the goal, reconstruct and return path
            if (current_node.x, current_node.y) == goal_node_coords:
                self.get_logger().info("Path found!")
                return self.reconstruct_path(current_node)

            # If already processed, skip
            if (current_node.x, current_node.y) in closed_set:
                continue

            closed_set.add((current_node.x, current_node.y))

            for neighbor in self.get_neighbors(current_node):
                neighbor_x, neighbor_y, cost_multiplier = neighbor

                if (neighbor_x, neighbor_y) in closed_set:
                    continue

                occupancy_prob = self.grid[neighbor_x][neighbor_y]
                # If the cell is too occupied (above threshold, including inflated obstacles)
                if occupancy_prob > self.occupied_threshold_prob:
                    continue

                # Calculate new cost to reach this neighbor
                cost_to_move = cost_multiplier * self.resolution
                new_cost = current_node.cost + cost_to_move * (1.0 + occupancy_prob)


                # If a better path to this neighbor is found, update it
                # Check if this neighbor is in g_scores and if the new path is cheaper
                if (neighbor_x, neighbor_y) not in g_scores or new_cost < g_scores[(neighbor_x, neighbor_y)]:
                    g_scores[(neighbor_x, neighbor_y)] = new_cost
                    neighbor_heuristic = self.heuristic((neighbor_x, neighbor_y), goal_grid)
                    neighbor_node = Node(neighbor_x, neighbor_y, new_cost, neighbor_heuristic)
                    neighbor_node.parent = current_node
                    heapq.heappush(open_set, neighbor_node)

        self.get_logger().warning("No path found!")
        return None

    def reconstruct_path(self, node):
        """
        Reconstructs the path from the goal node back to the start using parent pointers.
        Returns the path as a list of world coordinates.
        """
        path = []
        current = node
        while current:
            world_x, world_y = self.grid_to_world(current.x, current.y)
            path.append((world_x, world_y))
            current = current.parent
        return path[::-1] # Reverse the path to be from start to goal

    def world_to_grid(self, world_x, world_y):
        """
        Converts world coordinates (meters) to grid cell indices.
        """
        grid_x = int((world_x - self.origin_x) / self.resolution)
        grid_y = int((world_y - self.origin_y) / self.resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        """
        Converts grid cell indices to world coordinates (meters),
        typically to the center of the cell.
        """
        world_x = grid_x * self.resolution + self.origin_x + self.resolution / 2.0
        world_y = grid_y * self.resolution + self.origin_y + self.resolution / 2.0
        return world_x, world_y

class DStarPlannerROS2(Node):
    """
    ROS 2 node for integrating the DStarPlanner (A*).
    Subscribes to map, robot pose, and final destination, then publishes the next desired position.
    """
    def __init__(self):
        super().__init__('dstar_planner_node')
        self.get_logger().info("DStar Planner ROS2 Node initialized.")

        self.grid_map = None # Stores the raw OccupancyGrid message
        self.current_robot_pose = None # Stores the latest robot pose

        # 1. Define a ROS 2 parameter for the occupied threshold
        self.declare_parameter('occupied_threshold_percent', 60) # Default to 60%
        occupied_threshold_percent = self.get_parameter('occupied_threshold_percent').get_parameter_value().integer_value
        occupied_threshold_prob = float(occupied_threshold_percent) / 100.0
        self.get_logger().info(f"Occupied cell threshold set to {occupied_threshold_percent}% ({occupied_threshold_prob:.2f}).")

        # Pass the ROS2 node's logger and the threshold to the planner instance
        self.planner = DStarPlanner(self.get_logger(), occupied_threshold_prob)

        # Publisher for the next target pose (single PoseStamped message)
        self.next_target_publisher = self.create_publisher(PoseStamped, '/next_target_pose', 10)
        self.get_logger().info("Created publisher for /next_target_pose topic.")

        # Subscriber for the OccupancyGrid map
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map', # Standard ROS2 topic for occupancy grid maps
            self.map_callback,
            10
        )
        self.get_logger().info("Subscribed to /map topic.")

        # 2. Subscriber for robot's current pose (start position for planning)
        self.robot_pose_subscription = self.create_subscription(
            PoseStamped,
            '/robot_pose', # Common topic for robot's current pose (e.g., from AMCL or ground truth)
            self.robot_pose_callback,
            10
        )
        self.get_logger().info("Subscribed to /robot_pose topic for current robot position.")


        # 2. Subscriber for the final destination (goal position for planning)
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/final_destination', # Topic where the final goal will be published
            self.final_destination_callback,
            10
        )
        self.get_logger().info("Subscribed to /final_destination topic for goal position.")

        # Removed the 'set_goal' service as goals are now received via topic

    def robot_pose_callback(self, msg):
        """
        Callback to update the robot's current pose.
        """
        self.current_robot_pose = msg
        # self.get_logger().debug(f"Robot pose updated: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")

    def map_callback(self, msg):
        """
        Callback function for when a new OccupancyGrid map is received.
        Processes the map and inflates obstacles based on robot size.
        """
        self.get_logger().info("Received new OccupancyGrid map.")
        self.grid_map = msg
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution # Map's resolution (should be 0.05m as per requirements)
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # Update the planner with the new map information
        self.planner.set_grid_info(resolution, width, height, origin_x, origin_y)

        # --- Obstacle Inflation Logic ---
        robot_diameter_m = 0.30 # Robot size: 30 cm (0.3 meters)
        robot_radius_m = robot_diameter_m / 2.0 # Radius for inflation: 15 cm (0.15 meters)

        # Calculate how many cells outwards an obstacle should be inflated
        inflation_cells = math.ceil(robot_radius_m / resolution)

        self.get_logger().info(f"Inflating obstacles by {inflation_cells} grid cells outwards for robot radius {robot_radius_m:.2f}m.")

        # Create a temporary 2D array from the raw occupancy data for easier access
        raw_map_2d = [[0 for _ in range(height)] for _ in range(width)]
        for i in range(height):
            for j in range(width):
                raw_map_2d[j][i] = msg.data[i * width + j] # Store as (col, row)

        # Create the inflated occupancy grid
        inflated_occupancy_grid = [[0.0 for _ in range(height)] for _ in range(width)]

        # Iterate through every cell in the *new* (inflated) grid
        for grid_y in range(height):
            for grid_x in range(width):
                max_neighbor_cost = 0.0 # Will store the highest cost in the neighborhood

                # Check the neighborhood defined by `inflation_cells` around the current cell (grid_x, grid_y)
                for dy_offset in range(-inflation_cells, inflation_cells + 1):
                    for dx_offset in range(-inflation_cells, inflation_cells + 1):
                        # Calculate the coordinates of the neighbor in the *original* raw map
                        nx, ny = grid_x + dx_offset, grid_y + dy_offset

                        # Check if the neighbor is within the original map bounds
                        if 0 <= nx < width and 0 <= ny < height:
                            # Use Chebyshev distance for rectangular inflation zone.
                            if max(abs(dx_offset), abs(dy_offset)) <= inflation_cells:
                                original_value = raw_map_2d[nx][ny] # Get original occupancy value

                                current_cell_cost_original = 0.0
                                if original_value == -1: # Unknown space
                                    # Treating unknown as 0.0 allows planning through it.
                                    # You might set this to 0.5 for caution, or even 1.0 to block unknowns.
                                    current_cell_cost_original = 0.0
                                elif original_value >= self.planner.occupied_threshold_prob * 100: # Explicitly an obstacle
                                    current_cell_cost_original = 1.0 # Highest cost
                                else: # Free space or low occupancy
                                    current_cell_cost_original = float(original_value) / 100.0

                                # Keep track of the highest cost found in the neighborhood
                                if current_cell_cost_original > max_neighbor_cost:
                                    max_neighbor_cost = current_cell_cost_original

                # Assign the highest cost from its neighborhood to the current cell in the inflated grid.
                inflated_occupancy_grid[grid_x][grid_y] = max_neighbor_cost

        # Set the planner's grid to the newly calculated inflated grid
        self.planner.grid = inflated_occupancy_grid
        self.get_logger().info(f"Map updated and inflated. Grid dimensions: {self.planner.grid_width}x{self.planner.grid_height}, Resolution: {self.planner.resolution}")

    def final_destination_callback(self, msg):
        """
        Callback function for when a new final destination (goal) is received via topic.
        Triggers path planning from the current robot pose to this goal.
        """
        # Ensure map and robot pose are available before planning
        if self.grid_map is None:
            self.get_logger().warn("Map not received yet. Cannot plan path to final destination.")
            return

        if self.current_robot_pose is None:
            self.get_logger().warn("Robot's current pose not received yet. Cannot plan path to final destination.")
            return

        start_coords = (self.current_robot_pose.pose.position.x, self.current_robot_pose.pose.position.y)
        goal_coords = (msg.pose.position.x, msg.pose.position.y)

        self.get_logger().info(f"Received final destination: Goal({goal_coords[0]:.2f}, {goal_coords[1]:.2f}) from topic.")
        self.get_logger().info(f"Using current robot pose as start: Start({start_coords[0]:.2f}, {start_coords[1]:.2f}).")

        path_coords = self.planner.plan_path(start_coords, goal_coords)

        if path_coords:
            # Determine the next desired position
            if len(path_coords) > 1:
                # The next desired position is the second point in the path (path_coords[1])
                next_target_x, next_target_y = path_coords[1]
                self.get_logger().info(f"Path found. Next desired position: ({next_target_x:.2f}, {next_target_y:.2f})")
            else: # Path found, but it's just the start point (start == goal)
                # In this case, the robot is already at the goal, so send the goal itself
                next_target_x, next_target_y = path_coords[0]
                self.get_logger().info(f"Start is goal. Sending goal as next target: ({next_target_x:.2f}, {next_target_y:.2f})")

            # Create and populate the PoseStamped message for the next target
            next_target_pose_msg = PoseStamped()
            next_target_pose_msg.header.frame_id = self.grid_map.header.frame_id # Use the map's frame ID
            next_target_pose_msg.header.stamp = self.get_clock().now().to_msg() # Current ROS time
            next_target_pose_msg.pose.position.x = next_target_x
            next_target_pose_msg.pose.position.y = next_target_y
            next_target_pose_msg.pose.position.z = 0.0 # Assuming 2D planning, z is often 0

            # Publish the next desired position
            self.next_target_publisher.publish(next_target_pose_msg)
            self.get_logger().info("Published next desired position successfully.")
        else:
            self.get_logger().warn("Failed to find a path.")


def main(args=None):
    """
    Main function to initialize and run the ROS 2 node.
    """
    rclpy.init(args=args) # Initialize ROS 2
    dstar_planner_node = DStarPlannerROS2() # Create an instance of the node
    rclpy.spin(dstar_planner_node) # Keep the node alive and processing callbacks
    dstar_planner_node.destroy_node() # Clean up node resources
    rclpy.shutdown() # Shut down ROS 2

if __name__ == '__main__':
    main()
