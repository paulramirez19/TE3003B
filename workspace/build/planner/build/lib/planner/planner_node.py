import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Pose2D, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path, OccupancyGrid # OccupancyGrid is used for internal representation
from ament_index_python.packages import get_package_share_directory

import heapq
import math
import os
import yaml
from PIL import Image # Make sure to 'pip install Pillow'

class CustomAstarNode(Node):
    def __init__(self):
        super().__init__('custom_astar_planner_node')

        # --- Parameters ---
        self.declare_parameter('goal_tolerance_xy', 0.5) # How close robot needs to be to goal to consider it reached (meters)
        self.declare_parameter('planning_frequency', 1.0) # Hz, how often to trigger planning
        self.declare_parameter('obstacle_threshold', 80) # OccupancyGrid value > this is an obstacle (0-100). -1 (unknown) is also an obstacle.
        self.declare_parameter('map_yaml_path', 'default_map_path/warehouse_map.yaml') # Placeholder, will be set by launch file

        self.goal_tolerance_xy = self.get_parameter('goal_tolerance_xy').get_parameter_value().double_value
        self.planning_frequency = self.get_parameter('planning_frequency').get_parameter_value().double_value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().integer_value
        
        map_yaml_path = self.get_parameter('map_yaml_path').get_parameter_value().string_value
        
        # --- Define map_frame early (FIXED: Moved to top of init) ---
        self.map_frame = 'map' # Standard map frame, defined BEFORE map loading

        # --- Map Loading ---
        self.occupancy_grid_msg = None # Stores a nav_msgs/OccupancyGrid message internally
        self.map_info = { # Internal dictionary for map properties and data
            'resolution': None, 'origin_x': None, 'origin_y': None,
            'width': None, 'height': None, 'data': None # 'data' is the 1D occupancy array
        }
        self.load_map_from_file(map_yaml_path) # Now self.map_frame exists when this is called

        if self.occupancy_grid_msg is None:
            self.get_logger().error("Failed to load map from file. Planner cannot operate.")
            return 

        # --- Robot Actual Position Subscription (now Pose2D directly in map frame) ---
        self.robot_pose_sub = self.create_subscription(
            Pose2D,
            '/robot_pose_2d', # New topic for robot's 2D pose
            self.robot_pose_2d_callback,
            10
        )
        self.current_robot_pose_map_frame = None # Stores PoseStamped in map frame (for internal consistency)

        # --- Goal Position Subscription ---
        self.goal_sub = self.create_subscription(
            Pose2D,
            '/goal_position',
            self.goal_position_callback,
            10
        )
        self.goal_pose_map_frame = None # Stores PoseStamped in map frame

        # --- Path Publishers ---
        self.global_path_pub = self.create_publisher(Path, '/global_path', 10)
        self.desired_position_pub = self.create_publisher(Pose2D, '/desired_position', 10) # Changed to Pose2D

        self.current_path_points_world = None # List of (x, y) tuples in world coordinates

        # --- Planning Timer ---
        self.planning_timer = self.create_timer(1.0 / self.planning_frequency, self.planning_timer_callback)

        self.get_logger().info('Custom A* Planner Node initialized. Map loaded from file. Waiting for robot pose and goal...')

    # --- Map Loading Function (no changes) ---
    def load_map_from_file(self, yaml_path):
        self.get_logger().info(f"Attempting to load map from YAML: {yaml_path}")
        try:
            with open(yaml_path, 'r') as file:
                map_data = yaml.safe_load(file)

            image_filename = map_data['image']
            resolution = map_data['resolution']
            origin = map_data['origin']
            occupied_thresh = map_data['occupied_thresh']
            free_thresh = map_data['free_thresh']
            negate = map_data['negate']

            # Construct full path to PGM image (relative to YAML file)
            pgm_path = os.path.join(os.path.dirname(yaml_path), image_filename)
            self.get_logger().info(f"Loading PGM image: {pgm_path}")

            img = Image.open(pgm_path).convert('L') # Open as grayscale
            img_data = list(img.getdata()) # Get pixel values (0-255)

            width, height = img.size

            # Create OccupancyGrid message (for internal consistency)
            map_msg = OccupancyGrid()
            map_msg.header.stamp = self.get_clock().now().to_msg()
            map_msg.header.frame_id = self.map_frame
            map_msg.info.resolution = float(resolution)
            map_msg.info.width = width
            map_msg.info.height = height
            map_msg.info.origin.position.x = float(origin[0])
            map_msg.info.origin.position.y = float(origin[1])
            map_msg.info.origin.position.z = float(origin[2])
            map_msg.info.origin.orientation.w = 1.0 # Assuming no rotation of map origin for 2D planning

            # Convert PGM data to OccupancyGrid format (-1 to 100)
            # PGM values are 0-255. Map values: 0 (free), 100 (occupied), -1 (unknown)
            map_data_list = []
            # Map data is typically row-major, bottom-to-top. PGM is often top-to-bottom.
            # We reverse rows and iterate by column.
            for y in range(height -1, -1, -1): # Iterate rows from bottom to top
                for x in range(width):
                    pixel_value = img_data[y * width + x] # Access pixel in original image order
                    
                    if negate == 1:
                        pixel_value = 255 - pixel_value # Invert if negate is 1

                    if pixel_value > (occupied_thresh * 255):
                        map_data_list.append(100) # Occupied
                    elif pixel_value < (free_thresh * 255):
                        map_data_list.append(0) # Free
                    else:
                        map_data_list.append(-1) # Unknown or inconclusive

            map_msg.data = map_data_list

            self.occupancy_grid_msg = map_msg
            self.map_info['resolution'] = map_msg.info.resolution
            self.map_info['origin_x'] = map_msg.info.origin.position.x
            self.map_info['origin_y'] = map_msg.info.origin.position.y
            self.map_info['width'] = map_msg.info.width
            self.map_info['height'] = map_msg.info.height
            self.map_info['data'] = map_msg.data # This is the 1D array of occupancy values
            
            self.get_logger().info(f'Map loaded: {width}x{height} at {resolution:.3f} m/cell from {image_filename}.')

        except FileNotFoundError:
            self.get_logger().error(f"Map file not found: {yaml_path} or associated PGM. Check path and file existence.")
            self.occupancy_grid_msg = None
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing map YAML file: {e}")
            self.occupancy_grid_msg = None
        except Exception as e:
            self.get_logger().error(f"Unexpected error loading map: {e}")
            self.occupancy_grid_msg = None

    # --- Callbacks ---

    def robot_pose_2d_callback(self, msg: Pose2D):
        # We assume the Pose2D message is already in the map frame.
        robot_pose_stamped = PoseStamped()
        robot_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        robot_pose_stamped.header.frame_id = self.map_frame # Explicitly set frame to map
        robot_pose_stamped.pose.position.x = msg.x
        robot_pose_stamped.pose.position.y = msg.y
        robot_pose_stamped.pose.position.z = 0.0 # Assuming 2D
        robot_pose_stamped.pose.orientation = self._quaternion_from_yaw(msg.theta)
        
        self.current_robot_pose_map_frame = robot_pose_stamped
        # self.get_logger().debug(f"Robot pose in map: x={self.current_robot_pose_map_frame.pose.position.x:.2f}, y={self.current_robot_pose_map_frame.pose.position.y:.2f}")


    def goal_position_callback(self, msg: Pose2D):
        # Convert Pose2D to PoseStamped for consistency with map frame operations
        goal_pose_stamped = PoseStamped()
        goal_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        goal_pose_stamped.header.frame_id = self.map_frame # Assume goals are published in map frame
        goal_pose_stamped.pose.position.x = msg.x
        goal_pose_stamped.pose.position.y = msg.y
        # Convert Pose2D theta to Quaternion (yaw only)
        goal_pose_stamped.pose.orientation = self._quaternion_from_yaw(msg.theta)
        
        self.goal_pose_map_frame = goal_pose_stamped
        
        self.get_logger().info(f'Received new goal: X={msg.x:.2f}, Y={msg.y:.2f}, Theta={math.degrees(msg.theta):.2f} deg.')
        self.current_path_points_world = None # Invalidate current path to trigger replan

    # --- Planning Logic (no changes) ---

    def planning_timer_callback(self):
        # Check if we have enough info to plan (map, robot pose, and a goal)
        if not all([self.occupancy_grid_msg, self.current_robot_pose_map_frame, self.goal_pose_map_frame]):
            return

        # Check if robot is already at the goal
        dist_to_goal = math.sqrt(
            (self.current_robot_pose_map_frame.pose.position.x - self.goal_pose_map_frame.pose.position.x)**2 +
            (self.current_robot_pose_map_frame.pose.position.y - self.goal_pose_map_frame.pose.position.y)**2
        )

        if dist_to_goal < self.goal_tolerance_xy:
            if self.current_path_points_world is not None: # Check if a path was previously active
                self.get_logger().info("Robot is within goal tolerance. Path cleared and goal achieved.")
            self.current_path_points_world = None # Clear path once goal is reached
            self.goal_pose_map_frame = None # Clear goal to prevent continuous replanning to same spot
            self._publish_empty_path()
            self._publish_empty_desired_position()
            return

        # If no current path or path is considered invalid (e.g., robot deviated too much), try to plan
        # For simplicity, we replan if current_path_points_world is None (new goal or failed plan)
        if self.current_path_points_world is None:
            self._plan_path()
        
        # Publish path and desired position if a valid path exists
        if self.current_path_points_world:
            self._publish_global_path()
            self._publish_desired_position()
        else:
            # If planning failed, ensure no old path/desired position is published
            self._publish_empty_path()
            self._publish_empty_desired_position()


    def _plan_path(self):
        self.get_logger().info('Attempting to plan new path...')
        
        start_point_world = self.current_robot_pose_map_frame.pose.position
        end_point_world = self.goal_pose_map_frame.pose.position

        # Call the A* algorithm
        path_points_world = self._astar_algorithm(
            start_point_world,
            end_point_world
        )

        if path_points_world:
            self.get_logger().info(f'Path found with {len(path_points_world)} points.')
            self.current_path_points_world = path_points_world
        else:
            self.get_logger().warn('No path found!')
            self.current_path_points_world = None # Clear path if planning failed

    # --- Publishers ---

    def _publish_global_path(self):
        if not self.current_path_points_world:
            self._publish_empty_path()
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame
        for x, y in self.current_path_points_world:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0 # Default orientation (or use path segment to derive it)
            path_msg.poses.append(pose)
        self.global_path_pub.publish(path_msg)


    def _publish_empty_path(self):
        empty_path_msg = Path()
        empty_path_msg.header.stamp = self.get_clock().now().to_msg()
        empty_path_msg.header.frame_id = self.map_frame
        self.global_path_pub.publish(empty_path_msg)


    def _publish_desired_position(self):
        # Ensure there's a path with at least 2 points (current pose + next desired point)
        if not self.current_path_points_world or len(self.current_path_points_world) < 2:
            self._publish_empty_desired_position()
            return
        
        # The "desired position" is the next point on the path after the robot's current position.
        # current_path_points_world[0] is typically the robot's current pose.
        # So we take index 1 as the first point the robot should move towards.
        next_point_x, next_point_y = self.current_path_points_world[1]

        desired_pose_msg = Pose2D() # Changed to Pose2D
        desired_pose_msg.x = next_point_x
        desired_pose_msg.y = next_point_y
        
        # Estimate orientation towards the next point based on the current robot pose and the next point
        robot_x = self.current_robot_pose_map_frame.pose.position.x
        robot_y = self.current_robot_pose_map_frame.pose.position.y
        
        dx = next_point_x - robot_x
        dy = next_point_y - robot_y
        theta = math.atan2(dy, dx) # Yaw angle to point from robot to next path point
        
        desired_pose_msg.theta = theta
        
        self.desired_position_pub.publish(desired_pose_msg)


    def _publish_empty_desired_position(self):
        empty_pose_msg = Pose2D() # Changed to Pose2D
        empty_pose_msg.x = 0.0
        empty_pose_msg.y = 0.0
        empty_pose_msg.theta = 0.0
        self.desired_position_pub.publish(empty_pose_msg)

    # --- A* Algorithm Implementation ---

    class AstarGridNode:
        def __init__(self, position, parent=None):
            self.position = position
            self.parent = parent
            self.g = 0
            self.h = 0
            self.f = 0

        def __lt__(self, other):
            return self.f < other.f
        
        def __eq__(self, other):
            return self.position == other.position
        
        def __hash__(self):
            return hash(self.position)

    def _world_to_map(self, world_x, world_y):
        map_x = int((world_x - self.map_info['origin_x']) / self.map_info['resolution'])
        map_y = int((world_y - self.map_info['origin_y']) / self.map_info['resolution'])
        return map_x, map_y

    def _map_to_world(self, map_x, map_y):
        world_x = map_x * self.map_info['resolution'] + self.map_info['origin_x'] + self.map_info['resolution'] / 2
        world_y = map_y * self.map_info['resolution'] + self.map_info['origin_y'] + self.map_info['resolution'] / 2
        return world_x, world_y

    def _is_valid_and_free(self, map_x, map_y):
        """Checks if a cell is within map bounds and is considered free/navigable."""
        if not (0 <= map_x < self.map_info['width'] and 0 <= map_y < self.map_info['height']):
            return False # Correctly returns False if out of bounds

        index = map_y * self.map_info['width'] + map_x
        occupancy_value = self.map_info['data'][index]
        
        return occupancy_value < self.obstacle_threshold and occupancy_value != -1

    def _astar_algorithm(self, start_point_world: Point, end_point_world: Point):
        if not self.occupancy_grid_msg:
            self.get_logger().error("Map data is not available for A* planning (internal error - map not loaded).")
            return None

        start_x_map, start_y_map = self._world_to_map(start_point_world.x, start_point_world.y)
        end_x_map, end_y_map = self._world_to_map(end_point_world.x, end_point_world.y)

        # --- IMPORTANT CORRECTION FOR INDEX ERROR ---
        # 1. Check if start position is within map bounds first
        is_start_in_bounds = (0 <= start_x_map < self.map_info['width'] and 0 <= start_y_map < self.map_info['height'])
        if not is_start_in_bounds:
            self.get_logger().warn(f"A* Start position in map ({start_x_map},{start_y_map}) is OUT OF MAP BOUNDS. Cannot plan.")
            return None
        
        # 2. Now that we know it's in bounds, check if it's free/valid
        if not self._is_valid_and_free(start_x_map, start_y_map):
            # Only if in bounds, then we can safely get the occupancy value for the warning
            occupancy_val = self.map_info['data'][start_y_map * self.map_info['width'] + start_x_map]
            self.get_logger().warn(f"A* Start position in map ({start_x_map},{start_y_map}) is occupied (occupancy: {occupancy_val}). Cannot plan.")
            return None

        # 3. Repeat for end position
        is_end_in_bounds = (0 <= end_x_map < self.map_info['width'] and 0 <= end_y_map < self.map_info['height'])
        if not is_end_in_bounds:
            self.get_logger().warn(f"A* Goal position in map ({end_x_map},{end_y_map}) is OUT OF MAP BOUNDS. Cannot plan.")
            return None

        # 4. Now that we know it's in bounds, check if it's free/valid
        if not self._is_valid_and_free(end_x_map, end_y_map):
            # Only if in bounds, then we can safely get the occupancy value for the warning
            occupancy_val = self.map_info['data'][end_y_map * self.map_info['width'] + end_x_map]
            self.get_logger().warn(f"A* Goal position in map ({end_x_map},{end_y_map}) is invalid or occupied (occupancy: {occupancy_val}). Cannot plan.")
            return None
        # --- END OF CORRECTION ---

        start_node = self.AstarGridNode((start_x_map, start_y_map))
        end_node = self.AstarGridNode((end_x_map, end_y_map))

        open_list = []
        heapq.heappush(open_list, start_node)

        came_from = {}
        g_score = {start_node.position: 0}
        f_score = {start_node.position: self._heuristic(start_node.position, end_node.position)}

        movements = [
            (0, 1, 1), (0, -1, 1), (1, 0, 1), (-1, 0, 1), # Cardinal moves
            (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)) # Diagonal moves
        ]

        while open_list:
            current_node = heapq.heappop(open_list)

            if current_node.position == end_node.position:
                path = []
                current = current_node
                while current is not None:
                    world_x, world_y = self._map_to_world(current.position[0], current.position[1])
                    path.append((world_x, world_y))
                    current = current.parent
                return path[::-1]

            for dx, dy, cost_to_neighbor in movements:
                neighbor_pos = (current_node.position[0] + dx, current_node.position[1] + dy)

                if not self._is_valid_and_free(neighbor_pos[0], neighbor_pos[1]):
                    continue

                tentative_g_score = g_score.get(current_node.position, float('inf')) + cost_to_neighbor

                if tentative_g_score < g_score.get(neighbor_pos, float('inf')):
                    came_from[neighbor_pos] = current_node.position
                    g_score[neighbor_pos] = tentative_g_score
                    f_score[neighbor_pos] = tentative_g_score + self._heuristic(neighbor_pos, end_node.position)
                    
                    neighbor_node = self.AstarGridNode(neighbor_pos, current_node)
                    neighbor_node.g = g_score[neighbor_pos]
                    neighbor_node.h = self._heuristic(neighbor_pos, end_node.position)
                    neighbor_node.f = f_score[neighbor_pos]

                    heapq.heappush(open_list, neighbor_node)

        return None # No path found

    def _quaternion_from_yaw(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    custom_astar_node = CustomAstarNode()
    rclpy.spin(custom_astar_node)
    custom_astar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
