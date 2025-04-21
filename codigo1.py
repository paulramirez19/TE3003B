import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import numpy as np
import math

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.goal = np.array([0.0, 0.0])
        self.position = np.array([0.0, 0.0])
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.vel_subscription = self.create_subscription(
            Twist,
            'vel_raw',
            self.velocity_callback,
            10
        )
        self.goal_subscription = self.create_subscription(
            Point,
            'goal_position',
            self.goal_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        u = np.array([0.0, 0.0])
        dt = 0.0

    def goal_callback(self, msg: Point):
        self.goal = np.array([msg.x, msg.y])
        self.get_logger().info(f'New goal received: [{self.goal[0]:.2f}, {self.goal[1]:.2f}]')

    def velocity_callback(self, msg: Twist):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9 
        self.last_time = current_time

        v = msg.linear.x
        omega = msg.angular.z

        self.position[0] += v * math.cos(self.theta) * dt
        self.position[1] += v * math.sin(self.theta) * dt
        self.theta += omega * dt

        h = 0.5
        error = self.goal - self.position

        D = np.array([
            [math.cos(self.theta), -h * math.sin(self.theta)],
            [math.sin(self.theta),  h * math.cos(self.theta)]
        ])

        if np.linalg.det(D) != 0:
            K = np.array([[0.5, 0.0], [0.0, 1]])
            u = np.linalg.inv(D).dot(-K.dot(-error))

        twist_msg = Twist()
        twist_msg.linear.x = u[0]
        twist_msg.angular.z = u[1]
        self.publisher.publish(twist_msg)

        self.get_logger().info(
            f"Estimated Position: [{self.position[0]:.2f}, {self.position[1]:.2f}], "
            f"Error: [{error[0]:.2f}, {error[1]:.2f}], "
            f"Control: [{u[0]:.2f}, {u[1]:.2f}]"
        )

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()