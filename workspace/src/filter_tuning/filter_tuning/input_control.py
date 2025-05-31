import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class InputControl(Node):
    def __init__(self):
        super().__init__('input_control')
        self.input_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer_frequency = 30
        self.timer = self.create_timer(1.0 / self.timer_frequency, self.input_control_callback)
        self.declare_parameter('x_linear', 0.0)
        self.declare_parameter('y_linear', 0.0)
        self.declare_parameter('angular_velocity', 0.0)
        self.declare_parameter('timeout', 0.0)
        self.begin = self.get_clock().now()
    
    def input_control_callback(self):
        timeout = self.get_parameter('timeout').get_parameter_value().double_value
        duration = self.get_clock().now() - self.begin
        duration = float(duration.nanoseconds) / 1e9
        if duration <= timeout:
            x_linear = self.get_parameter('x_linear').get_parameter_value().double_value
            y_linear = self.get_parameter('y_linear').get_parameter_value().double_value
            angular_velocity = self.get_parameter('angular_velocity').get_parameter_value().double_value
            msg = Twist()
            msg.linear.x = x_linear
            msg.linear.y = y_linear
            msg.angular.z = angular_velocity
            self.get_logger().info(f'duration: {duration}, x_linear: {x_linear}, y_linear: {y_linear}, angular: {angular_velocity}')
            self.input_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    input_control = InputControl()
    rclpy.spin(input_control)
    input_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
