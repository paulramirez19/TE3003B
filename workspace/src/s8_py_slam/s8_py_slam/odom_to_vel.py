import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class OdomToVelRawPublisher(Node):

    def __init__(self):
        super().__init__('odom_to_vel_raw_publisher')
        # Subscribe to the /odom topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # Create a publisher for the /vel_raw topic of type Twist
        self.vel_raw_publisher = self.create_publisher(Twist, 'vel_raw', 10)
        self.get_logger().info('Odom to /vel_raw republisher node started.')

    def odom_callback(self, msg: Odometry):
        # Extract the Twist message from the Odometry message
        vel_raw_msg = msg.twist.twist
        # Publish it to the /vel_raw topic
        self.vel_raw_publisher.publish(vel_raw_msg)
        #self.get_logger().info(f'Republishing /vel_raw: Linear.x={vel_raw_msg.linear.x:.3f}, Angular.z={vel_raw_msg.angular.z:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = OdomToVelRawPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

