import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion

class FilterTuning(Node):
    def __init__(self):
        super().__init__('filter_tuning')
        # Subscriptions
        self.odom_raw_subs = self.create_subscription(Odometry, '/odom_raw', self.odom_raw_callback, 10)
        self.pose_subs = self.create_subscription(PoseStamped, '/pose_stamped', self.pose_stamped_callback, 10)
        self.ctrl_subs = self.create_subscription(Twist, '/cmd_vel', self.control_callback, 10)
        self.odom_fil_sub = self.create_subscription(Odometry, '/odom_filtered', self.odom_filtered_callback, 10)
        # File opening
        self.path: str = ''
        self.odom_raw_file = open(self.path + 'odom_raw.csv', 'w')
        self.pose_file = open(self.path + 'pose_stamped.csv', 'w')
        self.ctrl_file = open(self.path + 'cmd_vel.csv', 'w')
        # self.odom_fil_file = open(self.path + 'odom_filtered.csv', 'w')

    def get_yaw_from_quaternion(self, orientation):
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = euler_from_quaternion(q)
        return yaw

    def odom_raw_callback(self, msg):
        self.odom_raw_file.write(
            f'{msg.header.stamp},{msg.pose.pose.position.x},{msg.pose.pose.position.y},{self.get_yaw_from_quaternion(msg.pose.pose.orientation)}')

    def pose_stamped_callback(self, msg):
        self.pose_file.write(
            f'{msg.header.stamp},{msg.pose.position.x},{msg.pose.position.y},{self.get_yaw_from_quaternion(msg.pose.orientation)}')

    def control_callback(self, msg):
        x_dot = msg.linear.x
        y_dot = msg.linear.y
        linear_velocity = math.sqrt(x_dot * x_dot + y_dot * y_dot)
        self.ctrl_file.write(f'{linear_velocity},{msg.angular.z}')

    def odom_filtered_callback(self, msg):
        # self.odom_fil_file.write(
        #     f'{msg.pose.pose.position.x},{msg.pose.pose.position.y},{self.get_yaw_from_quaternion(msg.pose.pose.orientation)},' +
        #     f'{msg.pose.covariance[0]},{msg.pose.covariance[7]},{msg.pose.covariance[35]}')
        None

def main(args=None):
    rclpy.init(args=args)
    filter_tuning = FilterTuning()
    rclpy.spin(filter_tuning)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
