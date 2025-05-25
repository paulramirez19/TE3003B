#include "odometry/odometry.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <ratio>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace odometry {
namespace {

inline double NormalizeHeading(double angle) {
    if (angle <= -M_PI) { return M_PI + std::fmod(angle, M_PI); }
    if (angle >   M_PI) { return std::fmod(angle, M_PI) - M_PI; }
    return angle;
}

}  // namespace

Odometry::Odometry() : Node("odometry") {
    last_vel_time_ = get_clock()->now();

    declare_parameter<double>("linear_scale_x", 1.0);
    declare_parameter<double>("linear_scale_y", 1.0);
    declare_parameter<bool>("pub_odom_tf", false);

    subscription_ =
            create_subscription<geometry_msgs::msg::Twist>("/vel_raw", 50,
                                                           std::bind(&Odometry::OdometryCallback,
                                                                     this, std::placeholders::_1));
    publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom_raw", 50);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void Odometry::OdometryCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const rclcpp::Time current_time = get_clock()->now();

    const double linear_scale_x = get_parameter("linear_scale_x").as_double();
    const double linear_scale_y = get_parameter("linear_scale_y").as_double();
    const double linear_velocity_x = msg->linear.x * linear_scale_x;
    const double linear_velocity_y = msg->linear.y * linear_scale_y;
    const double angular_velocity_z = msg->angular.z;

    const double vel_dt = (current_time - last_vel_time_).seconds();
    last_vel_time_ = current_time;

    const double delta_heading = angular_velocity_z * vel_dt;
    const double delta_x =
            (linear_velocity_x * std::cos(heading_) - linear_velocity_y * std::sin(heading_)) *
            vel_dt;
    const double delta_y =
            (linear_velocity_x * std::sin(heading_) + linear_velocity_y * std::cos(heading_)) *
            vel_dt;

    x_pos_ += delta_x;
    y_pos_ += delta_y;
    heading_ += delta_heading;
    heading_ = NormalizeHeading(heading_);

    RCLCPP_INFO(get_logger(), "dt: %.6lf x: %.6lf y: %.6lf heading: %.6lf", vel_dt, x_pos_,
                y_pos_, heading_);

    tf2::Quaternion quaternion;
    quaternion.setRPY(/*roll=*/0.0, /*pitch=*/0.0, /*yaw=*/heading_);

    geometry_msgs::msg::Quaternion odom_quaternion;
    odom_quaternion.x = quaternion.x();
    odom_quaternion.y = quaternion.y();
    odom_quaternion.z = quaternion.z();
    odom_quaternion.w = quaternion.w();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    // Position in x, y, and z
    odom.pose.pose.position.x = x_pos_;
    odom.pose.pose.position.y = y_pos_;
    odom.pose.pose.position.z = 0.0;

    // Heading in quaternion
    odom.pose.pose.orientation = odom_quaternion;
    odom.pose.covariance[0] = 0.001;
    odom.pose.covariance[7] = 0.001;
    odom.pose.covariance[35] = 0.001;

    // Linear speed from encoders
    odom.twist.twist.linear.x = linear_velocity_x;
    odom.twist.twist.linear.y = linear_velocity_y;
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;

    // Angular speed from encoders
    odom.twist.twist.angular.z = angular_velocity_z;
    odom.twist.covariance[0] = 0.0001;
    odom.twist.covariance[7] = 0.0001;
    odom.twist.covariance[35] = 0.0001;

    const double linear_velocity = std::sqrt(linear_velocity_x * linear_velocity_x +
                                             linear_velocity_y * linear_velocity_y);
    RCLCPP_INFO(get_logger(), "linear_velocity: %.6lf, angular_velocity: %.6lf", linear_velocity,
                angular_velocity_z);

    publisher_->publish(odom);

    const bool pub_odom_tf = get_parameter("pub_odom_tf").as_bool();
    if (pub_odom_tf) {
        geometry_msgs::msg::TransformStamped t;
        const rclcpp::Time now = get_clock()->now();

        t.header.stamp = now;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_footprint";
        t.transform.translation.x = x_pos_;
        t.transform.translation.y = y_pos_;
        t.transform.translation.z = 0.0;

        t.transform.rotation.x = quaternion.x();
        t.transform.rotation.y = quaternion.y();
        t.transform.rotation.z = quaternion.z();
        t.transform.rotation.w = quaternion.w();

        tf_broadcaster_->sendTransform(t);
    }
}

} // namespace odometry

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<odometry::Odometry>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}