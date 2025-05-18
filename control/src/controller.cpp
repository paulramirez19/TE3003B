#include "control/controller.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/utils.hpp"

namespace control {
namespace {} // namespace

Controller::Controller() : Node("controller") {
    odom_sub_ =
            create_subscription<nav_msgs::msg::Odometry>("/odom_filtered", 10,
                                                         std::bind(&Controller::OdometryCallback,
                                                                   this, std::placeholders::_1));
    path_sub_ = create_subscription<
            nav_msgs::msg::Odometry>("/desired_position", 10,
                                     std::bind(&Controller::DesiredPositionCallback, this,
                                               std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds{30},
                               std::bind(&Controller::ControllerCallback, this));
    ctrl_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    declare_parameter<double>("distance_h", 1.0);
    declare_parameter<double>("x_gain", 1.0);
    declare_parameter<double>("y_gain", 1.0);
    declare_parameter<double>("theta_gain", 1.0);
}

void Controller::OdometryCallback(const nav_msgs::msg::Odometry& odom) {
    odom_ = odom;
}

void Controller::DesiredPositionCallback(const nav_msgs::msg::Odometry& path) {
    path_ = path;
}

void Controller::ControllerCallback() {
    const double h = get_parameter("distance_h").as_double();
    const tf2::Quaternion quat(odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y,
                               odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w);
    const double theta = tf2::getYaw<tf2::Quaternion>(quat);
    Eigen::Matrix<double, 2, 1> u;
    switch (controller_type_) {
        case ControllerType::kPosition: {
            // clang-format off
            const Eigen::Matrix<double, 2, 2> D{{std::cos(theta), -h * std::sin(theta)},
                                                {std::sin(theta),  h * std::cos(theta)}};
            // clang-format on
            const double x = odom_.pose.pose.position.x;
            const double y = odom_.pose.pose.position.y;
            const double x_desired = path_.pose.pose.position.x;
            const double y_desired = path_.pose.pose.position.y;

            const double x_dot_desired = path_.twist.twist.linear.x;
            const double y_dot_desired = path_.twist.twist.linear.y;

            const Eigen::Matrix<double, 2, 1> error{{x_desired - x}, {y_desired - y}};
            const Eigen::Matrix<double, 2, 1> q_dot{{x_dot_desired}, {y_dot_desired}};

            const double k_x = get_parameter("x_gain").as_double();
            const double k_y = get_parameter("y_gain").as_double();
            const Eigen::Matrix<double, 2, 2> K_p{{k_x, 0.0}, {0.0, k_y}};

            u = D.inverse() * (q_dot + K_p * error);
            break;
        }
        case ControllerType::kOrientation: {
            const tf2::Quaternion quat_desired(path_.pose.pose.orientation.x,
                                               path_.pose.pose.orientation.y,
                                               path_.pose.pose.orientation.z,
                                               path_.pose.pose.orientation.w);
            const double theta_desired = tf2::getYaw<tf2::Quaternion>(quat_desired);
            const double error_theta = theta_desired - theta;

            const double theta_dot_desired = path_.twist.twist.angular.z;

            const double k_theta = get_parameter("theta_gain").as_double();

            const Eigen::Matrix<double, 2, 1> phi{{0.0}, {1.0}};

            u = phi * (theta_dot_desired + k_theta * error_theta);
            break;
        }
    }
    geometry_msgs::msg::Twist ctrl;
    ctrl.linear.x = u(0, 0) * std::cos(theta);
    ctrl.linear.y = u(0, 0) * std::sin(theta);
    ctrl.angular.z = u(1, 0);

    ctrl_pub_->publish(ctrl);
}

} // namespace control

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<control::Controller>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
