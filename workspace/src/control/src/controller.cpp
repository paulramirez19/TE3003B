#include "control/controller.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <optional>
#include <utility>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

namespace control {
namespace {

inline double EvaluatePositionPolynomial(const Eigen::Matrix<double, 4, 1>& a, double t) {
    return a(0, 0) * std::pow(t, 3) + a(1, 0) * std::pow(t, 2) + a(2, 0) * t + a(3, 0);
}

inline double EvaluateVelocityPolynomial(const Eigen::Matrix<double, 4, 1>& a, double t) {
    return 3 * a(0, 0) * std::pow(t, 2) + 2 * a(1, 0) * t + a(2, 0);
}

std::pair<double, double> GetKinematics(const Eigen::Matrix<double, 4, 1>& a, double t) {
    return std::make_pair<double, double>(EvaluatePositionPolynomial(a, t),
                                          EvaluateVelocityPolynomial(a, t));
}

bool EqualPose(const nav_msgs::msg::Odometry& lhs, const nav_msgs::msg::Odometry& rhs) {
    const tf2::Quaternion lhs_quat(lhs.pose.pose.orientation.x, lhs.pose.pose.orientation.y,
                                   lhs.pose.pose.orientation.z, lhs.pose.pose.orientation.w);
    const tf2::Quaternion rhs_quat(rhs.pose.pose.orientation.x, rhs.pose.pose.orientation.y,
                                   rhs.pose.pose.orientation.z, rhs.pose.pose.orientation.w);
    return (lhs.pose.pose.position.x == rhs.pose.pose.position.x) &&
            (lhs.pose.pose.position.y == rhs.pose.pose.position.y) && (lhs_quat == rhs_quat);
}

double GetYawFromOdometry(const nav_msgs::msg::Odometry& odom) {
    const tf2::Quaternion quat(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                               odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    return tf2::getYaw<tf2::Quaternion>(quat);
}

} // namespace

Controller::Controller() : Node("controller") {
    odom_sub_ =
            create_subscription<nav_msgs::msg::Odometry>("/odom_filtered", 10,
                                                         std::bind(&Controller::OdometryCallback,
                                                                   this, std::placeholders::_1));
    path_sub_ =
            create_subscription<nav_msgs::msg::Odometry>("/desired_position", 10,
                                                         std::bind(&Controller::DesiredPoseCallback,
                                                                   this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds{30},
                               std::bind(&Controller::ControllerCallback, this));
    ctrl_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    initial_time_ = get_clock()->now();

    declare_parameter<double>("distance_h", 1.0);
    declare_parameter<double>("x_gain", 1.0);
    declare_parameter<double>("y_gain", 1.0);
    declare_parameter<double>("theta_gain", 1.0);
    declare_parameter<double>("trajectory_duration", 10.0);
}

void Controller::OdometryCallback(const nav_msgs::msg::Odometry& odom) {
    odom_ = odom;
}

void Controller::DesiredPoseCallback(const nav_msgs::msg::Odometry& pose) {
    pose_ = pose;
}

void Controller::ControllerCallback() {
    if (!pose_prev_.has_value()) {
        pose_prev_ = std::make_optional(pose_);
        SetBoundaryConditions();
    }
    if (pose_prev_.has_value() && !EqualPose(*pose_prev_, pose_)) {
        initial_time_ = get_clock()->now();
        pose_prev_ = pose_;
        SetBoundaryConditions();
    }
    const rclcpp::Duration time = get_clock()->now() - initial_time_;
    const auto [x_desired, x_dot_desired] =
            GetKinematics(cubic_splines_coeffs_[0].GetCoefficients(), time.seconds());
    const auto [y_desired, y_dot_desired] =
            GetKinematics(cubic_splines_coeffs_[1].GetCoefficients(), time.seconds());
    const auto [theta_desired, theta_dot_desired] =
            GetKinematics(cubic_splines_coeffs_[2].GetCoefficients(), time.seconds());
    const Eigen::Matrix<double, 2, 1> q_desired{{x_desired}, {y_desired}};
    const Eigen::Matrix<double, 2, 1> q_dot_desired{{x_dot_desired}, {y_dot_desired}};
    const Eigen::Matrix<double, 2, 1> u = (controller_type_ == ControllerType::kPosition)
            ? (PositionController(q_desired, q_dot_desired))
            : (OrientationController(theta_desired, theta_dot_desired));

    geometry_msgs::msg::Twist ctrl;

    const double theta = GetYawFromOdometry(odom_);
    ctrl.linear.x = u(0, 0) * std::cos(theta);
    ctrl.linear.y = u(0, 0) * std::sin(theta);
    ctrl.angular.z = u(1, 0);

    RCLCPP_INFO(get_logger(), "linear_velocity: %g, angular_velocity: %g", u(0, 0), u(1, 0));

    ctrl_pub_->publish(ctrl);
}

Eigen::Matrix<double, 2, 1> Controller::PositionController(
        const Eigen::Matrix<double, 2, 1>& q_desired,
        const Eigen::Matrix<double, 2, 1>& q_dot_desired) const {
    const double h = get_parameter("distance_h").as_double();
    const double theta = GetYawFromOdometry(odom_);
    // clang-format off
    const Eigen::Matrix<double, 2, 2> D{{std::cos(theta), -h * std::sin(theta)},
                                        {std::sin(theta),  h * std::cos(theta)}};
    // clang-format on
    const Eigen::Matrix<double, 2, 1> q{{odom_.pose.pose.position.x}, {odom_.pose.pose.position.y}};

    const double k_x = get_parameter("x_gain").as_double();
    const double k_y = get_parameter("y_gain").as_double();
    const Eigen::Matrix<double, 2, 2> K_p{{k_x, 0.0}, {0.0, k_y}};
    return D.inverse() * (q_dot_desired + K_p * (q_desired - q));
}

Eigen::Matrix<double, 2, 1> Controller::OrientationController(double theta_desired,
                                                              double theta_dot_desired) const {
    const double theta = GetYawFromOdometry(odom_);
    const double k_theta = get_parameter("theta_gain").as_double();
    const Eigen::Matrix<double, 2, 1> phi{{0.0}, {1.0}};
    return phi * (theta_dot_desired + k_theta * (theta_desired - theta));
}

Eigen::Matrix<double, 4, 1> Controller::GetSingleAxisBoundaryConditions(
        std::size_t state_variable) const {
    switch (state_variable) {
        case 0: {
            return Eigen::Matrix<double, 4, 1>{{odom_.pose.pose.position.x},
                                               {pose_prev_->pose.pose.position.x},
                                               {0.0},
                                               {0.0}};
        }
        case 1: {
            return Eigen::Matrix<double, 4, 1>{{odom_.pose.pose.position.y},
                                               {pose_prev_->pose.pose.position.y},
                                               {0.0},
                                               {0.0}};
        }
        case 2: {
            return Eigen::Matrix<double, 4, 1>{{GetYawFromOdometry(odom_)},
                                               {GetYawFromOdometry(*pose_prev_)},
                                               {0.0},
                                               {0.0}};
        }
        default: {
            RCLCPP_FATAL(get_logger(), "Invalid state variable index!");
            return Eigen::Matrix<double, 4, 1>();
        }
    }
}

void Controller::SetBoundaryConditions() {
    constexpr std::size_t num_state_vars = 3;
    for (std::size_t state_var_idx = 0; state_var_idx < num_state_vars; ++state_var_idx) {
        cubic_splines_coeffs_[state_var_idx].SetBoundaryConditions(
                GetSingleAxisBoundaryConditions(state_var_idx));
    }
}

} // namespace control

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<control::Controller>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
