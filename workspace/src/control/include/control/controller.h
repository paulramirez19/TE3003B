#ifndef CONTROL_CONTROLLER_H_
#define CONTROL_CONTROLLER_H_

#include <array>
#include <cstdint>
#include <utility>

#include "control/cubic_spline.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/timer.hpp"

namespace control {

class Controller : public rclcpp::Node {
public:
    Controller();

    void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void DesiredPoseCallback(const nav_msgs::msg::Odometry::SharedPtr pose);
    void ControllerCallback();

private:
    enum class ControllerType : std::int32_t { kPosition = 0, kOrientation = 1 };

    Eigen::Matrix<double, 2, 1> PositionController(
            const Eigen::Matrix<double, 2, 1>& q_desired,
            const Eigen::Matrix<double, 2, 1>& q_dot_desired) const;
    Eigen::Matrix<double, 2, 1> OrientationController(double q_desired, double q_dot_desired) const;
    Eigen::Matrix<double, 4, 1> GetSingleAxisBoundaryConditions(std::size_t state_variable) const;
    void SetBoundaryConditions();

    nav_msgs::msg::Odometry odom_;
    nav_msgs::msg::Odometry pose_;
    std::pair<bool, nav_msgs::msg::Odometry> pose_prev_;
    rclcpp::Time initial_time_;
    std::array<CubicSpline, 3> cubic_splines_coeffs_;

    ControllerType controller_type_{ControllerType::kPosition};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr path_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ctrl_pub_;
};

} // namespace control

#endif // CONTROL_CONTROLLER_H_
