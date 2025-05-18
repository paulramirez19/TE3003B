#ifndef CONTROL_CONTROLLER_H_
#define CONTROL_CONTROLLER_H_

#include <cstdint>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

namespace control {

class Controller : public rclcpp::Node {
public:
    Controller();

    void OdometryCallback(const nav_msgs::msg::Odometry& odom);
    void DesiredPositionCallback(const nav_msgs::msg::Odometry& path);
    void ControllerCallback();

private:
    enum class ControllerType : std::int32_t { kPosition = 0, kOrientation = 1 };

    nav_msgs::msg::Odometry odom_;
    nav_msgs::msg::Odometry path_;
    ControllerType controller_type_{ControllerType::kPosition};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr path_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ctrl_pub_;
};

} // namespace control

#endif // CONTROL_CONTROLLER_H_
