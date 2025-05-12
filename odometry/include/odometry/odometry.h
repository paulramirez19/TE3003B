#ifndef ODOMETRY_ODOMETRY_H_
#define ODOMETRY_ODOMETRY_H_

#include <chrono>
#include <memory>
#include <ratio>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace odometry {

class Odometry : public rclcpp::Node {
public:
    Odometry();

private:
    void odometry_callback(const geometry_msgs::msg::Twist& msg);

    double x_pos_{0.0};
    double y_pos_{0.0};
    double heading_{0.0};
    rclcpp::Time last_vel_time_{};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace odometry

#endif  // ODOMETRY_H_