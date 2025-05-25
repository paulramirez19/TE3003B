#include "sensor_fusion/sensor_fusion.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sensor_fusion {

SensorFusion::SensorFusion() : Node("sensor_fusion"), filter_{} {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom_raw", 10,
        std::bind(&SensorFusion::OdometryCallback, this,
                  std::placeholders::_1));
    scan_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/scan", 10,
        std::bind(&SensorFusion::LidarCallback, this, std::placeholders::_1));
    ctrl_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&SensorFusion::ControlCallback, this, std::placeholders::_1));
    timer_ =
        create_wall_timer(std::chrono::milliseconds{30},
                          std::bind(&SensorFusion::FusedSensorsCallback, this));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom_fused", 10);
}

void SensorFusion::OdometryCallback(const nav_msgs::msg::Odometry& odom) {
    std::scoped_lock<std::mutex> sl{mutex_};
    const Observation observation{odom};
    observations_.push(observation);
}

void SensorFusion::LidarCallback(const nav_msgs::msg::Odometry& scan) {
    std::scoped_lock<std::mutex> sl{mutex_};
    const Observation observation{scan};
    observations_.push(observation);
}

void SensorFusion::ControlCallback(const geometry_msgs::msg::Twist& ctrl) {
    const double x = ctrl.linear.x;
    const double y = ctrl.linear.y;
    const double linear_velocity = std::sqrt(x * x + y + y);
    control_(0, 0) = linear_velocity;
    control_(1, 0) = ctrl.angular.z;
}

void SensorFusion::FusedSensorsCallback() {
    std::scoped_lock<std::mutex> sl{mutex_};
    nav_msgs::msg::Odometry filtered_odometry;
    filtered_odometry = filter_.Predict(rclcpp::Clock().now(), control_);
    while (!observations_.empty()) {
        filter_.Update(observations_.top());
        observations_.pop();
    }
    odom_pub_->publish(filtered_odometry);
}

}  // namespace sensor_fusion

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sensor_fusion::SensorFusion>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
