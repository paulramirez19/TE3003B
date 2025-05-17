#ifndef SENSOR_FUSION_SENSOR_FUSION_H_
#define SENSOR_FUSION_SENSOR_FUSION_H_

#include <eigen3/Eigen/Core>
#include <functional>
#include <mutex>
#include <queue>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_fusion/extended_kalman_filter.h"
#include "sensor_fusion/observation.h"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace sensor_fusion {

class SensorFusion : public rclcpp::Node {
public:
    SensorFusion();

private:
    void OdometryCallback(const nav_msgs::msg::Odometry& odom);
    void LidarCallback(const nav_msgs::msg::Odometry& scan);
    void ControlCallback(const geometry_msgs::msg::Twist& ctrl);
    void FusedSensorsCallback();

    std::priority_queue<Observation, std::vector<Observation>,
                        std::greater<Observation>>
        observations_;
    Eigen::Matrix<double, 2, 1> control_;
    std::mutex mutex_;
    ExtendedKalmanFilter filter_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ctrl_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION_SENSOR_FUSION_H_
