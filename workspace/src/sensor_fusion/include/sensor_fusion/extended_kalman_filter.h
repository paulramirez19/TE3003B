#ifndef SENSOR_FUSION_EXTENDED_KALMAN_FILTER_H_
#define SENSOR_FUSION_EXNTEDED_KALMAN_FILTER_H_

#include <array>
#include <cmath>
#include <eigen3/Eigen/Core>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/logger.hpp"
#include "sensor_fusion/observation.h"

namespace sensor_fusion {

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter(const rclcpp::Clock::SharedPtr& clock);

    nav_msgs::msg::Odometry Predict(const rclcpp::Time& time,
                                    const Eigen::Matrix<double, 2, 1>& control);
    nav_msgs::msg::Odometry Update(const Observation& observation);

    void SetProcessCovariance(const std::vector<double>& cov);
    void SetObservationCovariance(const std::vector<double>& cov);

private:
    const rclcpp::Clock::SharedPtr& clock_;
    rclcpp::Logger logger_;
    // Transitions are from k - 1 to k where k - 1 is denoted by "prev"
    rclcpp::Time prev_time_;
    Eigen::Matrix<double, 3, 1> prev_state_;
    Eigen::Matrix<double, 3, 3> prev_covariance_;
    Eigen::Matrix<double, 3, 3> Q_;
    Eigen::Matrix<double, 3, 3> R_;
};

} // namespace sensor_fusion

#endif // SENSOR_FUSION_EXTENDED_KALMAN_FILTER_H_
