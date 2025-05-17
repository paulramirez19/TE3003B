#include "sensor_fusion/extended_kalman_filter.h"

#include <array>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace sensor_fusion {
namespace {

constexpr double kH{0.10};  // meters

// clang-format off
const Eigen::Matrix<double, 3, 3> Q{
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0}
};

const Eigen::Matrix<double, 3, 3> R{
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},
};

const Eigen::Matrix<double, 3, 3> H{
    {1.0, 0.0, 0.0},
    {0.0, 1.0, 0.0},
    {0.0, 0.0, 1.0}
};

// clang-format on

Eigen::Matrix<double, 3, 2> B(const Eigen::Matrix<double, 3, 1>& x) {
    // clang-format off
    return Eigen::Matrix<double, 3, 2>{
        {std::cos(x(2, 0)), -kH * std::sin(x(2, 0))},
        {std::sin(x(2, 0)),  kH * std::cos(x(2, 0))},
        {0.0, 1.0}
    };
    // clang-format on
}

Eigen::Matrix<double, 3, 3> F(const Eigen::Matrix<double, 3, 1> x,
                              const Eigen::Matrix<double, 2, 1> u) {
    // clang-format off
    return Eigen::Matrix<double, 3, 3>{
        {0.0, 0.0, -u(0, 0) * std::sin(x(2, 0)) - kH * std::cos(x(2, 0)) * u(1, 0)},
        {0.0, 0.0,  u(0, 0) * std::cos(x(2, 0)) - kH * std::sin(x(2, 0)) * u(1, 0)},
        {0.0, 0.0, 0.0}
    };
    // clang-format on
}

nav_msgs::msg::Odometry BuildOdometryMessage(
    const rclcpp::Time& time, const Eigen::Matrix<double, 3, 1>& x,
    const Eigen::Matrix<double, 3, 3>& P) {
    nav_msgs::msg::Odometry odom;

    odom.header.stamp = time;

    odom.pose.pose.position.x = x(0, 0);
    odom.pose.pose.position.y = x(1, 0);

    tf2::Quaternion quat;
    quat.setRPY(/*roll=*/0.0, /*pitch=*/0.0, /*yaw=*/x(2, 0));
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();

    /*
     * [x, y, z, Roll, Pitch, Yaw]
     *
     * [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
     *  [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
     *  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
     *  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
     *  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
     *  [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]
     */
    odom.pose.covariance[0] = P(0, 0);
    odom.pose.covariance[7] = P(1, 0);
    odom.pose.covariance[35] = P(2, 0);
    return odom;
}

Eigen::Matrix<double, 3, 1> ConvertObservationToState(
    const Observation& observation) {
    const nav_msgs::msg::Odometry& obser = observation.GetObservation();
    const tf2::Quaternion quat(
        obser.pose.pose.orientation.x, obser.pose.pose.orientation.y,
        obser.pose.pose.orientation.z, obser.pose.pose.orientation.w);
    const tf2::Matrix3x3 matrix(quat);
    double roll{};
    double pitch{};
    double yaw{};
    matrix.getRPY(roll, pitch, yaw);
    return Eigen::Matrix<double, 3, 1>{obser.pose.pose.position.x,
                                       obser.pose.pose.position.y, yaw};
}

}  // namespace

ExtendedKalmanFilter::ExtendedKalmanFilter() : prev_time_() {}

nav_msgs::msg::Odometry ExtendedKalmanFilter::Predict(
    const rclcpp::Time& time, const Eigen::Matrix<double, 2, 1>& control) {
    const Eigen::Matrix<double, 3, 1> x_dot = B(prev_state_) * control;
    const double delta_time = (time - prev_time_).seconds();
    const Eigen::Matrix<double, 3, 1> x_pred = prev_state_ + delta_time * x_dot;

    // clang-format off
    const Eigen::Matrix<double, 3, 3> covariance_pred =
        F(prev_state_, control) * prev_covariance_ * (F(prev_state_, control).transpose()) + Q;
    // clang-format on

    prev_time_ = time;
    prev_state_ = x_pred;
    prev_covariance_ = covariance_pred;
    return BuildOdometryMessage(time, x_pred, covariance_pred);
}

nav_msgs::msg::Odometry ExtendedKalmanFilter::Update(
    const Observation& observation) {
    // clang-format off
    const Eigen::Matrix<double, 3, 1> obser_state = ConvertObservationToState(observation);
    const Eigen::Matrix<double, 3, 1> y = (prev_state_ - obser_state);
    const Eigen::Matrix<double, 3, 3> S = H * prev_covariance_ * (H.transpose()) + R;
    const Eigen::Matrix<double, 3, 3> K = prev_covariance_ * (H.transpose()) * (S.inverse());
    prev_state_ = prev_state_ + K * y;
    const Eigen::Matrix<double, 3, 3> I = Eigen::Matrix<double, 3, 3>::Identity();
    prev_covariance_ = (I - K * H) * prev_covariance_;
    // clang-format on
    return BuildOdometryMessage(prev_time_, prev_state_, prev_covariance_);
}

}  // namespace sensor_fusion
