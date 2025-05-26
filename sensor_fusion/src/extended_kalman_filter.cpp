#include "sensor_fusion/extended_kalman_filter.h"

#include <array>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.hpp"

namespace sensor_fusion {
namespace {

constexpr double kH{0.10}; // meters

Eigen::Matrix<double, 3, 3> H() {
    Eigen::Matrix<double, 3, 3> H_internal;
    H_internal << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    return H_internal;
}

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
    Eigen::Matrix<double, 3, 3> F_internal;
    F_internal << 
        0.0, 0.0, -u(0, 0) * std::sin(x(2, 0)) - kH * std::cos(x(2, 0)) * u(1, 0),
        0.0, 0.0,  u(0, 0) * std::cos(x(2, 0)) - kH * std::sin(x(2, 0)) * u(1, 0),
        0.0, 0.0, 0.0;

    return F_internal;
    // clang-format on
}

nav_msgs::msg::Odometry BuildOdometryMessage(const rclcpp::Time& time,
                                             const Eigen::Matrix<double, 3, 1>& x,
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

Eigen::Matrix<double, 3, 1> ConvertObservationToState(const Observation& observation) {
    const geometry_msgs::msg::PoseStamped& obser = observation.GetObservation();
    const tf2::Quaternion quat(obser.pose.orientation.x, obser.pose.orientation.y,
                               obser.pose.orientation.z, obser.pose.orientation.w);
    const double yaw = tf2::getYaw<tf2::Quaternion>(quat);
    return Eigen::Matrix<double, 3, 1>{obser.pose.position.x, obser.pose.position.y, yaw};
}

Eigen::Matrix<double, 3, 3> ConvertVectorToMatrix(const std::vector<double>& vec) {
    return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(vec.data());
}

} // namespace

ExtendedKalmanFilter::ExtendedKalmanFilter(const rclcpp::Clock::SharedPtr& clock, const rclcpp::Logger& logger)
      : clock_{clock}, logger_{logger}, prev_time_{clock_->now()} {}

nav_msgs::msg::Odometry ExtendedKalmanFilter::Predict(const rclcpp::Time& time,
                                                      const Eigen::Matrix<double, 2, 1>& control) {
    const Eigen::Matrix<double, 3, 1> x_dot = B(prev_state_) * control;
    const double delta_time = (time - prev_time_).seconds();
    const Eigen::Matrix<double, 3, 1> x_pred = prev_state_ + delta_time * x_dot;

    // clang-format off
    const Eigen::Matrix<double, 3, 3> covariance_pred =
        F(prev_state_, control) * prev_covariance_ * (F(prev_state_, control).transpose()) + Q_;
    // clang-format on

    prev_time_ = time;
    prev_state_ = x_pred;
    prev_covariance_ = covariance_pred;
    return BuildOdometryMessage(time, x_pred, covariance_pred);
}

nav_msgs::msg::Odometry ExtendedKalmanFilter::Update(const Observation& observation) {
    // clang-format off
    const Eigen::Matrix<double, 3, 1> obser_state = ConvertObservationToState(observation);
    const Eigen::Matrix<double, 3, 1> y = (prev_state_ - obser_state);
    const Eigen::Matrix<double, 3, 3> S = H() * prev_covariance_ * (H().transpose()) + R_;
    const Eigen::Matrix<double, 3, 3> K = prev_covariance_ * (H().transpose()) * (S.inverse());
    prev_state_ = prev_state_ + K * y;
    const Eigen::Matrix<double, 3, 3> I = Eigen::Matrix<double, 3, 3>::Identity();
    prev_covariance_ = (I - K * H()) * prev_covariance_;
    // clang-format on
    return BuildOdometryMessage(prev_time_, prev_state_, prev_covariance_);
}

void ExtendedKalmanFilter::SetProcessCovariance(const std::vector<double>& cov) {
    Q_ = ConvertVectorToMatrix(cov);
}

void ExtendedKalmanFilter::SetObservationCovariance(const std::vector<double>& cov) {
    R_ = ConvertVectorToMatrix(cov);
}

} // namespace sensor_fusion
