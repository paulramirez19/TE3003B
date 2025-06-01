#ifndef CONTROL_CUBIC_SPLINE_H_
#define CONTROL_CUBIC_SPLINE_H_

#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Core>
#include "rclcpp/logger.hpp"

namespace control {

class CubicSpline {
public:
    CubicSpline();

    void SetTrajectoryDuration(double trajectory_duration);
    void SetBoundaryConditions(const Eigen::Matrix<double, 4, 1>& boundary_conditions);
    Eigen::Matrix<double, 4, 1> GetCoefficients() const;

private:
    rclcpp::Logger logger_;
    double T_;
    Eigen::Matrix<double, 4, 1> boundary_conditions_;
    // clang-format off
    Eigen::Matrix<double, 4, 4> matrix_;
    // clang-format on
};

} // namespace control

#endif // CONTROL_CUBIC_SPLINE_H_
