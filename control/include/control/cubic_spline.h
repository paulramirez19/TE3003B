#ifndef CONTROL_CUBIC_SPLINE_H_
#define CONTROL_CUBIC_SPLINE_H_

#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Core>

namespace control {

class CubicSpline {
public:
    CubicSpline() = default;

    void SetTrajectoryDuration(double trajectory_duration);
    void SetBoundaryConditions(const Eigen::Matrix<double, 4, 1>& boundary_conditions);
    Eigen::Matrix<double, 4, 1> GetCoefficients() const;

private:
    double T_;
    Eigen::Matrix<double, 4, 1> boundary_conditions_;
    // clang-format off
    Eigen::Matrix<double, 4, 4> matrix_{
        {0.0, 0.0, 0.0, 1.0},
        {std::pow(T_, 3.0), std::pow(T_, 2.0), T_, 1.0},
        {0.0, 0.0, 1.0, 0.0},
        {3 * std::pow(T_, 2.0), 2 * T_, 1.0, 0.0}
    };
    // clang-format on
};

} // namespace control

#endif // CONTROL_CUBIC_SPLINE_H_
