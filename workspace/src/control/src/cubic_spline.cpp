#include "control/cubic_spline.h"

#include <chrono>
#include <cmath>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace control {
namespace {}  // namespace

void CubicSpline::SetTrajectoryDuration(double trajectory_duration) {
    T_ = trajectory_duration;
}

void CubicSpline::SetBoundaryConditions(const Eigen::Matrix<double, 4, 1>& boundary_conditions) {
    boundary_conditions_ = boundary_conditions;
}

Eigen::Matrix<double, 4, 1> CubicSpline::GetCoefficients() const {
    return matrix_.inverse() * boundary_conditions_;
}

}  // namespace control
