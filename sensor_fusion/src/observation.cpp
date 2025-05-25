#include "sensor_fusion/observation.h"

#include "nav_msgs/msg/odometry.hpp"

namespace sensor_fusion {

Observation::Observation(const nav_msgs::msg::Odometry& observation)
    : observation_(observation) {}

Observation::Observation(const Observation& observation)
    : observation_{observation.observation_} {}

Observation& Observation::operator=(const Observation& observation) {
    if (this != &observation) {
        observation_ = observation.observation_;
    }
    return *this;
}

const nav_msgs::msg::Odometry& Observation::GetObservation() const {
    return observation_;
}

bool operator<(const Observation& lhs, const Observation& rhs) {
    return lhs.observation_.header.stamp.nanosec <
           rhs.observation_.header.stamp.nanosec;
}

bool operator>(const Observation& lhs, const Observation& rhs) {
    return lhs.observation_.header.stamp.nanosec >
           rhs.observation_.header.stamp.nanosec;
}

}  // namespace sensor_fusion
