#include "sensor_fusion/observation.h"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace sensor_fusion {

Observation::Observation(const nav_msgs::msg::Odometry& observation)
      : header_(observation.header), pose_(observation.pose.pose) {}

Observation::Observation(const geometry_msgs::msg::PoseStamped& observation)
      : header_(observation.header), pose_(observation.pose) {}

Observation::Observation(const Observation& observation)
      : header_(observation.header_), pose_(observation.pose_) {}

Observation& Observation::operator=(const Observation& observation) {
    if (this != &observation) {
        header_ = observation.header_;
        pose_ = observation.pose_;
    }
    return *this;
}

geometry_msgs::msg::PoseStamped Observation::GetObservation() const {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header_;
    pose.pose = pose_;
    return pose;
}

bool operator<(const Observation& lhs, const Observation& rhs) {
    return lhs.header_.stamp.nanosec < rhs.header_.stamp.nanosec;
}

bool operator>(const Observation& lhs, const Observation& rhs) {
    return lhs.header_.stamp.nanosec > rhs.header_.stamp.nanosec;
}

} // namespace sensor_fusion
