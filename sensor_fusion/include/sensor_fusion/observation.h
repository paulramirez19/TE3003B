#ifndef SENSOR_FUSION_OBSERVATION_H_
#define SENSOR_FUSION_OBSERVATION_H_

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"

namespace sensor_fusion {

class Observation {
public:
    Observation() = delete;
    Observation(const nav_msgs::msg::Odometry& observation);
    Observation(const geometry_msgs::msg::PoseStamped& observation);
    Observation(const Observation& observation);
    Observation& operator=(const Observation& observation);

    geometry_msgs::msg::PoseStamped GetObservation() const;

    friend bool operator<(const Observation& lhs, const Observation& rhs);
    friend bool operator>(const Observation& lhs, const Observation& rhs);

private:
    std_msgs::msg::Header header_;
    geometry_msgs::msg::Pose pose_;
};

} // namespace sensor_fusion

#endif // SENSOR_FUSION_OBSERVATION_H_
