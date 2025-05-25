#ifndef SENSOR_FUSION_OBSERVATION_H_
#define SENSOR_FUSION_OBSERVATION_H_

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace sensor_fusion {

class Observation {
public:
    Observation() = delete;
    Observation(const nav_msgs::msg::Odometry& observation);
    Observation(const Observation& observation);
    Observation& operator=(const Observation& observation);

    const nav_msgs::msg::Odometry& GetObservation() const;

    friend bool operator<(const Observation& lhs, const Observation& rhs);
    friend bool operator>(const Observation& lhs, const Observation& rhs);

private:
    nav_msgs::msg::Odometry observation_;
};

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION_OBSERVATION_H_
