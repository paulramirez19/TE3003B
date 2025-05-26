#include "sensor_fusion/sensor_fusion.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"

namespace sensor_fusion {
namespace {

double GetYawFromOdometry(const nav_msgs::msg::Odometry& odom) {
    const tf2::Quaternion quat(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                               odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    return tf2::getYaw<tf2::Quaternion>(quat);
}

} // namespace

SensorFusion::SensorFusion() : Node("sensor_fusion"), filter_{get_clock(), get_logger()} {
    odom_sub_ =
            create_subscription<nav_msgs::msg::Odometry>("/odom_raw", 10,
                                                         std::bind(&SensorFusion::OdometryCallback,
                                                                   this, std::placeholders::_1));
    pose_sub_ = create_subscription<
            geometry_msgs::msg::PoseStamped>("/pose_stamped", 10,
                                             std::bind(&SensorFusion::LidarCallback, this,
                                                       std::placeholders::_1));
    ctrl_sub_ =
            create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
                                                           std::bind(&SensorFusion::ControlCallback,
                                                                     this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds{30},
                               std::bind(&SensorFusion::FusedSensorsCallback, this));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom_filtered", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    declare_parameter<std::vector<double>>("process_noise_covariance", std::vector<double>(9, 0.0));
    declare_parameter<std::vector<double>>("observation_noise_covariance", std::vector<double>(9, 0.0));

    const std::vector<double> process_noise =
            get_parameter("process_noise_covariance").as_double_array();
    const std::vector<double> observation_noise =
            get_parameter("observation_noise_covariance").as_double_array();
    
    filter_.SetProcessCovariance(process_noise);
    filter_.SetObservationCovariance(observation_noise);
}

void SensorFusion::OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr odom) {
    const std::lock_guard<std::mutex> lg{mutex_};
    const Observation observation{*odom};
    observations_.push(observation);
}

void SensorFusion::LidarCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    const std::lock_guard<std::mutex> lg{mutex_};
    const Observation observation{*pose};
    observations_.push(observation);
}

void SensorFusion::ControlCallback(const geometry_msgs::msg::Twist::SharedPtr ctrl) {
    const double x = ctrl->linear.x;
    const double y = ctrl->linear.y;
    const double linear_velocity = std::sqrt(x * x + y * y);
    control_(0, 0) = linear_velocity;
    control_(1, 0) = ctrl->angular.z;
}

void SensorFusion::FusedSensorsCallback() {
    const std::lock_guard<std::mutex> lg{mutex_};
    nav_msgs::msg::Odometry filtered_odom;
    filtered_odom = filter_.Predict(get_clock()->now(), control_);
    while (!observations_.empty()) {
        filtered_odom = filter_.Update(observations_.top());
        observations_.pop();
    }

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    // Position
    t.transform.translation.x = filtered_odom.pose.pose.position.x;
    t.transform.translation.y = filtered_odom.pose.pose.position.y;
    t.transform.translation.z = 0.0;
    // Orientation
    t.transform.rotation.x = filtered_odom.pose.pose.orientation.x;
    t.transform.rotation.y = filtered_odom.pose.pose.orientation.y;
    t.transform.rotation.z = filtered_odom.pose.pose.orientation.z;
    t.transform.rotation.w = filtered_odom.pose.pose.orientation.w;

    tf_broadcaster_->sendTransform(t);

    RCLCPP_INFO(get_logger(), "x: %.6lf, y: %.6lf, heading: %.6lf",
                filtered_odom.pose.pose.position.x, filtered_odom.pose.pose.position.y,
                GetYawFromOdometry(filtered_odom));
    odom_pub_->publish(filtered_odom);
}

} // namespace sensor_fusion

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sensor_fusion::SensorFusion>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
