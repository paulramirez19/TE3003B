#include "sensor_fusion/extended_kalman_filter.h"

#include <gtest/gtest.h>
#include <eigen3/Eigen/Core>
#include <memory>
#include <cmath>
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_fusion/observation.h"
#include "rclcpp/rclcpp.hpp"

class ExtendedKalmanFilterTest : public testing::Test {
protected:
    ExtendedKalmanFilterTest()
          : clock_{std::make_shared<rclcpp::Clock>()},
            initial_time_(clock_->now()),
            extended_kalman_filter_{clock_} {}

    std::shared_ptr<rclcpp::Clock> clock_;
    rclcpp::Time initial_time_;
    sensor_fusion::ExtendedKalmanFilter extended_kalman_filter_;
};

TEST_F(ExtendedKalmanFilterTest, IsCorrectlyInitialized) {
    const Eigen::Matrix<double, 2, 1> u{0.0, 0.0};
    const nav_msgs::msg::Odometry prediction = extended_kalman_filter_.Predict(initial_time_, u);
    EXPECT_EQ(prediction.pose.pose.position.x, 0);
    EXPECT_EQ(prediction.pose.pose.position.y, 0);
}

TEST_F(ExtendedKalmanFilterTest, DoesUpdate) {
    nav_msgs::msg::Odometry odom;
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    const sensor_fusion::Observation obser{odom};
    nav_msgs::msg::Odometry update = extended_kalman_filter_.Update(obser);
    EXPECT_TRUE(std::isnan(update.pose.pose.position.x));
    EXPECT_TRUE(std::isnan(update.pose.pose.position.y));
}
