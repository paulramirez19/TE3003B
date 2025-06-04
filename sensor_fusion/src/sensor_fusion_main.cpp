#include "sensor_fusion/sensor_fusion.h"

#include <cstdlib>
#include <memory>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sensor_fusion::SensorFusion>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
