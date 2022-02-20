#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv, "elevation_mapping");
  rclcpp::start();  // To make use of ROS time in output macros.
  testing::InitGoogleTest(&argc, argv);
  int res = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return res;
}
