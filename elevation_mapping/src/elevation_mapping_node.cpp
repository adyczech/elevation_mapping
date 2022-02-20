/*
 * elevation_map_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <rclcpp/rclcpp.hpp>
#include "elevation_mapping/ElevationMapping.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv, "elevation_mapping");
  rclcpp::NodeHandle nodeHandle("~");
  elevation_mapping::ElevationMapping elevationMap(nodeHandle);

  // Spin
  rclcpp::AsyncSpinner spinner(nodeHandle.param("num_callback_threads", 1));  // Use n threads
  spinner.start();
  rclcpp::waitForShutdown();
  return 0;
}
