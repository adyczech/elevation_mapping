/*
 * elevation_map_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <rclcpp/rclcpp.hpp>
#include "elevation_mapping/ElevationMapping.hpp"
#include <chrono>

using namespace std::literals::chrono_literals;


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto elevationMap = std::make_shared<elevation_mapping::ElevationMapping>(options);

  rclcpp::sleep_for(1s);

  executor.add_callback_group(elevationMap->getFusionCallbackGroup(), elevationMap->get_node_base_interface());
  executor.add_callback_group(elevationMap->getVisibilityCleanupCallbackGroup(), elevationMap->get_node_base_interface());
  executor.add_callback_group(elevationMap->getPointcloudCallbackGroup(), elevationMap->get_node_base_interface());


  executor.add_node(elevationMap);

  executor.spin();

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
