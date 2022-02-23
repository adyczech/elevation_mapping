/*
 * elevation_map_node.cpp
 *
 *  Created on: Feb 21, 2022
 *      Author: Sivert Havso
 *   Institute: Aberystwyth University
 */

#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include "elevation_mapping/ElevationMapping.hpp"
#include "utilities.hpp"

using elevation_mapping::configure_thread;
using elevation_mapping::get_thread_time;
using elevation_mapping::ThreadPriority;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  
  rclcpp::executors::MultiThreadedExecutor high_prio_executor;
  rclcpp::executors::MultiThreadedExecutor med_prio_executor;
  rclcpp::executors::MultiThreadedExecutor default_prio_executor;
  rclcpp::executors::MultiThreadedExecutor low_prio_executor;

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto elevationMap = std::make_shared<elevation_mapping::ElevationMapping>(options);

  rclcpp::Logger logger = elevationMap->get_logger();

  default_prio_executor.add_node(elevationMap);

  // Fusion is main priority
  high_prio_executor.add_callback_group(
    elevationMap->getPointcloudCallbackGroup(), elevationMap->get_node_base_interface());
  
  med_prio_executor.add_callback_group(
    elevationMap->getFusionCallbackGroup(), elevationMap->get_node_base_interface());
  
  // Second priority is visibility cleanup
  low_prio_executor.add_callback_group(
    elevationMap->getVisibilityCleanupCallbackGroup(), elevationMap->get_node_base_interface());

  // Parameters, loading and saving maps, clearing map are low priority
  // low_prio_executor.add_callback_group(
  //   elevationMap->getDefaultCallbackGroup(), elevationMap->get_node_base_interface());

  // Create a thread for each of the executors ...
  auto high_prio_thread = std::thread(
    [&]() {
      high_prio_executor.spin();
    });
  auto med_prio_thread = std::thread(
    [&]() {
      med_prio_executor.spin();
    });
  auto low_prio_thread = std::thread(
    [&]() {
      low_prio_executor.spin();
    });

  // ... and configure them accordinly as high, medium, and low prio and pin them to the
  // first CPU.
  const int CPU_ZERO = 0;
  bool ret = configure_thread(high_prio_thread, ThreadPriority::HIGH, CPU_ZERO);
  if (!ret) {
    RCLCPP_WARN(logger, "Failed to configure high priority thread, are you root?");
  }
  ret = configure_thread(med_prio_thread, ThreadPriority::MEDIUM, CPU_ZERO);
  if (!ret) {
    RCLCPP_WARN(logger, "Failed to configure medium priority thread, are you root?");
  }
  ret = configure_thread(low_prio_thread, ThreadPriority::LOW, CPU_ZERO);
  if (!ret) {
    RCLCPP_WARN(logger, "Failed to configure low priority thread, are you root?");
  }

  default_prio_executor.spin();  // Blocking spin
  rclcpp::shutdown();

  high_prio_thread.join();
  med_prio_thread.join();
  low_prio_thread.join();

  return EXIT_SUCCESS;
}
