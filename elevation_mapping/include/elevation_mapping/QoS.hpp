/*
 * QoS.hpp
 *
 *  Created on: Feb 13, 2022
 *      Author: Sivert Havso
 *	 Institute: Aberystwyth University
 */

#pragma once

#include <rclcpp/qos.hpp>

namespace elevation_mapping
{

static const rmw_qos_profile_t qos_grid_map =
{
  RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  1,
  RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  RMW_QOS_POLICY_DURABILITY_VOLATILE,  // 'transient local' not allowed for intra process communication
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

/**
 * @brief Get a rclcpp::QoS object with the default elevation map settings.
 * 
 * @param[in] depth Set a custom size of the message queue. Defaults to 1
 * @param[in] deadline Set a custom deadline. Default is no deadline.
 * @return rclcpp::QoS default elevation map QoS
 */
inline rclcpp::QoS default_qos(
  size_t depth = 1, 
  const rclcpp::Duration deadline = rclcpp::Duration::from_nanoseconds(0)) {
  // First create a rmw_qos_profile_t to set the depth - rclcpp::QoS has no set depth function
  auto rmw_qos = qos_grid_map;
  rmw_qos.depth = depth;

  // Create a rclcpp::QoS object to set the deadline and return it
  auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos));
  // qos.deadline(deadline);
  return qos;
} 

}
