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
  RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
  RMW_QOS_DEADLINE_DEFAULT,
  RMW_QOS_LIFESPAN_DEFAULT,
  RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  false
};

/**
 * @brief Get a rclcpp::QoS object with the default elevation map settings.
 * 
 * @param depth Set a custom size of the message queue. Defaults to 1
 * @return rclcpp::QoS default elevation map QoS
 */
inline rclcpp::QoS default_qos(size_t depth = 1) {
  auto rmw_qos = qos_grid_map;
  rmw_qos.depth = depth;
  return rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_grid_map));
} 

}
