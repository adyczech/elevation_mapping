/*
 *  InputSourceTest.cpp
 *
 *  Created on: Oct 02, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/ElevationMapping.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

static void assertSuccessAndNumberOfSources(const std::string& inputConfiguration, bool successExpected,
                                            uint32_t numberOfExpectedInputSources) {
  elevation_mapping::InputSourceManager inputSourceManager(rclcpp::Node("~"));
  bool success = inputSourceManager.configureFromRos(inputConfiguration);
  ASSERT_EQ(success, successExpected) << "Configuration was:\n"
                                      << rclcpp::Node("~").param<XmlRpc::XmlRpcValue>(inputConfiguration, "not set").toXml() << "\n";
  ASSERT_EQ(inputSourceManager.getNumberOfSources(), numberOfExpectedInputSources);
}

TEST(InputSources, SingleInputValid) {  // NOLINT
  assertSuccessAndNumberOfSources("single_valid", true, 1);
}

TEST(InputSources, MultipleInputsValid) {  // NOLINT
  assertSuccessAndNumberOfSources("multiple_valid", true, 3);
}

TEST(InputSources, NoType) {  // NOLINT
  assertSuccessAndNumberOfSources("no_type", false, 0);
}

TEST(InputSources, NoTopic) {  // NOLINT
  assertSuccessAndNumberOfSources("no_topic", false, 0);
}

TEST(InputSources, NoQueueSize) {  // NOLINT
  assertSuccessAndNumberOfSources("no_queue_size", false, 0);
}

TEST(InputSources, NoPublishOnUpdate) {  // NOLINT
  assertSuccessAndNumberOfSources("no_publish_on_update", false, 0);
}

TEST(InputSources, SubscribingSameTwice) {  // NOLINT
  assertSuccessAndNumberOfSources("subscribing_same_topic_twice", false, 1);
}

TEST(InputSources, ConfigurationNotGiven) {  // NOLINT
  assertSuccessAndNumberOfSources("unset_namespace", false, 0);
}

TEST(InputSources, ConfigurationEmptySources) {  // NOLINT
  assertSuccessAndNumberOfSources("empty_sources_list", true, 0);
}

TEST(InputSources, ConfigurationWrongType) {  // NOLINT
  assertSuccessAndNumberOfSources("wrong_type_configuration", false, 0);
}

TEST(InputSources, ConfigurationNotAStruct) {  // NOLINT
  assertSuccessAndNumberOfSources("not_a_struct", false, 0);
}

TEST(InputSources, ConfigurationQueueSizeIsString) {  // NOLINT
  assertSuccessAndNumberOfSources("queue_size_is_string", false, 0);
}

TEST(InputSources, ConfigurationQueueSizeIsNegative) {  // NOLINT
  assertSuccessAndNumberOfSources("negative_queue_size", false, 0);
}

TEST(InputSources, UnknownType) {  // NOLINT
  rclcpp::Node node("~");
  elevation_mapping::InputSourceManager inputSourceManager(node);
  inputSourceManager.configureFromRos("unknown_type");

  elevation_mapping::ElevationMapping map{node};

  // Trying to register this misconfigured InputSourceManager to our map should fail.
  bool success =
      inputSourceManager.registerCallbacks(map, make_pair("pointcloud", &elevation_mapping::ElevationMapping::pointCloudCallback));
  ASSERT_FALSE(success);
}

TEST(ElevationMap, Constructor) {  // NOLINT
  rclcpp::Node node("~");
  elevation_mapping::ElevationMapping map(node);
}

TEST(InputSources, ListeningToTopicsAfterRegistration) {  // NOLINT
  // subscribe to the default parameter "input_sources"
  rclcpp::Node node("~");
  class ElevationMappingWithInputSourcesAccessor : public elevation_mapping::ElevationMapping {
   public:
    ElevationMappingWithInputSourcesAccessor(rclcpp::Node::SharedPtr node) : elevation_mapping::ElevationMapping(node) {}
    virtual ~ElevationMappingWithInputSourcesAccessor() = default;
    int getNumberOfSources() { return inputSources_.getNumberOfSources(); }
  } map{node};

  // Wait a bit.
  rclcpp::spinOnce();
  rclcpp::Duration(1.0).sleep();
  rclcpp::spinOnce();

  // Publish to the topics we expect map to subscribe.
  rclcpp::Node node("");
  rclcpp::Publisher firstLidarPublisher = node.advertise<sensor_msgs::msg::PointCloud2>("/lidar_1/depth/points", 1, false);
  rclcpp::Publisher secondLidarPublisher = node.advertise<sensor_msgs::msg::PointCloud2>("/lidar_2/depth/points", 1, false);

  // Check if we have exactly one subscriber per topic.
  ASSERT_EQ(firstLidarPublisher.getNumSubscribers(), 1);
  ASSERT_EQ(secondLidarPublisher.getNumSubscribers(), 1);
  // ASSERT_EQ(firstDepthImagePublisher.getNumSubscribers(), 1);
  ASSERT_EQ(map.getNumberOfSources(), 2);
}
