/*!
 * @file    PostprocessorTest.cpp
 * @authors Magnus Gärtner (ANYbotics)
 * @brief   Tests for the PostprocessorPool and the PostprocessingPipeline.
 */

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "elevation_mapping/postprocessing/PostprocessingPipelineFunctor.hpp"
#include "elevation_mapping/postprocessing/PostprocessorPool.hpp"

/**
 * We read in a postprocessing (mock) configuration that takes 150ms to execute. We test whether the postprocessorPool accepts/discards the
 * tasks for various configurations of time between tasks and number of threads in the pool.
 */
class RosFixture : public ::testing::Test {
  rclcpp::Node::SharedPtr node;

 protected:
  virtual void TearDown() { node.reset(); }

  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
    // const std::map<std::string, std::string> remappings{};
    // rclcpp::init(remappings, "post_processor_ros_test");
    // rclcpp::start();
  }

  void initialize(std::string name) {
    rclcpp::NodeOptions node_options;
    std::vector<std::string> args = {"--ros-args", "--params-file", "/home/sih26/galactic_ws/src/elevation_mapping/elevation_mapping/test/postprocessing/postprocessor_pipeline.yaml"};
    node_options.arguments(args);
    // node = std::make_shared<rclcpp::Node>("test_postprocessor_pool", "/ns", node_options);
    node = std::make_shared<rclcpp::Node>(std::string("test_postprocessor_pool" + name), node_options);
  }


 public:

  static void checkAcceptedTasks(uint poolSize, uint timeBetweenConsecutiveTasks, std::vector<bool> expectedAcceptanceOutcomes) {
    // Set up ROS node handle.
    auto node = std::make_shared<rclcpp::Node>("test_postprocessor_pool");

    elevation_mapping::PostprocessorPool pool(poolSize, node);
    int taskNumber = 0;
    for (auto expectedOutcome : expectedAcceptanceOutcomes) {
      bool accepted = pool.runTask(grid_map::GridMap());
      if (expectedOutcome) {
        ASSERT_TRUE(accepted) << "Postprocessor pool rejected task number: " << taskNumber << " although it should.";
      } else {
        ASSERT_FALSE(accepted) << "Postprocessor pool accepted task number: " << taskNumber << " although it should not. ";
      }
      taskNumber++;
      std::this_thread::sleep_for(std::chrono::milliseconds(timeBetweenConsecutiveTasks));
    }
  }
};

TEST_F(RosFixture, FiveTasksOneThreadSimultaneously) {  // NOLINT
  initialize("1");
  checkAcceptedTasks(1, 0, {true, false, false, false, false});
}

TEST_F(RosFixture, FiveTasksTwoThreadSimultaneously) {  // NOLINT
  initialize("2");
  checkAcceptedTasks(2, 0, {true, true, false, false, false});
}

TEST_F(RosFixture, EnoughTimeToProcess) {  // NOLINT
  initialize("3");
  checkAcceptedTasks(1, 200, {true, true, true, true, true});
}

TEST_F(RosFixture, EnoughTimeToProcessWithTwoThreads) {  // NOLINT
  initialize("4");
  checkAcceptedTasks(2, 100, {true, true, true, true, true});
}

TEST_F(RosFixture, ProcessEverySecond) {  // NOLINT
  initialize("5");
  checkAcceptedTasks(1, 100, {true, false, true, false, true});
}

TEST_F(RosFixture, TwoThreadsWithMiss) {  // NOLINT
  initialize("6");
  checkAcceptedTasks(2, 60, {true, true, false, true, true, false});
}
