/*
 * @file talker_test.cpp
 * @brief This file contains the tests for talker node
 * @author Neha Nitin Madhekar
 * @date 2023
 * @copyright Open Source Robotics Foundation, Inc.
 * @license Apache License, Version 2.0
 *    (you may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0)
 *
 */
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>

#include <std_msgs/msg/string.hpp>

/**
 * @class TaskPlanningFixture
 * @brief Fixture for testing task talker node.
 *
 * This fixture sets up and tears down a ROS2 node for testing talker.
 * It provides methods for starting and stopping ROS2 executables, along with a test case
 * for checking if a talker publishes the expected message.
 */
class TaskPlanningFixture : public testing::Test {
public:
  TaskPlanningFixture() : node_(std::make_shared<rclcpp::Node>("basic_test")) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }

  void SetUp() override {
    // Setup things that should occur before every test instance should go here

    /*
     * 1.) Define any ros2 package and exectuable you want to test
     *  example: package name = cpp_pubsub, node name = minimal_publisher,
     * executable = talker
     */
    bool retVal =
        StartROSExec("beginner_tutorials", "minimal_publisher", "talker");
    ASSERT_TRUE(retVal);

    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH SETUP!!");
  }

  void TearDown() override {
    // Tear things that should occur after every test instance should go here

    // Stop the running ros2 node, if any.
    bool retVal = StopROSExec();
    ASSERT_TRUE(retVal);

    std::cout << "DONE WITH TEARDOWN" << std::endl;
  }

protected:
  rclcpp::Node::SharedPtr node_;
  std::stringstream cmd_ss, cmdInfo_ss, killCmd_ss;

  bool StartROSExec(const char *pkg_name, const char *node_name,
                    const char *exec_name) {
    // build command strings
    cmd_ss << "ros2 run " << pkg_name << " " << exec_name
           << " > /dev/null 2> /dev/null &";
    cmdInfo_ss << "ros2 node info "
               << "/" << node_name << " > /dev/null 2> /dev/null";
    char execName[16];
    snprintf(execName, 16, "%s",
             exec_name); // pkill uses exec name <= 15 char only
    killCmd_ss << "pkill --signal SIGINT " << execName
               << " > /dev/null 2> /dev/null";

    // First kill the ros2 node, in case it's still running.
    StopROSExec();

    // Start a ros2 node and wait for it to get ready:
    int retVal = system(cmd_ss.str().c_str());
    if (retVal != 0)
      return false;

    retVal = -1;
    while (retVal != 0) {
      retVal = system(cmdInfo_ss.str().c_str());
      sleep(1);
    }
    return true;
  }

  bool StopROSExec() {
    if (killCmd_ss.str().empty())
      return true;

    int retVal = system(killCmd_ss.str().c_str());
    return retVal == 0;
  }
};

/**
 * @brief Test case to check if the talker publishes the expected message.
 */
TEST_F(TaskPlanningFixture, testTalkerPublishesMessage) {
  const std::string expected_message =
      "Welcome to ENPM808X ! " ;

  auto subscriber = node_->create_subscription<std_msgs::msg::String>(
      "topic", 10,
      [expected_message](const std_msgs::msg::String::SharedPtr msg) {
        EXPECT_EQ(msg->data, expected_message);
      });
}

/**
 * @brief Main function to run all tests.
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}