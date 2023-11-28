/*
 * @file publisher_member_function.cpp
 * @brief This file contains the implementation of the MinimalPublisher class
 * that includes custom service
 * @author Neha Nitin Madhekar
 * @date 2023
 * @copyright Open Source Robotics Foundation, Inc.
 * @license Apache License, Version 2.0
 *    (you may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0)
 *
 * This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer.
 */
#include <memory>
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <beginner_tutorials/srv/modify_service.hpp>
#include <chrono>
#include <functional>

#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

/**
 * @class MinimalPublisher
 * @brief This class represents a minimal ROS2 publisher. It also has service.
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for MinimalPublisher.
   */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    // Parameter for initializig publisher frequency
    auto pub_frequency_info = rcl_interfaces::msg::ParameterDescriptor();
    pub_frequency_info.description = "Custom frequency value for the publisher";

    // default frequency is 1.0
    this->declare_parameter("frequency", 0.5, pub_frequency_info);
    auto pub_frequency = this->get_parameter("frequency")
                             .get_parameter_value()
                             .get<std::float_t>();
    if (pub_frequency < 0) {
      RCLCPP_FATAL_STREAM_ONCE(rclcpp::get_logger("minimal_publisher"),
                               "Publisher Frequency Cannot be negative");
      exit(1);
    } else if (pub_frequency == 0) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("minimal_publisher"),
                          "Publisher Frequency set to zero");
    } else if (pub_frequency > 100) {
      RCLCPP_WARN_STREAM_ONCE(rclcpp::get_logger("minimal_publisher"),
                              "Publisher Frequency greater than hundread");
    } else {
      RCLCPP_DEBUG_STREAM(
          rclcpp::get_logger("minimal_publisher"),
          "Publisher Frequency parameter is " << pub_frequency << " Hz");

      RCLCPP_INFO_STREAM(rclcpp::get_logger("minimal_publisher"),
                         "Publishing at " << pub_frequency << " Hz");
    }

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    auto time =
        std::chrono::milliseconds(static_cast<int>(1000 / pub_frequency));

    timer_ = this->create_wall_timer(
        time, std::bind(&MinimalPublisher::timer_callback, this));

    // Creating a service object to get request and response
    auto serviceCallbackPtr =
        std::bind(&MinimalPublisher::modify_message, this,
                  std::placeholders::_1, std::placeholders::_2);

    service_ = create_service<beginner_tutorials::srv::ModifyService>(
        "custom_service", serviceCallbackPtr);

  }

  /**
   * @brief Callback function for modifying messages.
   * @param request The request message.
   * @param response The response message.
   */
  void modify_message(
      const std::shared_ptr<beginner_tutorials::srv::ModifyService::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::ModifyService::Response>
          response) {
    response->response_message =
        request->request_message + " This is modified message by service!!";
    RCLCPP_INFO_STREAM(rclcpp::get_logger("minimal_publisher"),
                       "Request message: " << request->request_message);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("minimal_publisher"),
                       "Response message: " << response->response_message);
  }


 private:
  /**
   * @brief Timer callback function.
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Welcome to ENPM808X ! ";
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
    make_transforms();
  }

  void make_transforms()
  {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";

    t.transform.translation.x = 1.0;
    t.transform.translation.y = 0.5;
    t.transform.translation.z = 0.0;
    
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    tf_static_broadcaster_->sendTransform(t);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ModifyService>::SharedPtr service_;
  size_t count_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

/**
 * @brief Main function for the ROS2 node.
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return Return code.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
