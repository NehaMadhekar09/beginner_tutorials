// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <beginner_tutorials/srv/modify_service.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
 public:
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
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("minimal_publisher"),
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

  void modify_message(
      const std::shared_ptr<beginner_tutorials::srv::ModifyService::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::ModifyService::Response>
          response) {
    response->response_message =
        request->request_message + "This is modified service!!";
    RCLCPP_INFO_STREAM(rclcpp::get_logger("minimal_publisher"),
                       "Request message: " << request->request_message);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("minimal_publisher"),
                       "Response message: " << response->response_message);
  }


 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Welcome to ENPM808X ! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ModifyService>::SharedPtr service_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
