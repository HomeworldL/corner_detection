// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "math.h"

// using namespace std::chrono_literals;

class ProcessNode : public rclcpp::Node
{
public:
  ProcessNode()
  : Node("process_node")
  {
    callback_group_subscriber2_ = this->create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

    callback_group_subscriber1_ = this->create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    
    // callback_group_subscriber2_ = this->create_callback_group(
    //   rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

    // Each of these callback groups is basically a thread
    // Everything assigned to one of them gets bundled into the same thread
    auto sub2_opt = rclcpp::SubscriptionOptions();
    sub2_opt.callback_group = callback_group_subscriber2_;

    auto sub1_opt = rclcpp::SubscriptionOptions();
    sub1_opt.callback_group = callback_group_subscriber1_;
    
    // auto sub2_opt = rclcpp::SubscriptionOptions();
    // sub2_opt.callback_group = callback_group_subscriber2_;

    subscription2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan2",
      rclcpp::QoS(10),
      std::bind(
        &ProcessNode::subscriber2_cb,
        this,
        std::placeholders::_1),
      sub2_opt);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Subscriptor 2 generates successfully.");

    
    subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan1",
      rclcpp::QoS(10),
      // std::bind is sort of C++'s way of passing a function
      // If you're used to function-passing, skip these comments
      std::bind(
        &ProcessNode::subscriber1_cb,  // First parameter is a reference to the function
        this,                               // What the function should be bound to
        std::placeholders::_1),             // At this point we're not positive of all the
                                            // parameters being passed
                                            // So we just put a generic placeholder
                                            // into the binder
                                            // (since we know we need ONE parameter)
      sub1_opt);                  // This is where we set the callback group.
                                  // This subscription will run with callback group subscriber1

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Subscriptor 1 generates successfully.");  

    // subscription2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    //   "scan2",
    //   rclcpp::QoS(10),
    //   std::bind(
    //     &ProcessNode::subscriber2_cb,
    //     this,
    //     std::placeholders::_1),
    //   sub2_opt);

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Subscriptor 2 generates successfully.");

    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
  }

private:
  void subscriber1_cb(sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Process: callback for timer 1.");
    // convert the scan data from frame1 to the global frame
    int range_count = message->ranges.size();
    for(int i = 0; i < range_count; i++)
    {
      float x1 = message->ranges[i] * sin(i * message->angle_increment);
      float y1 = message->ranges[i] * cos(i * message->angle_increment);
      float converted_range = sqrt(pow(x1 + a1, 2) + pow(y1 + b1, 2));

      message->ranges[i] = converted_range;
      message->intensities[i] = converted_range;
    }

    // publish the converted scan data to topic /scan
    this->publisher_->publish(*message);
  }

  void subscriber2_cb(sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Process: callback for timer 2.");
    // convert the scan data from frame2 to the global frame
    int range_count = message->ranges.size();
    for(int i = 0; i < range_count; i++)
    {
      float x2 = message->ranges[i] * sin(i * message->angle_increment);
      float y2 = message->ranges[i] * cos(i * message->angle_increment);
      float converted_range = sqrt(pow(x2 + a2, 2) + pow(y2 + b2, 2));

      message->ranges[i] = converted_range;
      message->intensities[i] = converted_range;
    }

    // publish the converted scan data to topic /scan
    this->publisher_->publish(*message);
  }

  // the coordinates of frame1 and frame2 in the global frame
  float a1 = 0.3;
  float b1 = 0.6;
  float a2 = 0.8;
  float b2 = -0.5;

  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber1_;
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber2_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription2_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};
