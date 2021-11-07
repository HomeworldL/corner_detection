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
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace cv;
// using namespace std::chrono_literals;

class PorcessNode : public rclcpp::Node
{
public:
  PorcessNode()
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

    subscription2_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image2",
      rclcpp::QoS(10),
      std::bind(
        &PorcessNode::subscriber2_cb,
        this,
        std::placeholders::_1),
      sub2_opt);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Subscriptor 2 generates successfully.");

    
    subscription1_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image1",
      rclcpp::QoS(10),
      // std::bind is sort of C++'s way of passing a function
      // If you're used to function-passing, skip these comments
      std::bind(
        &PorcessNode::subscriber1_cb,  // First parameter is a reference to the function
        this,                               // What the function should be bound to
        std::placeholders::_1),             // At this point we're not positive of all the
                                            // parameters being passed
                                            // So we just put a generic placeholder
                                            // into the binder
                                            // (since we know we need ONE parameter)
      sub1_opt);                  // This is where we set the callback group.
                                  // This subscription will run with callback group subscriber1

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Subscriptor 1 generates successfully.");  

    // subscription2_ = this->create_subscription<sensor_msgs::msg::Image>(
    //   "image2",
    //   rclcpp::QoS(10),
    //   std::bind(
    //     &PorcessNode::subscriber2_cb,
    //     this,
    //     std::placeholders::_1),
    //   sub2_opt);

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Subscriptor 2 generates successfully.");

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
  }

private:
  void subscriber1_cb(sensor_msgs::msg::Image::SharedPtr message)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Process: callback for timer 1.");
    
    // Laplace Operator processes images from /image1
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(message, message->encoding);
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("cv_bridge exception 1: %s", e.what());
      return;
    }

    // Declare the variables we are going to use
    Mat src, src_gray, dst, abs_dst;
    int kernel_size = 3;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;

    // Reduce noise by blurring with a Gaussian filter ( kernel size = 3 )
    src = cv_ptr->image;
    GaussianBlur(src, src, Size(3, 3), 0, 0, BORDER_DEFAULT);
    cvtColor(src, src_gray, COLOR_BGR2GRAY); // Convert the image to grayscale
    
    Laplacian(src_gray, dst, ddepth, kernel_size, scale, delta, BORDER_DEFAULT);
    
    // converting back to CV_8U
    convertScaleAbs(dst, abs_dst);

    cv_ptr->image = abs_dst;

    // publish the upsampled images to topic /image
    this->publisher_->publish(cv_ptr->toImageMsg());
  }

  void subscriber2_cb(sensor_msgs::msg::Image::SharedPtr message)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Process: callback for timer 2.");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(message, message->encoding);
    }
    catch(const std::exception& e)
    {
      ROS_ERROR("cv_bridge exception 2: %s", e.what());
      return;
    }

    // First we declare the variables we are going to use
    Mat src, src_gray, grad;
    const String window_name = "Sobel Demo - Simple Edge Detector";
    int ksize = parser.get<int>("ksize");
    int scale = parser.get<int>("scale");
    int delta = parser.get<int>("delta");
    int ddepth = CV_16S;

    // Remove noise by blurring with a Gaussian filter ( kernel size = 3 )
    GaussianBlur(cv_ptr->image, src, Size(3, 3), 0, 0, BORDER_DEFAULT);
    
    // Convert the image to grayscale
    cvtColor(src, src_gray, COLOR_BGR2GRAY);
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    Sobel(src_gray, grad_x, ddepth, 1, 0, ksize, scale, delta, BORDER_DEFAULT);
    Sobel(src_gray, grad_y, ddepth, 0, 1, ksize, scale, delta, BORDER_DEFAULT);
    
    // converting back to CV_8U
    convertScaleAbs(grad_x, abs_grad_x);
    convertScaleAbs(grad_y, abs_grad_y);
    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

    cv_ptr->image = grad;

    // publish the downsampled images to topic /image
    this->publisher_->publish(cv_ptr->toImageMsg());
  }

  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber1_;
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber2_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription1_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription2_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};
