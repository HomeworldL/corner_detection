/***************
 * 
 * Ref: https://docs.opencv.org/3.4.15/d7/da8/tutorial_table_of_content_imgproc.html
 * 
 *
 ***************/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace cv;

class ProcessNode : public rclcpp::Node
{
public:
  ProcessNode()
  : Node("process_node")
  {
    // callback_group_subscriber2_ = this->create_callback_group(
    //   rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

    callback_group_subscriber1_ = this->create_callback_group(
      rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    
    callback_group_subscriber2_ = this->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

    // auto sub2_opt = rclcpp::SubscriptionOptions();
    // sub2_opt.callback_group = callback_group_subscriber2_;

    auto sub1_opt = rclcpp::SubscriptionOptions();
    sub1_opt.callback_group = callback_group_subscriber1_;
    
    auto sub2_opt = rclcpp::SubscriptionOptions();
    sub2_opt.callback_group = callback_group_subscriber2_;

    // subscription2_ = this->create_subscription<sensor_msgs::msg::Image>(
    //   "image2",
    //   rclcpp::QoS(10),
    //   std::bind(
    //     &ProcessNode::subscriber2_cb,
    //     this,
    //     std::placeholders::_1),
    //   sub2_opt);

    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Subscriptor 2 generates successfully.");

    
    subscription1_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image1",
      rclcpp::QoS(10),
      std::bind(
        &ProcessNode::subscriber1_cb,
        this,
        std::placeholders::_1),
      sub1_opt);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Subscriptor 1 generates successfully.");  

    subscription2_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image2",
      rclcpp::QoS(10),
      std::bind(
        &ProcessNode::subscriber2_cb,
        this,
        std::placeholders::_1),
      sub2_opt);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Subscriptor 2 generates successfully.");

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
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "cv_bridge exception 1: %s", e.what());
      return;
    }

    // Declare the variables we are going to use
    Mat src_gray, dst, abs_dst;
    int kernel_size = 3;
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;

    // Reduce noise by blurring with a Gaussian filter ( kernel size = 3 )
    src_gray = cv_ptr->image;
    GaussianBlur(src_gray, src_gray, Size(3, 3), 0, 0, BORDER_DEFAULT);
    // cvtColor(src, src_gray, COLOR_BGR2GRAY); // Convert the image to grayscale
    
    Laplacian(src_gray, dst, ddepth, kernel_size, scale, delta, BORDER_DEFAULT);
    
    // converting back to CV_8U
    convertScaleAbs(dst, abs_dst);

    cv_ptr->image = abs_dst;

    // publish the upsampled images to topic /image
    this->publisher_->publish(*cv_ptr->toImageMsg());
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
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "cv_bridge exception process node: %s", e.what());
      return;
    }

    Mat src_gray, grad;
    int ddepth = CV_16S;

    // Remove noise by blurring with a Gaussian filter ( kernel size = 3 )
    GaussianBlur(cv_ptr->image, src_gray, Size(3, 3), 0, 0, BORDER_DEFAULT);
    
    // Convert the image to grayscale
    // cvtColor(src, src_gray, COLOR_BGR2GRAY);
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
    Sobel(src_gray, grad_x, ddepth, 1, 0);
    Sobel(src_gray, grad_y, ddepth, 0, 1);
    
    // converting back to CV_8U
    convertScaleAbs(grad_x, abs_grad_x);
    convertScaleAbs(grad_y, abs_grad_y);
    addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

    cv_ptr->image = grad;

    // publish the downsampled images to topic /image
    this->publisher_->publish(*cv_ptr->toImageMsg());
  }

  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber1_;
  rclcpp::callback_group::CallbackGroup::SharedPtr callback_group_subscriber2_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription1_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription2_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};
