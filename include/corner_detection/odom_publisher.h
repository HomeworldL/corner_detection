// #include <chrono>
// #include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"

#include <random>
using namespace std;

// properties
string child_frame_id = "base_footprint";

class OdomPublisher : public rclcpp::Node
{
public:
  OdomPublisher()
  : Node("odom_publisher")
  {
    publisher = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    timer = this->create_wall_timer(
      20ms, std::bind(&OdomPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = nav_msgs::msg::Odometry();

    // mark the timestamp
    rclcpp::Time now = this->now();
    message.header.stamp = now;
    message.header.frame_id = "odom";
    
    // properties
    message.child_frame_id = child_frame_id;
    message.pose.pose.position.x = 1.5; // random changes
    message.pose.pose.position.y = 0.5; // random changes
    message.pose.pose.position.z = 0.08; // random changes
    message.pose.pose.orientation.x = 0.0003; // random changes
    message.pose.pose.orientation.y = 0.0006; // random changes
    message.pose.pose.orientation.z = -0.5; // random changes
    message.pose.pose.orientation.w = 0.8; // random changes

    // float64[36] covariance
    // Row-major representation of the 6x6 covariance matrix
    // The orientation parameters use a fixed-axis representation.
    // In order, the parameters are:
    // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
    for(int i = 0; i < 36; i++)
    {
      if(i == 0)
        message.pose.covariance[i] = 1.0e-05;
      else if(i == 7)
        message.pose.covariance[i] = 1.0e-05;
      else if(i == 14)
        message.pose.covariance[i] = 1000000000000.0;
      else if(i == 21)
        message.pose.covariance[i] = 1000000000000.0;
      else if(i == 28)
        message.pose.covariance[i] = 1000000000000.0;
      else if(i == 35)
        message.pose.covariance[i] = 0.001;
      else
        message.pose.covariance[i] = 0;
    }

    // Vector3  linear
    // Vector3  angular
    message.twist.twist.linear.x = 0.04; // random changes
    message.twist.twist.linear.y = 0.05; // random changes
    message.twist.twist.linear.z = 0.0;
    message.twist.twist.angular.x = 0.0;
    message.twist.twist.angular.y = 0.0;
    message.twist.twist.angular.z = -0.3; // random changes

    // float64[36] covariance
    for(int i = 0; i < 36; i++)
    {
      if(i == 0)
        message.twist.covariance[i] = 1.0e-05;
      else if(i == 7)
        message.twist.covariance[i] = 1.0e-05;
      else if(i == 14)
        message.twist.covariance[i] = 1000000000000.0;
      else if(i == 21)
        message.twist.covariance[i] = 1000000000000.0;
      else if(i == 28)
        message.twist.covariance[i] = 1000000000000.0;
      else if(i == 35)
        message.twist.covariance[i] = 0.001;
      else
        message.twist.covariance[i] = 0;
    }

    publisher->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher;
};