#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// #include <sensor_msgs/msg/joint_state.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"

#include <random>
using namespace std;

// using namespace std::chrono_literals;

// properties
float angle_min = 0.0;
float angle_max = 6.283185307179586;
float angle_increment = 0.017453292519943297;
// float angle_max = 6.28000020980835;
// float angle_increment = 0.01749303564429283;
float time_increment = 0.0;
float scan_time = 0.0;
float range_min = 0.12;
float range_max = 3.5;

class TimerPublisher : public rclcpp::Node
{
public:
  TimerPublisher()
  : Node("timer_publisher")
  {
    // publisher_2 = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan2", 10);
    publisher_1 = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan1", 10);
    publisher_2 = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan2", 10);

    // timer_2 = this->create_wall_timer(
    //   5ms, std::bind(&TimerPublisher::timer2_callback, this));

    timer_1 = this->create_wall_timer(
      100ms, std::bind(&TimerPublisher::timer1_callback, this));

    timer_2 = this->create_wall_timer(
      100ms, std::bind(&TimerPublisher::timer2_callback, this));
  }

private:
  void timer1_callback()
  {
    auto message = sensor_msgs::msg::LaserScan();

    // mark the timestamp
    rclcpp::Time now = this->now();
    message.header.stamp = now;
    message.header.frame_id = "base_scan1";
    
    // properties
    message.angle_min = angle_min;
    message.angle_max = angle_max;
    message.angle_increment = angle_increment;
    message.time_increment = time_increment;
    message.scan_time = scan_time;
    message.range_min = range_min;
    message.range_max = range_max;

    // generate random ranges between range_min and range_max
    int range_count = (int)angle_max / angle_increment;
    for(int i = 0; i < range_count; i++)
    {
      float random_value = RandomFloat(range_min, range_max);
      message.ranges.push_back(random_value);
      message.intensities.push_back(random_value);
    }

    publisher_1->publish(message);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Timer: publishing msg from timer 1.");
  }

  void timer2_callback()
  {
    auto message = sensor_msgs::msg::LaserScan();
    
    // mark the timestamp
    rclcpp::Time now = this->now();
    message.header.stamp = now;
    message.header.frame_id = "base_scan2";
    
    // properties
    message.angle_min = angle_min;
    message.angle_max = angle_max;
    message.angle_increment = angle_increment;
    message.time_increment = time_increment;
    message.scan_time = scan_time;
    message.range_min = range_min;
    message.range_max = range_max;

    // generate random ranges between range_min and range_max
    int range_count = (int)angle_max / angle_increment;
    for(int i = 0; i < range_count; i++)
    {
      float random_value = RandomFloat(range_min, range_max);
      message.ranges.push_back(random_value);
      message.intensities.push_back(random_value);
    }

    publisher_2->publish(message);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Timer: publishing msg from timer 2.");
  }

  float RandomFloat(float a, float b) 
  {
    assert(b > a); 
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = b - a;
    float r = random * diff;
    return a + r;
  }

  rclcpp::TimerBase::SharedPtr timer_1;
  rclcpp::TimerBase::SharedPtr timer_2;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_1;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_2;
};
