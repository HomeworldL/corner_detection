#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class FastNode: public rclcpp::Node
{
public:
    FastNode()
    : Node("fast_node")
    {
        // Initialise ROS publishers and subscribers
        image_sub = this->create_subscription<sensor_msgs::msg::Image>("/image_processed", 
                                                                        10, 
                                                                        std::bind(&FastNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Fast node has been initialised.");
    }

    // Callback functions and relevant functions
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        image = *msg;
    }

private:
    sensor_msgs::msg::Image image;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
};
