#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class HarrisNode: public rclcpp::Node
{
public:
    HarrisNode()
    : Node("harris_node")
    {
        // Initialise ROS publishers and subscribers
        image_sub = this->create_subscription<sensor_msgs::msg::Image>("/image_processed", 
                                                                        10, 
                                                                        std::bind(&HarrisNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Harris node has been initialised.");
    }

    // Callback functions and relevant functions
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        scan = *msg;
    }

private:
    sensor_msgs::msg::Image image;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
};
