#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class MappingNode: public rclcpp::Node
{
public:
    MappingNode()
    : Node("mapping_node")
    {
        // Initialise ROS publishers and subscribers
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan_interp_filtered", 
                                                                        10, 
                                                                        std::bind(&MappingNode::scan_callback, this, std::placeholders::_1));

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mapping node has been initialised.");
    }

    // Callback functions and relevant functions
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan = *msg;
    }

private:
    sensor_msgs::msg::LaserScan scan;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
};
