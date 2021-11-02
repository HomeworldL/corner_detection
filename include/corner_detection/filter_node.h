#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <filters/filter_base.hpp>
#include <laser_filters/laser_filters.hpp>
#include <pluginlib/class_loader.hpp>

class FilterNode : public rclcpp::Node
{
public: 
    FilterNode()
    : Node("filter_node")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 
                                                                        10, 
                                                                        std::bind(&FilterNode::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_interp_filtered", 10);
        
        pluginlib::ClassLoader<filters::FilterBase<sensor_msgs::msg::LaserScan>> filter_loader("filters", "filters::FilterBase<sensor_msgs::msg::LaserScan>");
        filter = filter_loader.createSharedInstance("laser_filters::InterpolationFilter");

        filter->configure(
        "config", "InterpolationFilter",
        this->get_node_logging_interface(), this->get_node_parameters_interface());
    }

    ~FilterNode()
    {
    }

    void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if(msg->header.frame_id == "base_scan1")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Filter: callback for timer 1.");
        }
        else if(msg->header.frame_id == "base_scan2")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Filter: callback for timer 2.");
        }

        auto msg_out = sensor_msgs::msg::LaserScan();
        filter->update(*msg, msg_out);
        pub_->publish(msg_out);
    }

private:
    // filters::FilterChain<sensor_msgs::msg::LaserScan> filter_chain_;
    std::shared_ptr<filters::FilterBase<sensor_msgs::msg::LaserScan>> filter;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};
