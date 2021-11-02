#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>

class ProcessNode : public rclcpp::Node
{
public: 
    ProcessNode()
    : Node("process_node")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image", 
                                                                        10, 
                                                                        std::bind(&ProcessNode::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_processed", 10);
    }

    ~ProcessNode()
    {
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if(msg->header.frame_id == "image1")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Process: callback for timer 1.");
        }
        else if(msg->header.frame_id == "image2")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Process: callback for timer 2.");
        }

        auto msg_out = sensor_msgs::msg::Image();
        filter->update(*msg, msg_out);
        pub_->publish(msg_out);
    }

private:
    // filters::FilterChain<sensor_msgs::msg::Image> filter_chain_;
    std::shared_ptr<filters::FilterBase<sensor_msgs::msg::Image>> filter;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};
