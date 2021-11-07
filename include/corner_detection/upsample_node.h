/***************
 * 
 * Ref: https://docs.opencv.org/3.4.15/d4/d1f/tutorial_pyramids.html
 * 
 *
 ***************/
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace cv;

class UpsampleNode : public rclcpp::Node
{
public: 
    UpsampleNode()
    : Node("upsample_node")
    {
        sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image", 
                                                                        10, 
                                                                        std::bind(&UpsampleNode::callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/image_upsampled", 10);
    }

    ~UpsampleNode()
    {
    }

    void callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if(msg->header.frame_id == "image1")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Upsample: callback for timer 1.");
        }
        else if(msg->header.frame_id == "image2")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Upsample: callback for timer 2.");
        }
    
        // upsample images from /image1
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch(const std::exception& e)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "cv_bridge exception upsample node: %s", e.what());
            return;
        }
        
        // upsample cv_ptr->image
        Mat src = cv_ptr->image;
        pyrUp(src, src, Size(src.cols * 2, src.rows * 2));
        cv_ptr->image = src;

        // publish the upsampled images to topic /image_upsampled
        pub_->publish(*cv_ptr->toImageMsg());
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};
