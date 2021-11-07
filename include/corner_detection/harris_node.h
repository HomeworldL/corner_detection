#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace cv;


class HarrisNode: public rclcpp::Node
{
public:
    HarrisNode()
    : Node("harris_node")
    {
        // Initialise ROS publishers and subscribers
        image_sub = this->create_subscription<sensor_msgs::msg::Image>("/image_upsampled", 
                                                                        10, 
                                                                        std::bind(&HarrisNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Harris node has been initialised.");
    }

    // Callback functions and relevant functions
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if(msg->header.frame_id == "image1")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Harris: callback for timer 1.");
        }
        else if(msg->header.frame_id == "image2")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Harris: callback for timer 2.");
        }
        
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("cv_bridge exception 1: %s", e.what());
            return;
        }
        
        Mat src = cv_ptr->image;
        int blockSize = 2;
        int apertureSize = 3;
        double k = 0.04;
        Mat dst_norm, dst_norm_scaled;

        Mat dst = Mat::zeros(src.size(), CV_32FC1);

        cvtColor(src, src_gray, COLOR_BGR2GRAY);
        cornerHarris(src_gray, dst, blockSize, apertureSize, k);
        
        normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
        convertScaleAbs(dst_norm, dst_norm_scaled);
        for(int i = 0; i < dst_norm.rows ; i++)
        {
            for(int j = 0; j < dst_norm.cols; j++)
            {
                if((int) dst_norm.at<float>(i,j) > thresh)
                {
                    circle(dst_norm_scaled, Point(j,i), 5,  Scalar(0), 2, 8, 0);
                }
            }
        }
        
        imshow("Harris Keypoints", dst_norm_scaled);
        waitKey();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
};
