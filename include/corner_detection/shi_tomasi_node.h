/***************
 * 
 * Ref: https://docs.opencv.org/3.4.15/d8/dd8/tutorial_good_features_to_track.html
 * 
 *
 ***************/  
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace cv;

class ShiTomasiNode: public rclcpp::Node
{
public:
    ShiTomasiNode()
    : Node("shi_tomasi_node")
    {
        // Initialise ROS publishers and subscribers
        image_sub = this->create_subscription<sensor_msgs::msg::Image>("/image_upsampled", 
                                                                        std::bind(&ShiTomasiNode::image_callback, this, std::placeholders::_1),
                                                                        10);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shi-Tomasi node has been initialised.");
    }

    // Callback functions and relevant functions
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {        
        if(msg->header.frame_id == "image1")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Shi_Tomasi: callback for timer 1.");
        }
        else if(msg->header.frame_id == "image2")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Shi_Tomasi: callback for timer 2.");
        }
        
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        }
        catch(const std::exception& e)
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "cv_bridge exception shi-tomasi node: %s", e.what());
            return;
        }

        Mat src_gray = cv_ptr->image;
        int maxCorners = 23;
        vector<Point2f> corners;
        double qualityLevel = 0.01;
        double minDistance = 10;
        int blockSize = 3;
        bool useHarrisDetector = false;
        double k = 0.04;
        Mat copy = src_gray.clone();
        RNG rng(12345);

        // cvtColor(src_gray, src_gray, COLOR_BGR2GRAY);
        
        goodFeaturesToTrack(src_gray,
                        corners,
                        maxCorners,
                        qualityLevel,
                        minDistance,
                        Mat(),
                        blockSize,
                        useHarrisDetector,
                        k);

        //-- Draw keypoints
        // int radius = 4;
        // for( size_t i = 0; i < corners.size(); i++ )
        // {
        //     circle(copy, corners[i], radius, Scalar(rng.uniform(0,255), rng.uniform(0, 256), rng.uniform(0, 256)), FILLED);
        // }

        //-- Show detected (drawn) keypoints
        // imshow("Shi-Tomasi Keypoints", copy);
        // waitKey();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
};
