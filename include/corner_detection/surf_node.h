#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace cv::xfeatures2d;

class SurfNode: public rclcpp::Node
{
public:
    SurfNode()
    : Node("surf_node")
    {
        // Initialise ROS publishers and subscribers
        image_sub = this->create_subscription<sensor_msgs::msg::Image>("/image_upsampled", 
                                                                        10, 
                                                                        std::bind(&SurfNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Surf node has been initialised.");
    }

    // Callback functions and relevant functions
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {        
        if(msg->header.frame_id == "image1")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Surf: callback for timer 1.");
        }
        else if(msg->header.frame_id == "image2")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Surf: callback for timer 2.");
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
        
        //-- Detect the keypoints using SURF Detector
        int minHessian = 400;
        Ptr<SURF> detector = SURF::create(minHessian);
        std::vector<KeyPoint> keypoints;

        Mat src = cv_ptr->image;
        cvtColor(cv_ptr->image, src_gray, COLOR_BGR2GRAY);
        detector->detect(src_gray, keypoints);

        //-- Draw keypoints
        Mat img_keypoints;
        drawKeypoints(src, keypoints, img_keypoints);

        //-- Show detected (drawn) keypoints
        imshow("SURF Keypoints", img_keypoints);
        waitKey();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
};
