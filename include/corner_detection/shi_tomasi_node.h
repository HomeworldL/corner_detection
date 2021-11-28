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
                                                                        10, 
                                                                        std::bind(&ShiTomasiNode::image_callback, this, std::placeholders::_1));

        // Initialise outfile    
        delay_chain3.open(result_path + "delay_chain3.txt", ios::out);
        if (!delay_chain3.is_open()) 
        {
            cout<<"Error opening file delay_chain3.txt! "<<endl;
        }

        // set output accuracy
        delay_chain3.setf(ios::fixed, ios::floatfield);
        delay_chain3.precision(9);

        delay_chain4.open(result_path + "delay_chain4.txt", ios::out);
        if (!delay_chain4.is_open()) 
        {
            cout<<"Error opening file delay_chain4.txt! "<<endl;
        }

        // set output accuracy
        delay_chain4.setf(ios::fixed, ios::floatfield);
        delay_chain4.precision(9);

        count1 = 0;
        count2 = 0;
        addition1 = 0.;
        addition2 = 0.;

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Shi-Tomasi node has been initialised.");
    }

    ~ShiTomasiNode()
    {
        delay_chain3.close();
        delay_chain4.close();
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

        rclcpp::Time msg_time = msg->header.stamp;
        string frame_id = msg->header.frame_id;

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

        // if(count1 < 5)
        // {
        //     count1++;
        //     return;
        // }

        output_delay(msg_time, frame_id);
    }

    void output_delay(rclcpp::Time msg_time, string frame_id)
    {
        rclcpp::Time now = this->now();
        rclcpp::Duration delay = now - msg_time;
                
        if(frame_id == "image1")
        {
            if(count1 < 5)
            {
                count1++;
                return;
            }
            
            if(count1 < 505)
            {
                count1++;
                addition1 += delay.seconds();
                delay_chain3 << delay.seconds() << endl;
            }
            else if(count1 == 505)
            {
                delay_chain3 << endl << (double)(addition1 / (double)count1) << endl;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "We have got enough delay info from chain1.");
                count1++;
            }
        }
        else if(frame_id == "image2")
        {
            if(count2 < 5)
            {
                count2++;
                return;
            }

            if(count2 < 505)
            {
                count2++;
                addition2 += delay.seconds();
                delay_chain4 << delay.seconds() << endl;
            }
            else if(count2 == 505)
            {
                delay_chain4 << endl << (double)(addition2 / (double)count2) << endl;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "We have got enough delay info from chain2.");
                count2++;
            }
        }
    }

private:
    ofstream delay_chain3;
    ofstream delay_chain4;

    int count1, count2;
    double addition1, addition2;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

    string result_path = "/home/eric/eloquent_ws/src/corner_detection/results/";
};
