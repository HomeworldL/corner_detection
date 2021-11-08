/***************
 * 
 * Ref: https://docs.opencv.org/3.4.15/d4/d7d/tutorial_harris_detector.html
 * 
 *
 ***************/
#include <vector>
#include <iostream>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

using namespace cv;

string result_path = "~/crystal_ws/src/corner_detection/results/";

class HarrisNode: public rclcpp::Node
{
public:
    HarrisNode()
    : Node("harris_node")
    {
        // Initialise ROS publishers and subscribers
        image_sub = this->create_subscription<sensor_msgs::msg::Image>("/image_upsampled", 
                                                                        std::bind(&HarrisNode::image_callback, this, std::placeholders::_1),
                                                                        10);

        // Initialise outfile    
        delay_outfile_1.open(result_path + "delay_chain1.txt", ios::out);
        if (!delay_outfile_1.is_open()) 
        {
            cout<<"Error opening file delay_outfile_1.txt! "<<endl;
        }

        // set output accuracy
        delay_outfile_1.setf(ios::fixed, ios::floatfield);
        delay_outfile_1.precision(9);

        delay_outfile_2.open(result_path + "delay_chain2.txt", ios::out);
        if (!delay_outfile_2.is_open()) 
        {
            cout<<"Error opening file delay_outfile_2.txt! "<<endl;
        }

        // set output accuracy
        delay_outfile_2.setf(ios::fixed, ios::floatfield);
        delay_outfile_2.precision(9);

        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Harris node has been initialised.");
    }

    ~HarrisNode()
    {
        delay_outfile_1.close();
        delay_outfile_2.close();
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
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "cv_bridge exception harris node: %s", e.what());
            return;
        }

        rclcpp::Time msg_time = msg->header.stamp;
        string frame_id = msg->header.frame_id;
        
        Mat src_gray = cv_ptr->image;
        int blockSize = 2;
        int apertureSize = 3;
        int thresh = 200;
        double k = 0.04;
        Mat dst_norm, dst_norm_scaled;

        Mat dst = Mat::zeros(src_gray.size(), CV_32FC1);

        // cvtColor(src, src_gray, COLOR_BGR2GRAY);
        cornerHarris(src_gray, dst, blockSize, apertureSize, k);
        
        normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
        convertScaleAbs(dst_norm, dst_norm_scaled);
        // for(int i = 0; i < dst_norm.rows ; i++)
        // {
        //     for(int j = 0; j < dst_norm.cols; j++)
        //     {
        //         if((int) dst_norm.at<float>(i,j) > thresh)
        //         {
        //             circle(dst_norm_scaled, Point(j,i), 5,  Scalar(0), 2, 8, 0);
        //         }
        //     }
        // }
        
        // imshow("Harris Keypoints", dst_norm_scaled);
        // waitKey();
        
        if(count1 < 5)
        {
            count1++;
            return;
        }

        output_delay(msg_time, frame_id);
    }

    void output_delay(rclcpp::Time msg_time, string frame_id)
    {
        rclcpp::Time now = this->now();
        rclcpp::Duration delay = now - msg_time;
                
        if(frame_id == "image1")
        {
            if(count1 < 500)
            {
                count1++;
                addition1 += delay.seconds();
                delay_outfile_1 << delay.seconds() << endl;
            }
            else if(count1 == 500)
            {
                delay_outfile_1 << endl << (double)(addition1 / (double)count1) << endl;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "We have got enough delay info from chain1.");
                count1++;
            }
        }
        else if(frame_id == "image2")
        {
            if(count2 < 500)
            {
                count2++;
                addition2 += delay.seconds();
                delay_outfile_2 << delay.seconds() << endl;
            }
            else if(count2 == 500)
            {
                delay_outfile_2 << endl << (double)(addition2 / (double)count2) << endl;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "We have got enough delay info from chain2.");
                count2++;
            }
        }
    }

private:
    ofstream delay_outfile_1;
    ofstream delay_outfile_2;

    int count1, count2;
    double addition1, addition2;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
};
