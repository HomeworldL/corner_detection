#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <cstring> 
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <string>

using namespace std;
using namespace cv;

class TimerPublisher : public rclcpp::Node
{
public:
  TimerPublisher()
  : Node("timer_publisher")
  {
    // publisher_2 = this->create_publisher<sensor_msgs::msg::Image>("/image2", 10);
    publisher_1 = this->create_publisher<sensor_msgs::msg::Image>("/image1", 10);
    publisher_2 = this->create_publisher<sensor_msgs::msg::Image>("/image2", 10);

    // timer_2 = this->create_wall_timer(
    //   100ms, std::bind(&TimerPublisher::timer2_callback, this));

    timer_1 = this->create_wall_timer(
      100ms, std::bind(&TimerPublisher::timer1_callback, this));

    timer_2 = this->create_wall_timer(
      100ms, std::bind(&TimerPublisher::timer2_callback, this));

    // load images with timestamps
    cout << " Loading images.... It may take some time." << endl;
    LoadImages(image_file, vstrImages);
    
    count_1 = 0;
    count_2 = 0;
  }

private:
  void timer1_callback()
  {
    if(count_1 <= 1200)
    {
      Mat image;
      image = imread(image_path + vstrImages[count_1], CV_LOAD_IMAGE_GRAYSCALE);
      count_1++;
      
      cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

      rclcpp::Time time = this->now();
      cv_ptr->encoding = "mono8";
      cv_ptr->header.stamp = time;
      cv_ptr->header.frame_id = "image1";

      cv_ptr->image = image;
      image.release();

      publisher_1->publish(*cv_ptr->toImageMsg());
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Timer: publishing image from timer 1.");
    }
    else
    {
      count_1 = 0;
    }
  }

  void timer2_callback()
  {
    if(count_2 <= 1200)
    {
      Mat image;
      image = imread(image_path + vstrImages[count_2], CV_LOAD_IMAGE_GRAYSCALE);
      count_2++;
      
      cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

      rclcpp::Time time = this->now();
      cv_ptr->encoding = "mono8";
      cv_ptr->header.stamp = time;
      cv_ptr->header.frame_id = "image2";

      cv_ptr->image = image;
      image.release();

      publisher_2->publish(*cv_ptr->toImageMsg());
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Timer: publishing image from timer 2.");
    }
    else
    {
      count_2 = 0;
    }
  }

  void LoadImages(const string &strFilename, vector<string> &vstrImage)
  {
      ifstream file_handler;
      file_handler.open(strFilename.c_str());
      while(!file_handler.eof())
      {
          string s;

          getline(file_handler,s);
          if(!s.empty())
          {
              stringstream ss;
              ss << s;
              double t;
              ss >> t;
              string sRGB;
              ss >> sRGB;
              vstrImage.push_back(sRGB);
          }
      }
  }

  int count_1, count_2;
  vector<string> vstrImages;

  string image_file = "/media/eric/ubuntu/DATA/Event/ETH/shapes_translation/Text/shapes_translation/images.txt";
  string image_path = "/media/eric/ubuntu/DATA/Event/ETH/shapes_translation/Text/shapes_translation/";

  rclcpp::TimerBase::SharedPtr timer_1;
  rclcpp::TimerBase::SharedPtr timer_2;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_1;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_2;
};
