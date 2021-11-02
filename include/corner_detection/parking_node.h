#include <math.h>
#include <vector>
#include <iostream>
#include <algorithm> // std::min_element
#include <iterator>  // std::begin, std::end
#include <unistd.h>  // sleep
// #if __has_include(<format>)
// #include <format>
// #endif

// #ifdef __cpp_lib_format
// // Code with std::format
// #else
// // Code without std::format, or just #error if you only
// // want to support compilers and standard libraries with std::format
// #endif
// #include <fmt/core.h>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std;

#define _USE_MATH_DEFINES

struct Point
{
    float x;
    float y;
};

struct AngleDistance
{
    int angle;
    float distance;
};

string result_path = "/home/eric/eloquent_ws/src/dag_case/results/";

class ParkingNode: public rclcpp::Node
{
public:
    ParkingNode()
    : Node("parking_node")
    {
        // Initialise variables
        odom = nav_msgs::msg::Odometry();
        last_pose_x = 0.0;
        last_pose_y = 0.0;
        last_pose_theta = 0.0;
        goal_pose_x = 0.0;
        goal_pose_y = 0.0;
        goal_pose_theta = 0.0;
        step = 0;
        // scan = []
        // rotation_point = []
        // center_point = []
        theta = 0.0;
        yaw = 0.0;
        get_key_state = false;
        init_scan_state = false;  // To get the initial scan at the beginning
        init_odom_state = false;   // To get the initial odom at the beginning

        count1 = 0;
        count2 = 0;
        addition1 = 0;
        addition2 = 0;

        // Initialise ROS publishers and subscribers
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan_interp_filtered", 
                                                                        10, 
                                                                        std::bind(&ParkingNode::scan_callback, this, std::placeholders::_1));
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 
                                                                        10, 
                                                                        std::bind(&ParkingNode::odom_callback, this, std::placeholders::_1));

        reset_pub = this->create_publisher<std_msgs::msg::Empty>("/reset", 10);
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        scan_spot_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_spot", 10);

        // Initialise timers
        update_timer = this->create_wall_timer(10ms,
            std::bind(&ParkingNode::update_callback, this));
            
        
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
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parking node has been initialised.");
    }
    
    ~ParkingNode()
    {
        delay_outfile_1.close();
        delay_outfile_2.close();
    }

    // Callback functions and relevant functions
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        scan = *msg;
        output_delay();
        
        init_scan_state = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom = *msg;
        last_pose_x = msg->pose.pose.position.x;
        last_pose_y = msg->pose.pose.position.y;

        float roll, pitch;
        euler_from_quaternion(msg->pose.pose.orientation, roll, pitch, last_pose_theta);
        yaw = last_pose_theta;
        init_odom_state = true;
    }

    void update_callback()
    {
        // if(init_scan_state && init_odom_state)
            // park_robot();
    }

    void output_delay()
    {
        rclcpp::Time now = this->now();
        rclcpp::Duration delay = now - scan.header.stamp;
                
        if(scan.header.frame_id == "base_scan1")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Parking: callback for timer 1.");
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
        else if(scan.header.frame_id == "base_scan2")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Node Parking: callback for timer 2.");
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

    void park_robot()
    {
        bool scan_done;
        int center_angle, start_angle, end_angle;
        scan_parking_spot(scan_done, center_angle, start_angle, end_angle);
        auto twist = geometry_msgs::msg::Twist();
        auto reset = std_msgs::msg::Empty();

        // Step 0: Find a parking spot
        if(this->step == 0)
        {
            if(scan_done == true)
            {
                bool fining_spot;
                Point start_point, end_point;
                find_parking_spot(center_angle, start_angle, end_angle, fining_spot, start_point, end_point);
                
                if(fining_spot == true)
                {
                    this->theta = atan2(start_point.y - end_point.y, start_point.x - end_point.x);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "=================================");
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "|        |     x     |     y     |");
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), '| start  | {0:>10.3f}| {1:>10.3f}|'.format(start_point[0], start_point[1]));
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), '| center | {0:>10.3f}| {1:>10.3f}|'.format(this->center_point[0], this->center_point[1]));
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), '| end    | {0:>10.3f}| {1:>10.3f}|'.format(end_point[0], end_point[1]));
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), fmt::format("| start  | {0:>10.3f}| {1:>10.3f}|", start_point.x, start_point.y));
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), fmt::format("| center | {0:>10.3f}| {1:>10.3f}|", this->center_point.x, this->center_point.y));
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), fmt::format("| end    | {0:>10.3f}| {1:>10.3f}|", end_point.x, end_point.y));
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "=================================");
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), '| theta  | {0:.2f} deg'.format(this->theta * M_PI / 180));
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), '| yaw    | {0:.2f} deg'.format(this->yaw * M_PI / 180));
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), fmt::format("| theta  | {0:.2f} deg", this->theta * M_PI / 180));
                    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), fmt::format("| yaw    | {0:.2f} deg", this->yaw * M_PI / 180));
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "=================================");
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "===== Go to parking spot!!! =====");
                    this->step = 1;
                }
            }
            else
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Fail finding parking spot.");
        }// Step 1: Turn
        else if(this->step == 1)
        {
            float init_yaw = this->yaw;
            this->yaw = this->theta + this->yaw;
            if(this->theta > 0)
            {
                if(this->theta - init_yaw > 0.1)
                {
                    twist.linear.x = 0.0;
                    twist.angular.z = 0.2;
                }
                else
                {
                    twist.linear.x = 0.0;
                    twist.angular.z = 0.0;
                    cmd_vel_pub->publish(twist);
                    sleep(1);
                    reset_pub->publish(reset);
                    sleep(3);
                    this->rotation_point = rotate_origin_only(this->center_point.x, this->center_point.y, -(M_PI / 2 - init_yaw));
                    this->step = 2;
                }
            }
            else
            {
                if(this->theta - init_yaw < -0.1)
                {
                    twist.linear.x = 0.0;
                    twist.angular.z = -0.2;
                }
                else
                {
                    twist.linear.x = 0.0;
                    twist.angular.z = 0.0;
                    cmd_vel_pub->publish(twist);
                    sleep(1);
                    reset_pub->publish(reset);
                    sleep(3);
                    this->rotation_point = rotate_origin_only(this->center_point.x, this->center_point.y, -(M_PI / 2 - init_yaw));
                    this->step = 2;
                }
            }
        } // Step 2: Move straight
        else if(step == 2)
        {
            if(abs(this->odom.pose.pose.position.x - this->rotation_point.y) > 0.02)
            {
                if(this->odom.pose.pose.position.x > this->rotation_point.y)
                {
                    twist.linear.x = -0.05;
                    twist.angular.z = 0.0;
                }
                else
                {
                    twist.linear.x = 0.05;
                    twist.angular.z = 0.0;
                }
            }
            else
            {
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                this->step = 3;
            }
        } // Step 3: Turn
        else if(step == 3)
        {
            if(this->yaw > -M_PI / 2)
            {
                twist.linear.x = 0.0;
                twist.angular.z = -0.2;
            }
            else
            {
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                this->step = 4;
            }
        } // Step 4: Move Straight
        else if(step == 4)
        {
            vector<float> ranges;
            for(int i = 150; i < 210; i++)
            {
                if(this->scan.ranges[i] != 0)
                    ranges.push_back(this->scan.ranges[i]);
            }
            
            std::vector<float>::iterator min_range = std::min_element(std::begin(ranges), std::end(ranges));
            if(*min_range > 0.2)
            {
                twist.linear.x = -0.04;
                twist.angular.z = 0.0;
            }
            else
            {
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Automatic parking done.");
                cmd_vel_pub->publish(twist);
                return;
            }
        }

        cmd_vel_pub->publish(twist);
        scan_spot_filter(center_angle, start_angle, end_angle);
    }

    void scan_parking_spot(bool &scan_done, int &center_angle, int &start_angle, int &end_angle)
    {
        scan_done = false;
        vector<int> intensity_index;
        vector<int> index_count;
        vector<int> spot_angle_index;
        int minimun_scan_angle = 30;
        int maximun_scan_angle = 330;
        int intensity_threshold = 100;

        for(int i = 0; i < 360; i++)
        {
            if(i >= minimun_scan_angle && i < maximun_scan_angle)
            {
                float spot_intensity = pow(scan.intensities[i], 2) * scan.ranges[i] / 100000;
                if(spot_intensity >= intensity_threshold)
                {
                    intensity_index.push_back(i);
                    index_count.push_back(i);
                }
                else
                {
                    intensity_index.push_back(0);
                }
            }
            else
                intensity_index.push_back(0);
        }

        int len_index_count = index_count.size();
        for(int i = 0; i < len_index_count; i++)
        {
            if(abs(index_count[i] - index_count[(int)(len_index_count / 2)]) < 20)
            {
                spot_angle_index.push_back(index_count[i]);
                int len_of_spot_angle_index = spot_angle_index.size();
                if(len_of_spot_angle_index > 10)
                {
                    scan_done = true;
                    center_angle = spot_angle_index[(int)(len_of_spot_angle_index / 2)];
                    start_angle = spot_angle_index[2];
                    end_angle = spot_angle_index[len_of_spot_angle_index - 3];
                }
                else
                    scan_done = false;
            }
        }
    }

    AngleDistance get_angle_distance(int angle)
    {
        float distance = scan.ranges[(int)angle];
        // if(scan.ranges[(int)angle] != NULL && distance != 0.0)
        if(distance != 0.0)
        {
            angle = int(angle);
            distance = distance;
        }

        struct AngleDistance ad;
        ad.angle = angle;
        ad.distance = distance;
        return ad;
    }

    Point get_point(AngleDistance start_angle_distance)
    {
        float angle = start_angle_distance.angle;
        // radians = (degrees * M_PI ) / 180 ;
        angle = (angle - 180) * M_PI / 180;
        float distance = start_angle_distance.distance;
        float x, y;

        if(angle >= 0 && angle < M_PI / 2)
        {
            x = distance * cos(angle) * -1;
            y = distance * sin(angle) * -1;
        }
        else if(angle >= M_PI / 2 && angle < M_PI)
        {
            x = distance * cos(angle) * -1;
            y = distance * sin(angle) * -1;
        }
        else if(angle >= -M_PI / 2 && angle < 0)
        {
            x = distance * cos(angle) * -1;
            y = distance * sin(angle) * -1;
        }
        else
        {
            x = distance * cos(angle) * -1;
            y = distance * sin(angle) * -1;
        }

        struct Point p;
        p.x = x;
        p.y = y;

        return p;
    }

    void find_parking_spot(int center_angle, int start_angle, int end_angle, bool &fining_spot, Point &start_point, Point &end_point)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "scan parking spot done!");
        fining_spot = false;
        AngleDistance start_angle_distance = get_angle_distance(start_angle);
        AngleDistance center_angle_distance = get_angle_distance(center_angle);
        AngleDistance end_angle_distance = get_angle_distance(end_angle);

        if(start_angle_distance.distance != 0 && center_angle_distance.distance != 0 && end_angle_distance.distance != 0) 
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "calibration......");
            start_point = get_point(start_angle_distance);
            this->center_point = get_point(center_angle_distance);
            end_point = get_point(end_angle_distance);
            fining_spot = true;
        }
        else
        {
            fining_spot = false;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "wrong scan!!");
        }
    }

    Point rotate_origin_only(float x, float y, float radians)
    {
        float xx = x * cos(radians) + y * sin(radians);
        float yy = -x * sin(radians) + y * cos(radians);
        
        struct Point p;
        p.x = xx;
        p.y = yy;

        return p;
    }

    void scan_spot_filter(int center_angle, int start_angle, int end_angle)
    {
        sensor_msgs::msg::LaserScan scan_spot = this->scan;

        vector<float> scan_spot_list = scan_spot.intensities;
        for(int i = 0; i < 360; i++)
            scan_spot_list[i] = 0.0;
        scan_spot_list[start_angle] = this->scan.ranges[start_angle] + 10000;
        scan_spot_list[center_angle] = this->scan.ranges[center_angle] + 10000;
        scan_spot_list[end_angle] = this->scan.ranges[end_angle] + 10000;
        scan_spot.intensities = scan_spot_list;

        scan_spot_pub->publish(scan_spot);
    }

    void euler_from_quaternion(geometry_msgs::msg::Quaternion quat, float & roll, float & pitch, float & yaw)
    {
        /*
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quat = [x, y, z, w]
        */
        float x = quat.x;
        float y = quat.y;
        float z = quat.z;
        float w = quat.w;

        float sinr_cosp = 2 * (w*x + y*z);
        float cosr_cosp = 1 - 2*(x*x + y*y);
        roll = atan2(sinr_cosp, cosr_cosp);

        float sinp = 2 * (w*y - z*x);
        pitch = asin(sinp);

        float siny_cosp = 2 * (w*z + x*y);
        float cosy_cosp = 1 - 2 * (y*y + z*z);
        yaw = atan2(siny_cosp, cosy_cosp);
    }

private:
    int step;
    nav_msgs::msg::Odometry odom;
    sensor_msgs::msg::LaserScan scan;

    Point rotation_point;
    Point center_point;

    float theta, yaw;
    float last_pose_x, last_pose_y, last_pose_theta;
    float goal_pose_x, goal_pose_y, goal_pose_theta;
    bool get_key_state, init_scan_state, init_odom_state;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_spot_pub;

    rclcpp::TimerBase::SharedPtr update_timer;
    
    ofstream delay_outfile_1;
    ofstream delay_outfile_2;

    int count1, count2;
    double addition1, addition2;
};
