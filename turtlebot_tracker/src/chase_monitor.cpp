#include <ros/ros.h>
#include "turtlesim/Pose.h"
#include "ros/console.h"

float rt_x = 1000, rt_y = 1000, pt_x = 0, pt_y = 0;
float target_radius = 1.0;

void callback_rt_pose(const turtlesim::Pose& msg)
{
    rt_x = msg.x;
    rt_y = msg.y;
}

void callback_pt_pose(const turtlesim::Pose& msg)
{
    pt_x = msg.x;
    pt_y = msg.y;
}


void print_callback(const ros::TimerEvent& event){
    float dist = pow(pow(rt_x - pt_x, 2) + pow(rt_y - pt_y, 2), 0.5);
    ROS_INFO_STREAM("Current Distance: " << dist);
    if (dist < target_radius){
        ROS_INFO("Chase Completed!");

        ros::shutdown();
    }
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "Chase_Monitor");
    ros::NodeHandle nh;
    ros::Subscriber rt_bot_pose_subscriber = nh.subscribe(std::string(argv[1]) + std::string("/pose"), 1, callback_rt_pose);
    ros::Subscriber pt_bot_pose_subscriber = nh.subscribe(std::string(argv[2]) + std::string("/pose"), 1, callback_pt_pose);
    ros::Timer check_timer_ = nh.createTimer(ros::Duration(0.1), print_callback);

    ros::spin();
}