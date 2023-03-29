#include <bits/stdc++.h>
#include <algorithm>
#include "turtlebot_tracker/CircleTracker.h"
#include <ros/console.h>

namespace NRT_Task{

CircleTracker::CircleTracker(ros::NodeHandle* nh, std::string bot_name)
: nh_(nh),
bot_name_(bot_name)
{
    ROS_INFO_STREAM("Initialising Circle Tracker with Radius: " << 1. << ", Speed: " << 1.);
    set_radius_speed_server_ = nh_->advertiseService(bot_name + std::string("/set_radius_and_speed"), &CircleTracker::set_radius_speed_callback, this);
    rt_pose_publisher_= nh_->advertise<turtlesim::Pose>("/rt_real_pose", 1);
    velocity_publisher_ = nh_->advertise<geometry_msgs::Twist>(bot_name + std::string("/cmd_vel"), 1);
    pose_subscriber_ = nh_->subscribe(bot_name + std::string("/pose"), 1, &CircleTracker::pose_callback, this);
    velocity_publisher_timer_ = nh_->createTimer(ros::Duration(0.1), &CircleTracker::publish_callback, this);
    rt_pose_publisher_timer_ = nh_->createTimer(ros::Duration(5.), &CircleTracker::rt_pose_publish_callback, this);
    velocity_msg_.linear.x = 1;
    velocity_msg_.angular.z = 1;
        
}

bool CircleTracker::set_radius_speed_callback( turtlebot_tracker::SetRadiusSpeed::Request &req, turtlebot_tracker::SetRadiusSpeed::Response &res){
    if ((req.radius.data == 0) || (req.speed.data == 0)){
        res.status.data = false;

        velocity_msg_.linear.x = 0;
        velocity_msg_.angular.z = 0;
        
        return false;
    }

    velocity_msg_.linear.x = req.speed.data;
    velocity_msg_.angular.z = req.speed.data / req.radius.data;

    res.status.data = true;
    return true;
}

void CircleTracker::publish_callback(const ros::TimerEvent& event){
    velocity_publisher_.publish(velocity_msg_);
}

void CircleTracker::pose_callback(const turtlesim::Pose& msg){
    rt_pose = msg;
}

void CircleTracker::rt_pose_publish_callback(const ros::TimerEvent& event){
    rt_pose_publisher_.publish(rt_pose);
}

}
