#include <bits/stdc++.h>
#include <algorithm>
#include "turtlebot_tracker/PointTracker.h"
#include "geometry_msgs/Twist.h"
#include <ros/console.h>

namespace NRT_Task{

PointTracker::PointTracker(ros::NodeHandle* nh, float Kp_theta, float Kd_theta, float Ki_theta,float Kp_dist, float Kd_dist, float Ki_dist, std::string bot_name)
: nh_(nh),
Kp_theta_(Kp_theta),
Kd_theta_(Kd_theta),
Ki_theta_(Ki_theta),
Kp_dist_(Kp_dist),
Kd_dist_(Kd_dist),
Ki_dist_(Ki_dist),
target_radius_(0.4),
error_theta_last(0),
error_dist_last(0),
integral_error_dist(0),
integral_error_theta(0),
bot_name_(bot_name),
last_loop_time_(std::chrono::high_resolution_clock::now())
{
    ROS_INFO_STREAM("Initialising Tracker with ditance gains Kp: " << Kp_dist_ << ", Kd: " << Kd_dist_ << ", Ki: " << Ki_dist_);
    ROS_INFO_STREAM("Initialising Tracker with theta gains Kp: " << Kp_theta_ << ", Kd: " << Kd_theta_ << ", Ki: " << Ki_theta_);
    pose_subscriber_ = nh_->subscribe(bot_name + std::string("/pose"), 1, &PointTracker::pose_callback, this);
    velocity_publisher_ = nh_->advertise<geometry_msgs::Twist>(bot_name + std::string("/cmd_vel"), 1);
    set_target_point_server_ = nh_->advertiseService(bot_name + std::string("/set_target"), &PointTracker::set_target_callback, this);
    linear_velocity_cap = 2;
    angular_velocity_cap = 4;
}

void PointTracker::pose_callback(const turtlesim::Pose& msg){
    float error_theta = std::atan2(target_pose_.y - msg.y, target_pose_.x - msg.x) - msg.theta;
    if (abs(error_theta) >= 3){
        error_theta = 3.14;
    }

    float error_dist = pow(pow(target_pose_.y - msg.y, 2) + pow(target_pose_.x - msg.x, 2), 0.5);

    if ((msg.x == 0) || (msg.x == 11) || (msg.y == 0) || (msg.y == 11)){
        integral_error_dist = 0.;
    } 

    std::chrono::_V2::system_clock::time_point current_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_loop_time_).count()/10E3F;
    
    float error_dist_d = (error_dist - error_dist_last)/duration;
    
    if (error_dist < 0.5){
        integral_error_dist = 0;
        integral_error_theta = 0;
    }
    
    if (error_dist < target_radius_){
        integral_error_dist = 0;
        integral_error_theta = 0;
        velocity_msg_.angular.z = 0.;
        velocity_msg_.linear.x = 0.;
        velocity_publisher_.publish(velocity_msg_);
        return;
    }

    float error_theta_d = (error_theta - error_theta_last)/duration;
    integral_error_dist += (error_dist + error_dist_last)/2*duration;
    integral_error_theta += (error_theta + error_theta_last)/2*duration;

    velocity_msg_.angular.z = Kp_theta_*error_theta + Kd_theta_*error_theta_d + Ki_theta_*integral_error_theta;
    velocity_msg_.linear.x = 0.1*(Kp_dist_*error_dist + Kd_dist_*error_dist_d + Ki_dist_*integral_error_dist);

    velocity_msg_.angular.z = std::min(velocity_msg_.angular.z, angular_velocity_cap);
    velocity_msg_.linear.x = std::min(velocity_msg_.linear.x, linear_velocity_cap);

    error_theta_last = error_theta;
    error_dist_last = error_dist;

    ROS_INFO_STREAM("Ditance Error: " << error_dist_last << ": Theta Error: " << error_theta_last);

    if (duration > 0.1){
        velocity_publisher_.publish(velocity_msg_);
    }
}

bool PointTracker::set_target_callback(turtlebot_tracker::SetTarget::Request &req, turtlebot_tracker::SetTarget::Response &res){
    target_pose_ = req.target_point;
    target_pose_.x = std::max((float)0, std::min((float) 11, req.target_point.x));
    target_pose_.y = std::max((float)0, std::min((float) 11, req.target_point.y));

    ROS_INFO_STREAM("Set the new target state to (" << target_pose_.x << ", " << target_pose_.y << ")");

    res.status.data = true;

    return true;
}

}