#include <bits/stdc++.h>
#include <algorithm>
#include "turtlebot_tracker/RandomEscaper.h"
#include <ros/console.h>
#include <math.h>

namespace NRT_Task{

RandomEscaper::RandomEscaper(ros::NodeHandle* nh, std::string bot_name, std::string police_name)
: nh_(nh),
bot_name_(bot_name),
police_name_(police_name),
linear_velocity_cap(2.0),
angular_velocity_cap(4.0),
warning_radius(3.0),
escape_distance_(3.0)
{
    ROS_INFO_STREAM("Initialising Circle Tracker with Radius: " << 1. << ", Speed: " << 1.);
    velocity_publisher_ = nh_->advertise<geometry_msgs::Twist>(bot_name + std::string("/cmd_vel"), 1);
    self_pose_subscriber_ = nh_->subscribe(bot_name_ + std::string("/pose"), 1, &RandomEscaper::self_pose_callback, this);
    pt_pose_subscriber_ = nh_->subscribe(police_name_ + std::string("/pose"), 1, &RandomEscaper::pt_pose_callback, this);
    velocity_publisher_timer_ = nh_->createTimer(ros::Duration(0.1), &RandomEscaper::publish_callback, this);
    velocity_randomize_timer_ = nh_->createTimer(ros::Duration(3.0), &RandomEscaper::randomize_velocity, this);
    velocity_msg_.linear.x = 0;
    velocity_msg_.angular.z = 0;

    // Add the escape routes to check
    for (int i = 0; i <8; i++){
        static float escape_angle = M_PI * (-1. + 2.*float(i)/8.);
        static Eigen::Vector2d this_escape_vector(escape_distance_*std::cos(escape_angle), escape_distance_*std::sin(escape_angle));
        escape_routes.push_back(this_escape_vector);
    }   
    // static Eigen::Rotation2Dd rotation_matrix(escape_angle);
}


void RandomEscaper::pt_pose_callback(const turtlesim::Pose& msg){
    pt_pose = msg;
}

void RandomEscaper::self_pose_callback(const turtlesim::Pose& msg){
    self_pose = msg;
}

void RandomEscaper::publish_callback(const ros::TimerEvent& event){
    float dist = pow(pow(self_pose.y - pt_pose.y, 2) + pow(self_pose.x - pt_pose.x, 2), 0.5);

    // ROS_INFO_STREAM("Angle: " << dir_angle);
    // ROS_INFO_STREAM("Self Pose: (" << self_pose.x << ", " << self_pose.y << ") | Police Pose: (" << pt_pose.x << ", " << pt_pose.y << ")");
    // ROS_INFO_STREAM("Distance: " << dist);
    if (dist <= warning_radius){
        escape();
        return;
    }

    velocity_publisher_.publish(velocity_msg_);
}

void RandomEscaper::randomize_velocity(const ros::TimerEvent& event){

    float angular_vel = float(rand()%100-50)/50.*angular_velocity_cap;
    float linear_vel = float(rand()%100-50)/50.*linear_velocity_cap;

    velocity_msg_.linear.x = linear_vel;
    velocity_msg_.angular.z = angular_vel;
}

bool RandomEscaper::is_a_free_point(double point_x, double point_y){
    if ((abs(point_x - 5.5) > 5) || (abs(point_y - 5.5) > 5)){
        return false;
    }

    if (pow(pow(point_y - pt_pose.y, 2) + pow(point_x - pt_pose.x, 2), 0.5) < warning_radius){
        return false;
    }

    return true;
}

void RandomEscaper::escape(){
    float dir_angle = abs(std::atan2(pt_pose.y - self_pose.y, pt_pose.x - self_pose.x) - self_pose.theta);
    if(dir_angle >= 160./180.*3.14){
        velocity_msg_.linear.x = linear_velocity_cap;
        velocity_msg_.angular.z = 0;
    }else{
        velocity_msg_.linear.x = 0;
        velocity_msg_.angular.z = angular_velocity_cap;
    }

    velocity_publisher_.publish(velocity_msg_);
}

}
