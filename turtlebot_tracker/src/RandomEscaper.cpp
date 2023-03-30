#include <bits/stdc++.h>
#include <algorithm>
#include "turtlebot_tracker/RandomEscaper.h"
#include <ros/console.h>
#include <math.h>
#include <chrono>
#include <thread>

namespace NRT_Task{

RandomEscaper::RandomEscaper(ros::NodeHandle* nh, std::string bot_name, std::string police_name, float max_linear_velocity, float max_angular_velocity, float warning_radius, float escape_distance)
: nh_(nh),
bot_name_(bot_name),
police_name_(police_name),
linear_velocity_cap(max_linear_velocity),
angular_velocity_cap(max_angular_velocity),
warning_radius(warning_radius),
escape_distance_(escape_distance)
{
    velocity_publisher_ = nh_->advertise<geometry_msgs::Twist>(bot_name + std::string("/cmd_vel"), 1);
    rt_pose_publisher_= nh_->advertise<turtlesim::Pose>("/rt_real_pose", 1);
    self_pose_subscriber_ = nh_->subscribe(bot_name_ + std::string("/pose"), 1, &RandomEscaper::self_pose_callback, this);
    pt_pose_subscriber_ = nh_->subscribe(police_name_ + std::string("/pose"), 1, &RandomEscaper::pt_pose_callback, this);
    velocity_publisher_timer_ = nh_->createTimer(ros::Duration(0.3), &RandomEscaper::publish_callback, this);
    velocity_randomize_timer_ = nh_->createTimer(ros::Duration(3.0), &RandomEscaper::randomize_velocity, this);
    rt_pose_publisher_timer_ = nh_->createTimer(ros::Duration(5.), &RandomEscaper::rt_pose_publish_callback, this);
    velocity_msg_.linear.x = 0;
    velocity_msg_.angular.z = 0;

    wall_sampling = 7;
    for (int i = 0; i < wall_sampling; i++){
        obstacles.push_back(Eigen::Vector2f(float(i)/wall_sampling*11., 0.));
        obstacles.push_back(Eigen::Vector2f(11., float(i)/wall_sampling*11.));
        obstacles.push_back(Eigen::Vector2f(float(i)/wall_sampling*11., 11.));
        obstacles.push_back(Eigen::Vector2f(0., float(i)/wall_sampling*11.));
    }

    obstacles.push_back(Eigen::Vector2f());


}


void RandomEscaper::pt_pose_callback(const turtlesim::Pose& msg){
    obstacles[4*wall_sampling][0] = msg.x;
    obstacles[4*wall_sampling][1] = msg.y;
}

void RandomEscaper::self_pose_callback(const turtlesim::Pose& msg){
    self_pose = msg;
}

void RandomEscaper::publish_callback(const ros::TimerEvent& event){
    int obstacle_index = check_for_obstacles();
    if (obstacle_in_sight){
        escaping_velocity(obstacle_index);
    }
    
    velocity_publisher_.publish(velocity_msg_);
}

void RandomEscaper::randomize_velocity(const ros::TimerEvent& event){
    if (!obstacle_in_sight){
        float angular_vel = float(rand()%100-50)/50.*angular_velocity_cap;
        float linear_vel = float(rand()%100-50)/50.*linear_velocity_cap;

        velocity_msg_.linear.x = linear_vel;
        velocity_msg_.angular.z = angular_vel;
    }
}


void RandomEscaper::escaping_velocity(int obstacle_index){
    // ROS_INFO_STREAM("Current Obstacle Distance: " << (Eigen::Vector2f(self_pose.x, self_pose.y) - obstacles[obstacle_index]).norm());
    // ROS_INFO_STREAM("Current Obstacle: " << obstacle_index);
    
    if(obstacle_index == 4*wall_sampling){
        float dir_angle = abs(std::atan2(obstacles[obstacle_index][1] - self_pose.y, obstacles[obstacle_index][0] - self_pose.x) - self_pose.theta);
        if(dir_angle >= 100./180.*M_PI){
            ROS_INFO("Avoiding the police bot!");
            velocity_msg_.linear.x = linear_velocity_cap;
            velocity_msg_.angular.z = 0;
        }else{
            velocity_msg_.linear.x = 0;
            velocity_msg_.angular.z = angular_velocity_cap;
        }
    }else if(obstacle_index % 4 == 0){
        ROS_INFO("Avoiding the wall!");
        if(abs(self_pose.theta - M_PI/2.) <= 20./180.*M_PI){
            velocity_msg_.linear.x = linear_velocity_cap;
            velocity_msg_.angular.z = 0;
        }else{
            velocity_msg_.linear.x = 0;
            velocity_msg_.angular.z = angular_velocity_cap;
        }
    }else if(obstacle_index % 4 == 1){
        ROS_INFO("Avoiding the wall!");
        if((abs(self_pose.theta - M_PI) <= 20./180.*M_PI) || (abs(self_pose.theta + M_PI) <= 20./180.*M_PI)){
            velocity_msg_.linear.x = linear_velocity_cap;
            velocity_msg_.angular.z = 0;
        }else{
            velocity_msg_.linear.x = 0;
            velocity_msg_.angular.z = angular_velocity_cap;
        }
    }else if(obstacle_index % 4 == 2){
        ROS_INFO("Avoiding the wall!");
        if(abs(self_pose.theta + M_PI/2) <= 20./180.*M_PI){
            velocity_msg_.linear.x = linear_velocity_cap;
            velocity_msg_.angular.z = 0;
        }else{
            velocity_msg_.linear.x = 0;
            velocity_msg_.angular.z = angular_velocity_cap;
        }
    }else if(obstacle_index % 4 == 3){
        ROS_INFO("Avoiding the wall!");
        if(abs(self_pose.theta) <= 20./180.*M_PI){
            velocity_msg_.linear.x = linear_velocity_cap;
            velocity_msg_.angular.z = 0;
        }else{
            velocity_msg_.linear.x = 0;
            velocity_msg_.angular.z = angular_velocity_cap;
        }
    }

    velocity_publisher_.publish(velocity_msg_);
}

int RandomEscaper::check_for_obstacles(){
    int closest_obstacle = -1;
    float closest_distance = 10E9F;
    
    if ((Eigen::Vector2f(self_pose.x, self_pose.y) - obstacles[4*wall_sampling]).norm() < warning_radius){
        obstacle_in_sight = true;
        return 4*wall_sampling;
    }
    
    for (int i = 4*wall_sampling - 1; i > -1; i--){
        float this_distance = (Eigen::Vector2f(self_pose.x, self_pose.y) - obstacles[i]).norm();
        if (this_distance < closest_distance){
            closest_distance = this_distance;
            closest_obstacle = i;
        }
    }
    if (closest_distance < 1.){
        obstacle_in_sight = true;
    }else{
        obstacle_in_sight = false;
    }

    return closest_obstacle;
}

void RandomEscaper::rt_pose_publish_callback(const ros::TimerEvent& event){
    rt_pose_publisher_.publish(self_pose);
}

}
