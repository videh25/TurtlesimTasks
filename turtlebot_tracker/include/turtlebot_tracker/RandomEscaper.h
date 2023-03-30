#ifndef RANDOM_ESCAPER_TURTLE_H
#define RANDOM_ESCAPER_TURTLE_H

#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <string.h>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include "geometry_msgs/Twist.h"

namespace NRT_Task{

class RandomEscaper{
    private:
        ros::NodeHandle* nh_;
        ros::Subscriber self_pose_subscriber_;
        ros::Subscriber pt_pose_subscriber_;
        ros::Publisher velocity_publisher_;
        ros::Publisher rt_pose_publisher_;
        ros::Timer velocity_publisher_timer_;
        ros::Timer velocity_randomize_timer_;
        ros::Timer rt_pose_publisher_timer_;

        float linear_velocity_cap;
        float angular_velocity_cap;
        float warning_radius;
        float escape_distance_;
        float wall_sampling;
        std::string bot_name_;
        std::string police_name_;
        geometry_msgs::Twist velocity_msg_;
        turtlesim::Pose self_pose;
        

        void pt_pose_callback(const turtlesim::Pose& msg);
        void self_pose_callback(const turtlesim::Pose& msg);
        void publish_callback(const ros::TimerEvent& event);
        void randomize_velocity(const ros::TimerEvent& event);
        void escaping_velocity(int obstacle_index);
        int check_for_obstacles();
        void rt_pose_publish_callback(const ros::TimerEvent& event);
        bool obstacle_in_sight;
        std::vector<Eigen::Vector2f> obstacles;

    public:
        RandomEscaper(ros::NodeHandle* nh, std::string bot_name, std::string police_name, float max_linear_velocity, float max_angular_velocity, float warning_radius, float escape_distance);

};

};


#endif