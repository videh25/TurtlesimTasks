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
        ros::Timer velocity_publisher_timer_;
        ros::Timer velocity_randomize_timer_;

        float linear_velocity_cap;
        float angular_velocity_cap;
        float warning_radius;
        float escape_distance_;
        std::string bot_name_;
        std::string police_name_;
        geometry_msgs::Twist velocity_msg_;
        turtlesim::Pose pt_pose;
        turtlesim::Pose self_pose;
        std::vector<Eigen::Vector2cf> escape_routes;
        

        void pt_pose_callback(const turtlesim::Pose& msg);
        void self_pose_callback(const turtlesim::Pose& msg);
        void publish_callback(const ros::TimerEvent& event);
        void randomize_velocity(const ros::TimerEvent& event);
        void escape();
        bool is_a_free_point(double point_x, double point_y);
        bool self_pose_received_once;
        bool pt_pose_received_once;

    public:
        RandomEscaper(ros::NodeHandle* nh, std::string bot_name, std::string police_name);

};

};


#endif