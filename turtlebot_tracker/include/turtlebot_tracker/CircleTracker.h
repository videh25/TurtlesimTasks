#ifndef CIRCLE_TRACKER_TURTLE_H
#define CIRCLE_TRACKER_TURTLE_H

#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <string.h>
#include "geometry_msgs/Twist.h"
#include "turtlebot_tracker/SetRadiusSpeed.h"

namespace NRT_Task{

class CircleTracker{
    private:
        ros::NodeHandle* nh_;
        ros::Subscriber pose_subscriber_;
        ros::Publisher velocity_publisher_;
        ros::Publisher rt_pose_publisher_;
        ros::ServiceServer set_radius_speed_server_;
        ros::Timer velocity_publisher_timer_;
        ros::Timer rt_pose_publisher_timer_;

        std::string bot_name_;
        geometry_msgs::Twist velocity_msg_;
        turtlesim::Pose rt_pose;

        void pose_callback(const turtlesim::Pose& msg);
        bool set_radius_speed_callback( turtlebot_tracker::SetRadiusSpeed::Request &req, 
                                  turtlebot_tracker::SetRadiusSpeed::Response &res);
        void publish_callback(const ros::TimerEvent& event);
        void rt_pose_publish_callback(const ros::TimerEvent& event);

    public:
        CircleTracker(ros::NodeHandle* nh, std::string bot_name);

};

};


#endif