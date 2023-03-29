#ifndef PID_TRACKER_TURTLE_H
#define PID_TRACKER_TURTLE_H

#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <string.h>
#include "geometry_msgs/Twist.h"
#include "turtlebot_tracker/SetTarget.h"

namespace NRT_Task{

class PointTracker{
    private:
        ros::NodeHandle* nh_;
        ros::Subscriber pose_subscriber_;
        ros::Publisher velocity_publisher_;
        ros::ServiceServer set_target_point_server_;

        float Kp_theta_, Kd_theta_, Ki_theta_;
        float Kp_dist_, Kd_dist_, Ki_dist_;
        float error_theta_last, error_dist_last;
        float integral_error_theta = 0, integral_error_dist = 0;
        double linear_velocity_cap, angular_velocity_cap; 
        std::chrono::_V2::system_clock::time_point last_loop_time_;  // std::chrono::high_resolution_clock::now();

        std::string bot_name_;
        geometry_msgs::Twist velocity_msg_;
        turtlesim::Pose target_pose_;

        void pose_callback(const turtlesim::Pose& msg);
        bool set_target_callback( turtlebot_tracker::SetTarget::Request &req, 
                                  turtlebot_tracker::SetTarget::Response &res);

    public:
        PointTracker(ros::NodeHandle* nh, float Kp_theta, float Kd_theta, float Ki_theta,float Kp_dist, float Kd_dist, float Ki_dist, std::string bot_name);

};

};


#endif