#ifndef CHASE_TRACKER_TURTLE_H
#define CHASE_TRACKER_TURTLE_H

#include "turtlebot_tracker/PointTracker.h"

namespace NRT_Task{

class ChaseTracker: public PointTracker{
    private:
        ros::Subscriber rt_pose_subscriber;
        float acceleration_limit_;
        double linear_velocity_cap, angular_velocity_cap;

        void rt_pose_callback(const turtlesim::Pose& msg);
        void pose_callback(const turtlesim::Pose& msg);

    public:
        ChaseTracker(ros::NodeHandle* nh, float Kp_theta, float Kd_theta, float Ki_theta,float Kp_dist, float Kd_dist, float Ki_dist, float velocity_limit, float acceleration_limit, std::string bot_name);
};


}


#endif