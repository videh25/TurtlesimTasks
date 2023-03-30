#include <ros/ros.h>
#include "turtlesim/Pose.h"
#include "turtlebot_tracker/PointTracker.h"

int main (int argc, char **argv)
{
    std::string bot_name;
    float Kpt, Kdt, Kit;
    float Kpd, Kdd, Kid;
    float velo_lim;
    bot_name = std::string(argv[1]);


    ros::init(argc, argv, bot_name + std::string("_point_tracking_node"));
    ros::NodeHandle nh;
    nh.getParam(bot_name+"/PID_Distance/Kp", Kpd);
    nh.getParam(bot_name+"/PID_Distance/Ki", Kid);
    nh.getParam(bot_name+"/PID_Distance/Kd", Kdd);

    nh.getParam(bot_name+"/PID_Theta/Kp", Kpt);
    nh.getParam(bot_name+"/PID_Theta/Ki", Kit);
    nh.getParam(bot_name+"/PID_Theta/Kd", Kdt);

    nh.getParam(bot_name+"/velocity_limit", velo_lim);
    
    NRT_Task::PointTracker tracker_node(&nh, Kpt, Kdt, Kit, Kpd, Kdd, Kid, velo_lim,bot_name);
    ros::spin(); 
}


// rosrun turtlebot_tracker turtle_point_tracker T1 1 0.2 0 0.3 0 0.1