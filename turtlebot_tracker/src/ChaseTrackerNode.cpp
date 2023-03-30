#include <ros/ros.h>
#include "turtlesim/Pose.h"
#include "turtlebot_tracker/ChaseTracker.h"

int main (int argc, char **argv)
{
    std::string bot_name;
    float Kpt, Kdt, Kit;
    float Kpd, Kdd, Kid;
    float acceleration_lim, velo_lim;


    // Arguments received as {bot_name Kp Kd}
    if (argc >= 2){
        bot_name = std::string(argv[1]);
    } else {
        bot_name = std::string("");
    }
    ros::init(argc, argv, bot_name + std::string("_chase_tracking_node"));
    ros::NodeHandle nh;
    nh.getParam(bot_name+"/PID_Distance/Kp", Kpd);
    nh.getParam(bot_name+"/PID_Distance/Ki", Kid);
    nh.getParam(bot_name+"/PID_Distance/Kd", Kdd);

    nh.getParam(bot_name+"/PID_Theta/Kp", Kpt);
    nh.getParam(bot_name+"/PID_Theta/Ki", Kit);
    nh.getParam(bot_name+"/PID_Theta/Kd", Kdt);

    nh.getParam(bot_name+"/velocity_limit", velo_lim);
    nh.getParam(bot_name+"/acceleration_limit", acceleration_lim);
    

    ROS_INFO_STREAM("Initialised Parameters: Kpd: " << Kpd << "| Kid: " << Kid <<"| Kdd: " << Kdd);
    ROS_INFO_STREAM("Initialised Parameters: Kpt: " << Kpt << "| Kit: " << Kit <<"| Kdt: " << Kdt);
    ROS_INFO_STREAM("Initialised Parameters: acceleration_lim: " << acceleration_lim << "| velo_lim: " << velo_lim);
    NRT_Task::ChaseTracker tracker_node(&nh, Kpt, Kdt, Kit, Kpd, Kdd, Kid, velo_lim, acceleration_lim, bot_name);
    ros::spin(); 
}


// rosrun turtlebot_tracker turtle_chase_tracker PT 4 0.8 0 1.2 0 0.4 20 20