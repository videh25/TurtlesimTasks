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

    if (argc >= 9){
        Kpt = std::stof(argv[2]);
        Kdt = std::stof(argv[3]);
        Kit = std::stof(argv[4]);

        Kpd = std::stof(argv[5]);
        Kdd = std::stof(argv[6]);
        Kid = std::stof(argv[7]);
        acceleration_lim = std::stof(argv[8]);
        velo_lim = std::stof(argv[9]);

    } else {
        Kpt = 1.;
        Kdt = 0.;
        Kit = 0.;

        Kpd = 1.;
        Kdd = 0.;
        Kid = 0.;
    }
    
    ros::init(argc, argv, bot_name + std::string("_chase_tracking_node"));
    ros::NodeHandle nh;
    NRT_Task::ChaseTracker tracker_node(&nh, Kpt, Kdt, Kit, Kpd, Kdd, Kid, acceleration_lim, velo_lim, bot_name);
    ros::spin(); 
}


// rosrun turtlebot_tracker turtle_chase_tracker PT 4 0.8 0 1.2 0 0.4 5