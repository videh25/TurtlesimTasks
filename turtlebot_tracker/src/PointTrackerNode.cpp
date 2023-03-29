#include <ros/ros.h>
#include "turtlesim/Pose.h"
#include "turtlebot_tracker/PointTracker.h"

int main (int argc, char **argv)
{
    std::string bot_name;
    float Kpt, Kdt, Kit;
    float Kpd, Kdd, Kid;


    // Arguments received as {bot_name Kp Kd}
    if (argc >= 2){
        bot_name = std::string(argv[1]);
    } else {
        bot_name = std::string("");
    }

    if (argc >= 8){
        Kpt = std::stof(argv[2]);
        Kdt = std::stof(argv[3]);
        Kit = std::stof(argv[4]);

        Kpd = std::stof(argv[5]);
        Kdd = std::stof(argv[6]);
        Kid = std::stof(argv[7]);

    } else {
        Kpt = 1.;
        Kdt = 0.;
        Kit = 0.;

        Kpd = 1.;
        Kdd = 0.;
        Kid = 0.;
    }
    
    ros::init(argc, argv, bot_name + std::string("_point_tracking_node"));
    ros::NodeHandle nh;
    NRT_Task::PointTracker tracker_node(&nh, Kpt, Kdt, Kit, Kpd, Kdd, Kid, bot_name);
    ros::spin(); 
}


// rosrun turtlebot_tracker turtle_point_tracker T1 1 0.2 0 0.3 0 0.1