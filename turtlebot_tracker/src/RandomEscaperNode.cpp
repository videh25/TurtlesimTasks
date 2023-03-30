#include <ros/ros.h>
#include "turtlesim/Pose.h"
#include "turtlebot_tracker/RandomEscaper.h"

int main (int argc, char **argv)
{
    std::string bot_name, police_name;

    // Arguments received as {bot_name Kp Kd}
    bot_name = std::string(argv[1]);
    police_name = std::string(argv[2]);

    ros::init(argc, argv, bot_name + std::string("_random_escaper_node"));
    ros::NodeHandle nh;
    NRT_Task::RandomEscaper tracker_node(&nh, bot_name, police_name);
    ros::spin(); 
}

