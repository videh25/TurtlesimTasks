#include <ros/ros.h>
#include "turtlesim/Pose.h"
#include "turtlebot_tracker/CircleTracker.h"

int main (int argc, char **argv)
{
    std::string bot_name;

    // Arguments received as {bot_name Kp Kd}
    if (argc >= 2){
        bot_name = std::string(argv[1]);
    } else {
        bot_name = std::string("");
    }

    ros::init(argc, argv, bot_name + std::string("_circle_tracking_node"));
    ros::NodeHandle nh;
    NRT_Task::CircleTracker tracker_node(&nh, bot_name);
    ros::spin(); 
}

