#include <ros/ros.h>
#include "turtlesim/Pose.h"
#include "turtlebot_tracker/RandomEscaper.h"

int main (int argc, char **argv)
{
    std::string bot_name, police_name;
    bot_name = std::string(argv[1]);
    police_name = std::string(argv[2]);
    float max_linear_velo, max_angular_velo, warning_rad, escape_dist; 

    ros::init(argc, argv, bot_name + std::string("_random_escaper_node"));
    ros::NodeHandle nh;

    nh.getParam(bot_name+"/linear_velocity_cap", max_linear_velo);
    nh.getParam(bot_name+"/angular_velocity_cap", max_angular_velo);
    nh.getParam(bot_name+"/warning_radius", warning_rad);
    nh.getParam(bot_name+"/escape_distance", escape_dist);

    NRT_Task::RandomEscaper tracker_node(&nh, bot_name, police_name, max_linear_velo, max_angular_velo, warning_rad, escape_dist);
    ros::spin(); 
}

