#include "turtlebot_tracker/ChaseTracker.h"

namespace NRT_Task{

ChaseTracker::ChaseTracker(ros::NodeHandle* nh, float Kp_theta, float Kd_theta, float Ki_theta,float Kp_dist, float Kd_dist, float Ki_dist, float velocity_limit, float acceleration_limit, std::string bot_name)
: PointTracker(nh, Kp_theta, Kd_theta, Ki_theta, Kp_dist, Kd_dist, Ki_dist, velocity_limit, bot_name), 
acceleration_limit_(acceleration_limit)
{
    rt_pose_subscriber = nh_->subscribe("/rt_real_pose", 1, &ChaseTracker::rt_pose_callback, this);
    pose_subscriber_ = nh_->subscribe(bot_name + std::string("/pose"), 1, &ChaseTracker::pose_callback, this);
    target_radius_ = 1.0;
    angular_velocity_cap = velocity_limit/0.3;
}

void ChaseTracker::rt_pose_callback(const turtlesim::Pose& msg){
    target_pose_.x = msg.x;
    target_pose_.y = msg.y;
}

void ChaseTracker::pose_callback(const turtlesim::Pose& msg){
    float error_theta = std::atan2(target_pose_.y - msg.y, target_pose_.x - msg.x) - msg.theta;
    if (abs(error_theta) >= 3){
        error_theta = 3.14;
    }

    float error_dist = pow(pow(target_pose_.y - msg.y, 2) + pow(target_pose_.x - msg.x, 2), 0.5);

    if ((msg.x == 0) || (msg.x == 11) || (msg.y == 0) || (msg.y == 11)){
        integral_error_dist = 0.;
    }

    std::chrono::_V2::system_clock::time_point current_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_loop_time_).count()/10E3F;
    
    float error_dist_d = (error_dist - error_dist_last)/duration;
    
    // if (error_dist < linear_velocity_cap/angular_velocity_cap){
    //     integral_error_dist = 0;
    //     integral_error_theta = 0;
    // }
    
    if (error_dist < target_radius_){
        integral_error_dist = 0;
        integral_error_theta = 0;
        velocity_msg_.angular.z = 0.;
        velocity_msg_.linear.x = 0.;
        velocity_publisher_.publish(velocity_msg_);
        return;
    }

    float error_theta_d = (error_theta - error_theta_last)/duration;
    integral_error_dist += (error_dist + error_dist_last)/2*duration;
    integral_error_theta += (error_theta + error_theta_last)/2*duration;

    velocity_msg_.angular.z = (Kp_theta_*error_theta + Kd_theta_*error_theta_d + Ki_theta_*integral_error_theta);
    velocity_msg_.linear.x = 0.1*(Kp_dist_*error_dist + Kd_dist_*error_dist_d + Ki_dist_*integral_error_dist);


    float real_acceleration = (velocity_msg_.linear.x - msg.linear_velocity)/duration; 
    // ROS_INFO_STREAM("Distance: " << error_dist);
    if (abs(real_acceleration) >= acceleration_limit_){
        velocity_msg_.linear.x = msg.linear_velocity + (-1 + 2*(real_acceleration > 0))*(acceleration_limit_);
    }

    velocity_msg_.angular.z = std::min(velocity_msg_.angular.z, angular_velocity_cap);
    velocity_msg_.linear.x = std::min(velocity_msg_.linear.x, linear_velocity_cap);
    ROS_INFO_STREAM("Latest Linear Speed: " << velocity_msg_.linear.x);
    ROS_INFO_STREAM("Linear Speed Limit: " << linear_velocity_cap);

    error_theta_last = error_theta;
    error_dist_last = error_dist;


    if (duration > 0.1){
        velocity_publisher_.publish(velocity_msg_);
    }
}

bool PointTracker::set_target_callback(turtlebot_tracker::SetTarget::Request &req, turtlebot_tracker::SetTarget::Response &res){
    target_pose_ = req.target_point;
    target_pose_.x = std::max((float)0, std::min((float) 11, req.target_point.x));
    target_pose_.y = std::max((float)0, std::min((float) 11, req.target_point.y));

    ROS_INFO_STREAM("Set the new target state to (" << target_pose_.x << ", " << target_pose_.y << ")");

    res.status.data = true;

    return true;
}

}
