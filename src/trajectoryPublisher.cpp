#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <geom_flip/flatTargetmsg.h>
#include <sstream>
#include <iostream>
#include <Eigen/Dense>
#include <math.h>

#include "geom_flip/helper.h"

#define DO_CIRCLE 0

#define PI M_PI

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle n;
    ros::Publisher desired_state_pub = n.advertise<geom_flip::flatTargetmsg>("desired_state", 1);
    ros::Rate loop_rate(100);
    ros::Time start(ros::Time::now());

    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    Eigen::Vector3d accel_;

    Eigen::Vector3d traj_axis_;
    Eigen::Vector3d traj_origin_;
    Eigen::Vector3d traj_radial_;
    traj_axis_ << 1.0, 0.0, 0.0;
    traj_origin_ << 0.0, 0.0, 20.0;
    traj_radial_ << 5.0, 0.0, 0.0;

    int count = 0;
    double thrust = 30;
    double radius = 5.0, omega = 12, T_ = 2*3.14 / omega;
    double theta, trig_time;
    ros::Time cuurTime, startTime;
    Eigen::Matrix3d R;
    startTime = ros::Time::now();
    while (ros::ok()) {
        cuurTime = ros::Time::now();
        trig_time = (cuurTime - startTime).toSec();
#if DO_CIRCLE
        theta = omega * trig_time;
        pos_ = std::cos(theta) * traj_radial_ + std::sin(theta) * traj_axis_.cross(traj_radial_) +
                 (1 - std::cos(theta)) * traj_axis_.dot(traj_radial_) * traj_axis_ + traj_origin_;
        vel_ = omega * traj_axis_.cross(pos_);
        accel_ = omega * traj_axis_.cross(vel_);
#else
        pos_ << traj_origin_;
        vel_ << 0.0, 0.0, 0.0;
        if (trig_time < 1.0 * T_){
                accel_ = thrust* Eigen::Vector3d::UnitZ();
        }
        else if(trig_time < 3.0 * T_){ 
                R = Eigen::AngleAxisd(omega * trig_time, traj_axis_);
                accel_ = thrust* R * Eigen::Vector3d::UnitZ();
        }
        else if(trig_time < 3.1 * T_){
                accel_ = 0.1 * thrust * Eigen::Vector3d::UnitZ();
        }
        else{
                accel_ = 0.1 * thrust * Eigen::Vector3d::UnitZ();
                pos_ << traj_origin_;
                vel_ << 0.0, 0.0, 0.0;
        }
#endif
        geom_flip::flatTargetmsg msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.position = toGeometry(pos_);
        msg.acceleration = toGeometry(accel_);
        msg.velocity = toGeometry(vel_);
        desired_state_pub.publish(msg);
        loop_rate.sleep();
    }
    return 0;
}
