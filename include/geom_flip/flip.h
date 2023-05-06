#ifndef FLIP_H
#define FLIP_H

#include <ros/ros.h>
#include <Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CompanionProcessStatus.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <dynamic_reconfigure/server.h>
#include <geom_flip/FlipConfig.h>
#include <geom_flip/flatTargetmsg.h>

enum FlightState{
    WAITING_FOR_HOME_POSE,
    MISSION_EXECUTION,
    LANDING,
    LANDED
};

class flip{
private:
    ros::NodeHandle nh_;
    Eigen::Vector3d mav_pos, mav_vel, mav_angVel;
    Eigen::Vector3d target_pos, target_vel, target_accel;
    Eigen::Vector3d target_pos_prev, target_vel_prev;
    Eigen::Vector3d target_jerk;
    Eigen::Vector3d desired_thrust, desired_rate;
    Eigen::Vector4d mav_att, q_des;
    Eigen::Vector4d cmdBodyRate; 
    double target_mav_yaw;
    double max_fb_acc;
    double attctrl_tau_ = 1.0;

    bool is_sim = true;
    bool homepose_set = false;
    
    FlightState nodeState;

    Eigen::Vector3d g{Eigen::Vector3d(0.0, 0.0, -9.8)};

    
    mavros_msgs::State current_state;
    mavros_msgs::CommandBool arm_cmd;
    geometry_msgs::Pose home_pose;

    Eigen::Vector3d Kpos, Kvel, D;
    double Kpos_x, Kpos_y, Kpos_z, Kvel_x, Kvel_y, Kvel_z;
    double norm_thrust_const = 0.05, norm_thrust_offset = 0.1;

    ros::Subscriber mavStateSub;
    void mavStateCallback(const mavros_msgs::State::ConstPtr &msg);

    ros::Subscriber mavPoseSub;
    void mavPoseCallback(const geometry_msgs::PoseStamped &msg);

    ros::Subscriber mavTwistSub;
    void mavTwistCallback(const geometry_msgs::TwistStamped &msg);

    ros::Subscriber desiredStateSub;
    void desiredStateCallback(const geom_flip::flatTargetmsg &msg);

    ros::Timer mavCmdloop_timer;
    void mavStatusLoopCallback(const ros::TimerEvent& event);
    
    ros::Timer mavStatusloop_timer;
    void mavCmdLoopCallback(const ros::TimerEvent& ebent);

    void pubMotorCommands();
    void pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude);
    void pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude);

    ros::Publisher target_pose_pub, angularVelPub;
    ros::Publisher referencePosePub;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    ros::Time last_request, ref_req_now, ref_req_prev;
    double dt;

    template <class T>
    void waitForHomePose(const T *val, const std::string &msg, double hz = 2.0){
        ros::Rate pause(hz);
        ROS_INFO_STREAM(msg);
        while(ros::ok() && !(*val)){
            ros::spinOnce();
            pause.sleep();
        }
    }
public:
    flip(const ros::NodeHandle &nh);
    void get_state(Eigen::Vector3d &pos, Eigen::Vector4d &att, Eigen::Vector3d &vel, Eigen::Vector3d &angvel){
        pos = mav_pos;
        att = mav_att;
        vel = mav_vel;
        angvel = mav_angVel;
    }
    Eigen::Vector3d getDesiredThrust(){ 
        return desired_thrust; 
    };
    Eigen::Vector3d getDesiredRate(){ 
        return desired_rate; 
    };
    Eigen::Vector3d controlPosition(const Eigen::Vector3d &targte_pos, const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc);
    static Eigen::Vector4d acc2quat(const Eigen::Vector3d &vector_acc, const double &yaw);
    Eigen::Vector3d poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error);
    void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &target_acc);
    void update(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
              const Eigen::Vector3d &ref_jerk);
    void dynamicReconfigureCallback(geom_flip::FlipConfig &config, uint32_t level);
    ~flip();
};
#endif