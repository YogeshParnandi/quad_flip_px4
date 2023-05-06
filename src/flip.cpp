// Trajectory Tracking node

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <Eigen/Dense>

#include "geom_flip/flip.h"
#include "geom_flip/helper.h"

using namespace std;
using namespace Eigen;

flip::flip(const ros::NodeHandle &nh):nh_(nh){
    mavStateSub = nh_.subscribe("mavros/state", 1, &flip::mavStateCallback, this);
    mavPoseSub = nh_.subscribe("mavros/local_position/pose", 1, &flip::mavPoseCallback, this);
    mavTwistSub = nh_.subscribe("mavros/local_position/velocity_local", 1, &flip::mavTwistCallback, this);
    desiredStateSub = nh_.subscribe("desired_state", 1, &flip::desiredStateCallback, this);
    
    mavCmdloop_timer = nh_.createTimer(ros::Duration(0.01), &flip::mavCmdLoopCallback, this);
    mavStatusloop_timer = nh_.createTimer(ros::Duration(1), &flip::mavStatusLoopCallback, this);

    target_pose_pub =  nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    angularVelPub = nh_.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
    referencePosePub = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 1);

    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // change params on the go
    nodeState = WAITING_FOR_HOME_POSE;
    nh_.param<double>("Kp_x", Kpos_x, 1.0);
    nh_.param<double>("Kp_y", Kpos_y, 1.0);
    nh_.param<double>("Kp_z", Kpos_z, 2.0);
    nh_.param<double>("Kv_x", Kvel_x, 1.0);
    nh_.param<double>("Kv_y", Kvel_y, 1.0);
    nh_.param<double>("Kv_z", Kvel_z, 2.0);
    D << 0.0, 0.0, 0.0;
    target_pos << 0.0, 0.0, 20.0;
    target_vel << 0.0, 0.0, 0.0;
    mav_pos << 0.0, 0.0, 0.0;
    mav_vel << 0.0, 0.0, 0.0;
    Kpos << -Kpos_x, -Kpos_y, -Kpos_z;
    Kvel << -Kvel_x, -Kvel_y, -Kvel_z;
    max_fb_acc = 30.0;
    target_mav_yaw = 0.0;
}

flip::~flip()
{
}

void flip::mavStateCallback(const mavros_msgs::State::ConstPtr &msg){
    // Current State of drone
    current_state = *msg;
}

void flip::mavPoseCallback(const geometry_msgs::PoseStamped &msg){
    // current pose and oreintation of drone
    if (!homepose_set){
        homepose_set = true;
        home_pose = msg.pose;
        ROS_INFO_STREAM("Home pose set to: " << home_pose);
    }
    mav_pos = toEigen(msg.pose.position);
    // target_pos = toEigen(msg.pose.position);

    mav_att(0) = msg.pose.orientation.w;
    mav_att(1) = msg.pose.orientation.x;
    mav_att(2) = msg.pose.orientation.y;
    mav_att(3) = msg.pose.orientation.z;
    
}

void flip::mavTwistCallback(const geometry_msgs::TwistStamped &msg){
    // current linear and angular velocities of drone
    mav_vel = toEigen(msg.twist.linear);
    mav_angVel = toEigen(msg.twist.angular);
}

void flip::mavCmdLoopCallback(const ros::TimerEvent &event){
    switch (nodeState){
        case WAITING_FOR_HOME_POSE:
            waitForHomePose(&homepose_set, "Waiting for home pose..");
            ROS_INFO("Got home pose! Ready to Arm");
            nodeState = MISSION_EXECUTION;
            break;
        case MISSION_EXECUTION:{
            Eigen::Vector3d desired_acc;
            desired_acc = controlPosition(target_pos, target_vel, target_accel);
            computeBodyRateCmd(cmdBodyRate, desired_acc);
            pubReferencePose(target_pos, q_des);
            pubRateCommands(cmdBodyRate, q_des);
            break;
        }
        case LANDING:{
            geometry_msgs::PoseStamped land_msg;
            land_msg.header.stamp = ros::Time::now();
            land_msg.pose = home_pose;
            land_msg.pose.position.z = land_msg.pose.position.z + 1.0;
            target_pose_pub.publish(land_msg);
            nodeState = LANDED;
            ros::spinOnce();
            break;
        }
        case LANDED:{
            ROS_INFO("Landed");
            mavCmdloop_timer.stop();
            break;
        }
    }
}

void flip::mavStatusLoopCallback(const ros::TimerEvent &event){
    if (is_sim){
        mavros_msgs::SetMode offb_setmode;
        arm_cmd.request.value = true;
        offb_setmode.request.custom_mode = "OFFBOARD";
        if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if (set_mode_client.call(offb_setmode) && offb_setmode.response.mode_sent){
                ROS_INFO("OFFBOARD Enabled");
            }
            last_request = ros::Time::now();
        }
        else{
            if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Armed");
                }
                last_request = ros::Time::now();
            }
        }
    }
}


void flip::desiredStateCallback(const geom_flip::flatTargetmsg &msg){
    ref_req_prev = ref_req_now;
    target_pos_prev = target_pos;
    target_vel_prev = target_vel;

    ref_req_now = ros::Time::now();
    dt = (ref_req_now - ref_req_prev).toSec();

    target_pos = toEigen(msg.position);
    target_vel = toEigen(msg.velocity);
    target_accel = toEigen(msg.acceleration);

    target_jerk = Eigen::Vector3d::Zero();
}

Eigen::Vector4d flip::acc2quat(const Eigen::Vector3d &vector_acc, const double &yaw){
    Eigen::Vector4d quat;
    Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
    Eigen::Matrix3d rotmat;

    proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

    zb_des = vector_acc/vector_acc.norm();
    yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();

    rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
    quat = rot2quat(rotmat);
    return quat;
}

Eigen::Vector3d flip::controlPosition(const Eigen::Vector3d &targte_pos, const Eigen::Vector3d &target_vel, const Eigen::Vector3d &target_acc){
    
    const Eigen::Vector3d acc_ref = target_acc;

    const Eigen::Vector4d q_ref = acc2quat(acc_ref - g, target_mav_yaw);
    const Eigen::Matrix3d R_ref = qaut2rot(q_ref);

    const Eigen::Vector3d pos_error = mav_pos - target_pos;
    const Eigen::Vector3d vel_error = mav_vel - target_vel;

    const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);

    const Eigen::Vector3d a_rd = R_ref * D.asDiagonal() * R_ref.transpose() * target_vel;
    const Eigen::Vector3d a_des = a_fb + acc_ref - a_rd - g;
    return a_des;
}

Eigen::Vector3d flip::poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error){
    Eigen::Vector3d a_fb = Kpos.asDiagonal() * pos_error + Kvel.asDiagonal() * vel_error;
    if(a_fb.norm() > max_fb_acc){
        a_fb = (max_fb_acc / a_fb.norm()) * a_fb;
    }
    return a_fb;
}

void flip::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &target_acc){
    q_des = acc2quat(target_acc, target_mav_yaw);
    update(mav_att, q_des, target_acc, target_jerk);
    bodyrate_cmd.head(3) = getDesiredRate();
    double thrust_command = getDesiredThrust().z();
    bodyrate_cmd(3) = std::max(0.0, std::min(1.0, norm_thrust_const * thrust_command + norm_thrust_offset));
}

void flip::pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Vector4d &target_attitude){
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.body_rate.x = cmd(0);
    msg.body_rate.y = cmd(1);
    msg.body_rate.z = cmd(2);
    msg.type_mask = 128;  // Ignore orientation messages
    msg.orientation.w = target_attitude(0);
    msg.orientation.x = target_attitude(1);
    msg.orientation.y = target_attitude(2);
    msg.orientation.z = target_attitude(3);
    msg.thrust = cmd(3);
    angularVelPub.publish(msg);
}

void flip::pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude){
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.pose.position.x = target_position(0);
    msg.pose.position.y = target_position(1);
    msg.pose.position.z = target_position(2);
    msg.pose.orientation.w = target_attitude(0);
    msg.pose.orientation.x = target_attitude(1);
    msg.pose.orientation.y = target_attitude(2);
    msg.pose.orientation.z = target_attitude(3);
    referencePosePub.publish(msg);
}

void flip::update(Eigen::Vector4d &curr_att, const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
              const Eigen::Vector3d &ref_jerk){
    Eigen::Vector4d ratecmd;
    Eigen::Matrix3d rotmat;    // Rotation matrix of current attitude
    Eigen::Matrix3d rotmat_d;  // Rotation matrix of desired attitude
    Eigen::Vector3d error_att;

    rotmat = qaut2rot(curr_att);
    rotmat_d = qaut2rot(ref_att);

    error_att = 0.5 * matrix_hat_inv(rotmat_d.transpose() * rotmat - rotmat.transpose() * rotmat_d);
    desired_rate = (2.0 / attctrl_tau_) * error_att;
    const Eigen::Vector3d zb = rotmat.col(2);
    desired_thrust(0) = 0.0;
    desired_thrust(1) = 0.0;
    desired_thrust(2) = ref_acc.dot(zb);
}


void flip::dynamicReconfigureCallback(geom_flip::FlipConfig &config, uint32_t level) {
    if (Kpos_x != config.Kp_x){
        Kpos_x = config.Kp_x;
        ROS_INFO("Reconfigure request : Kp_x  = %.2f  ", config.Kp_x);
    } 
    else if (Kpos_y != config.Kp_y) {
        Kpos_y = config.Kp_y;
        ROS_INFO("Reconfigure request : Kp_y  = %.2f  ", config.Kp_y);
    } 
    else if (Kpos_z != config.Kp_z) {
        Kpos_z = config.Kp_z;
        ROS_INFO("Reconfigure request : Kp_z  = %.2f  ", config.Kp_z);
    } 
    else if (Kvel_x != config.Kv_x) {
        Kvel_x = config.Kv_x;
        ROS_INFO("Reconfigure request : Kv_x  = %.2f  ", config.Kv_x);
    } 
    else if (Kvel_y != config.Kv_y) {
        Kvel_y = config.Kv_y;
        ROS_INFO("Reconfigure request : Kv_y =%.2f  ", config.Kv_y);
    } 
    else if (Kvel_z != config.Kv_z) {
        Kvel_z = config.Kv_z;
        ROS_INFO("Reconfigure request : Kv_z  = %.2f  ", config.Kv_z);
    }
    Kpos << -Kpos_x, -Kpos_y, -Kpos_z;
    Kvel << -Kvel_x, -Kvel_y, -Kvel_z;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "geom_tracking_node");
    ros::NodeHandle nh;
    flip* myflip = new flip(nh);
    dynamic_reconfigure::Server<geom_flip::FlipConfig> srv;
    dynamic_reconfigure::Server<geom_flip::FlipConfig>::CallbackType f;
    f = boost::bind(&flip::dynamicReconfigureCallback, myflip, _1, _2);
    srv.setCallback(f);
    ros::spin();
    return 0;
}