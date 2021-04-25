#ifndef PX4_EGO_H
#define PX4_EGO_H

#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>


#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>



#include <math.h>
#include "uav_utils/geometry_utils.h"
#include "uav_utils/utils.h"

using namespace std;

ros::Subscriber odom_sub;
ros::Subscriber traj_cmd_sub, point_cmd_sub;
ros::Publisher setpoint_raw_attitude_pub;

nav_msgs::Odometry drone_odom;
Eigen::Vector3d pos_drone;                      // 无人机位置
Eigen::Vector3d vel_drone;                      // 无人机速度
Eigen::Quaterniond q_drone;                 // 无人机四元数
double yaw_drone;

quadrotor_msgs::PositionCommand traj_cmd;
Eigen::Vector3d pos_des;         
Eigen::Vector3d vel_des;         
Eigen::Vector3d acc_des;                   
double yaw_des;                

Eigen::Quaterniond u_q_des;   
Eigen::Vector3d u_att_des;   

void drone_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    drone_odom = *msg;

    pos_drone(0) = drone_odom.pose.pose.position.x;
    pos_drone(1) = drone_odom.pose.pose.position.y;
    pos_drone(2) = drone_odom.pose.pose.position.z;

    vel_drone(0) = drone_odom.twist.twist.linear.x;
    vel_drone(1) = drone_odom.twist.twist.linear.y;
    vel_drone(2) = drone_odom.twist.twist.linear.z;

    q_drone.w() = msg->pose.pose.orientation.w;
    q_drone.x() = msg->pose.pose.orientation.x;
    q_drone.y() = msg->pose.pose.orientation.y;
    q_drone.z() = msg->pose.pose.orientation.z;    

    yaw_drone = uav_utils::get_yaw_from_quaternion(q_drone);
}

void traj_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    traj_cmd = *msg;

    pos_des(0) = traj_cmd.position.x;
    pos_des(1) = traj_cmd.position.y;
    pos_des(2) = traj_cmd.position.z;

    vel_des(0) = traj_cmd.velocity.x;
    vel_des(1) = traj_cmd.velocity.y;
    vel_des(2) = traj_cmd.velocity.z;

    acc_des(0) = traj_cmd.acceleration.x;
    acc_des(1) = traj_cmd.acceleration.y;
    acc_des(2) = traj_cmd.acceleration.z;

    yaw_des = uav_utils::normalize_angle(traj_cmd.yaw);
}

void point_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    traj_cmd = *msg;

    pos_des(0) = traj_cmd.position.x;
    pos_des(1) = traj_cmd.position.y;
    pos_des(2) = traj_cmd.position.z;

    vel_des(0) = traj_cmd.velocity.x;
    vel_des(1) = traj_cmd.velocity.y;
    vel_des(2) = traj_cmd.velocity.z;

    acc_des(0) = traj_cmd.acceleration.x;
    acc_des(1) = traj_cmd.acceleration.y;
    acc_des(2) = traj_cmd.acceleration.z;

    yaw_des = uav_utils::normalize_angle(traj_cmd.yaw);
}

//发送角度期望值至飞控（输入：期望角度-四元数,期望推力）
// void send_attitude_setpoint(const prometheus_msgs::AttitudeReference& _AttitudeReference)
// {
//     mavros_msgs::AttitudeTarget att_setpoint;
//     //geometry_msgs/Quaternion

//     //Mappings: If any of these bits are set, the corresponding input should be ignored:
//     //bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude

//     att_setpoint.type_mask = 0b00000111;

//     att_setpoint.orientation.x = _AttitudeReference.desired_att_q.x;
//     att_setpoint.orientation.y = _AttitudeReference.desired_att_q.y;
//     att_setpoint.orientation.z = _AttitudeReference.desired_att_q.z;
//     att_setpoint.orientation.w = _AttitudeReference.desired_att_q.w;

//     att_setpoint.thrust = _AttitudeReference.desired_throttle;

//     setpoint_raw_attitude_pub.publish(att_setpoint);
// }
#endif
