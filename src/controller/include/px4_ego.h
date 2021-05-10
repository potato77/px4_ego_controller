#ifndef PX4_EGO_H
#define PX4_EGO_H

#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>

#include <prometheus_msgs/ControlCommand.h>


#include "control_utils.h"
#include "uav_utils/geometry_utils.h"
#include "uav_utils/utils.h"

using namespace std;

ros::Subscriber odom_sub;
ros::Subscriber traj_cmd_sub;
ros::Subscriber cmd_sub;

ros::Publisher setpoint_raw_local_pub, setpoint_raw_attitude_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

prometheus_msgs::ControlCommand Command_Now;                      //无人机当前执行命令
prometheus_msgs::ControlCommand Command_Last;                     //无人机上一条执行命令
Eigen::Vector3d Takeoff_position;                                // 起飞位置

mavros_msgs::SetMode mode_cmd;
mavros_msgs::CommandBool arm_cmd;

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
bool traj_control = false; 

Desired_State_t desired_state;
Drone_State_t drone_state;

Eigen::Quaterniond u_q_des;   
Eigen::Vector4d u_att;   



// 控制参数
Eigen::Matrix3d Kp;
Eigen::Matrix3d Kv;
Eigen::Matrix3d Ka;


void init()
{
    // 初始化命令-
    // 默认设置：Idle模式 电机怠速旋转 等待来自上层的控制指令
    Command_Now.Mode                                = prometheus_msgs::ControlCommand::Idle;
    Command_Now.Command_ID                          = 0;
    Command_Now.Reference_State.Move_mode           = prometheus_msgs::PositionReference::XYZ_POS;
    Command_Now.Reference_State.Move_frame          = prometheus_msgs::PositionReference::ENU_FRAME;
    Command_Now.Reference_State.position_ref[0]     = 0;
    Command_Now.Reference_State.position_ref[1]     = 0;
    Command_Now.Reference_State.position_ref[2]     = 0;
    Command_Now.Reference_State.velocity_ref[0]     = 0;
    Command_Now.Reference_State.velocity_ref[1]     = 0;
    Command_Now.Reference_State.velocity_ref[2]     = 0;
    Command_Now.Reference_State.acceleration_ref[0] = 0;
    Command_Now.Reference_State.acceleration_ref[1] = 0;
    Command_Now.Reference_State.acceleration_ref[2] = 0;
    Command_Now.Reference_State.yaw_ref             = 0;

    u_att.setZero();
}

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

    drone_state.p = pos_drone;
    drone_state.v = vel_drone;
    drone_state.a.setZero();
    drone_state.q = q_drone;
    drone_state.yaw = yaw_drone;

}

void traj_cmd_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    traj_cmd = *msg;

    traj_control = true;
}

void cmd_cb(const prometheus_msgs::ControlCommand::ConstPtr& msg)
{
    Command_Now = *msg;

    if(Command_Now.Mode != prometheus_msgs::ControlCommand::Move)
    {
        traj_control = false;
    }
}

void idle()
{
    mavros_msgs::PositionTarget pos_setpoint;

    //飞控如何接收该信号请见mavlink_receiver.cpp
    //飞控如何执行该指令请见FlightTaskOffboard.cpp
    pos_setpoint.type_mask = 0x4000;

    setpoint_raw_local_pub.publish(pos_setpoint);
}

Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy)
{
    // YPR - ZYX
    return Eigen::Quaterniond(
                    Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                    Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
                    );
}

// 发送角度期望值至飞控（输入：期望角度-四元数,期望推力）
void send_attitude_setpoint(Eigen::Vector4d& u_att)
{
    mavros_msgs::AttitudeTarget att_setpoint;
    //geometry_msgs/Quaternion

    //Mappings: If any of these bits are set, the corresponding input should be ignored:
    //bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude

    att_setpoint.type_mask = 0b00000111;

    Eigen::Vector3d att_des;
    att_des << u_att(0), u_att(1), u_att(2);

    Eigen::Quaterniond q_des = quaternion_from_rpy(att_des);

    att_setpoint.orientation.x = q_des.x();
    att_setpoint.orientation.y = q_des.y();
    att_setpoint.orientation.z = q_des.z();
    att_setpoint.orientation.w = q_des.w();
    att_setpoint.thrust = u_att(3);

    setpoint_raw_attitude_pub.publish(att_setpoint);
}

// 输入：
// 无人机位置、速度、偏航角
// 期望位置、速度、加速度、偏航角
// 输出：
void pos_controller()
{
    Eigen::Vector3d pos_error = pos_des - pos_drone;
    Eigen::Vector3d vel_error  = vel_des - vel_drone;

    // 加速度 - 反馈部分
    // 根据位置、速度误差计算，同时设置限幅，即max_fb_acc_
    Eigen::Vector3d a_fb = Kp * pos_error + Kv * vel_error;  // feedforward term for trajectory error
    float max_fb_acc_ = 9.0;
    if (a_fb.norm() > max_fb_acc_)
    {
        a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large
        ROS_WARN("---->a_fb is too large.");
    }
        
    Eigen::Vector3d g_ << 0.0, 0.0, -9.8;
    // 期望加速度 = 加速度反馈部分 + 加速度参考值  - 重力加速度
    Eigen::Vector3d a_des = a_fb + acc_des  - g_;

    // 计算期望四元数
    q_des = acc2quaternion(a_des, mavYaw_);
}




#endif
