#include <ros/ros.h>
#include <px4_ego.h>

//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_ego");
    ros::NodeHandle nh("~");


    odom_sub = nh.subscribe<nav_msgs::Odometry>( "/px4/drone_odom", 10, drone_odom_cb);

    traj_cmd_sub  = nh.subscribe<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 10, traj_cmd_cb);

    point_cmd_sub  = nh.subscribe<quadrotor_msgs::PositionCommand>("/terminal/point_cmd", 10, point_cmd_cb);

    // 【发布】角度/角速度期望值 坐标系 ENU系
    //  本话题要发送至飞控(通过Mavros功能包 /plugins/setpoint_raw.cpp发送), 对应Mavlink消息为SET_ATTITUDE_TARGET (#82), 对应的飞控中的uORB消息为vehicle_attitude_setpoint.msg（角度） 或vehicle_rates_setpoint.msg（角速度）
    setpoint_raw_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>( "/mavros/setpoint_raw/attitude", 10);



}