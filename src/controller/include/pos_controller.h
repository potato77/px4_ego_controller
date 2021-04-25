#ifndef POS_CONTROLLER_H
#define POS_CONTROLLER_H

#include <Eigen/Eigen>
#include <iostream>
#include "uav_utils/geometry_utils.h"
#include "uav_utils/utils.h"

using namespace std;

struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
    Eigen::Vector3d a;
	double yaw;
	Eigen::Quaterniond q;
};

struct Drone_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
    Eigen::Vector3d a;
	double yaw;
	Eigen::Quaterniond q;
};


class pos_controller
{
     //public表明该数据成员、成员函数是对全部用户开放的。全部用户都能够直接进行调用，在程序的不论什么其他地方訪问。
    public:

        //构造函数
        pos_controller(void):
            nh("~")
        {
            nh.param<float>("Quad/mass", Quad_MASS, 1.0);
            nh.param<double>("Quad/hov_percent", hov_percent, 0.5);

            nh.param<double>("gain/Kp_xy", Kp(0,0), 1.0f);
            nh.param<double>("gain/Kp_xy", Kp(1,1), 1.0f);
            nh.param<double>("gain/Kp_z" , Kp(2,2), 1.0f);
            nh.param<double>("gain/Kv_xy", Kv(0,0), 1.0f);
            nh.param<double>("gain/Kv_xy", Kv(1,1), 1.0f);
            nh.param<double>("gain/Kv_z" , Kv(2,2), 1.0f);
            nh.param<double>("gain/Ki_xy", Kvi(0,0), 1.0f);
            nh.param<double>("gain/Ki_xy", Kvi(1,1), 1.0f);
            nh.param<double>("gain/Ki_z" , Kvi(2,2), 1.0f);
            nh.param<double>("gain/Ka_xy", Ka(0,0), 1.0f);
            nh.param<double>("gain/Ka_xy", Ka(1,1), 1.0f);
            nh.param<double>("gain/Ka_z" , Ka(2,2), 1.0f);
            nh.param<double>("gain/K_yaw", Kyaw, 1.0f);

            int_e_v.setZero();
            gravity = 9.8;
        }

        //Quadrotor Parameter
        float Quad_MASS;
        float gravity;
        double hov_percent;

        Eigen::Matrix3d Kp;
        Eigen::Matrix3d Kv;
        Eigen::Matrix3d Kvi;
        Eigen::Matrix3d Ka;
        double Kyaw;

        Eigen::Vector3d int_e_v;

        void process_control(const Desired_State_t& des, const Drone_State_t& odom, Eigen::Vector4d& u);

    private:

        ros::NodeHandle nh;

};

// 控制主程序
// 输入：期望位置、速度、加速度，相关参数
// 输出：期望姿态、期望油门
void pos_controller::process_control(const Desired_State_t& des, const Drone_State_t& odom, Eigen::Vector4d& u_att)
{
	// 误差项，期望力
	Eigen::Vector3d e_p, e_v, F_des;
	double e_yaw = 0.0;


	if (des.v(0) != 0.0 || des.v(1) != 0.0 || des.v(2) != 0.0) 
    {
		int_e_v.setZero();
	}

	// 获取当前偏航角
	double yaw_curr = odom.yaw;
	// 期望偏航角
	double	yaw_des = des.yaw;
	// 坐标转换矩阵（仅根据当前yaw角生成）
	// 没看懂为什么要乘上wRc和cRw
	// 
	Eigen::Matrix3d wRc = uav_utils::rotz(yaw_curr);
	Eigen::Matrix3d cRw = wRc.transpose();

	// 位置误差
	e_p = des.p - odom.p;
	// p控制
	// e_p是ENU坐标系的,u_p也是ENU系的
	//  wRc * Kp * cRw * e_p =  Kp * e_p
	Eigen::Vector3d u_p = wRc * Kp * cRw * e_p;

	// 速度误差 = 期望速度  + 位置p控制 - 当前速度
	// 此处有点串级控制的意思
	e_v = des.v + u_p - odom.v;
	
	// 积分项？
	for (size_t k = 0; k < 3; ++k) 
    {
		if (std::fabs(e_v(k)) < 0.2) 
        {
			int_e_v(k) += e_v(k) * 1.0 / 50.0;
		}
	}

	// 速度比例控制
	Eigen::Vector3d u_v_p = wRc * Kv * cRw * e_v;

	// 速度积分控制
	Eigen::Vector3d u_v_i = wRc * Kvi * cRw * int_e_v;

	const std::vector<double> integration_output_limits = {0.4, 0.4, 0.4};
	for (size_t k = 0; k < 3; ++k) 
    {
		if (std::fabs(u_v_i(k)) > integration_output_limits[k]) 
        {
			uav_utils::limit_range(u_v_i(k), integration_output_limits[k]);
			ROS_INFO("Integration saturate for axis %zu, value=%.3f", k, u_v_i(k));
		}
	}

	// 速度控制
	Eigen::Vector3d u_v = u_v_p + u_v_i;

	// 偏航角误差
	e_yaw = yaw_des - yaw_curr;

	while(e_yaw > M_PI) e_yaw -= (2 * M_PI);
	while(e_yaw < -M_PI) e_yaw += (2 * M_PI);

	// 偏航教控制量
	double u_yaw = Kyaw * e_yaw;
	
	// 期望力 = 质量*控制量 + 重力抵消 + 期望加速度*质量*Ka
	F_des = u_v * Quad_MASS + Eigen::Vector3d(0, 0, Quad_MASS * gravity) + Ka * Quad_MASS * des.a;
	
	// 如果向上推力小于重力的一半
	// 或者向上推力大于重力的两倍
	if (F_des(2) < 0.5 * Quad_MASS * gravity)
	{
        ROS_INFO("thrust too low F_des(2)=%.3f",F_des(2));
		F_des = F_des / F_des(2) * (0.5 * Quad_MASS * gravity);
	}
	else if (F_des(2) > 2 * Quad_MASS * gravity)
	{
        ROS_INFO("thrust too high F_des(2)=%.3f",F_des(2));
		F_des = F_des / F_des(2) * (2 * Quad_MASS * gravity);
	}

	// 角度限制幅度
	if (std::fabs(F_des(0)/F_des(2)) > std::tan(uav_utils::toRad(25.0)))
	{
        ROS_INFO("roll agnle (%f) too tilt", uav_utils::toDeg(std::atan2(F_des(0),F_des(2))) );
		F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(uav_utils::toRad(25.0));
	}

	// 角度限制幅度
	if (std::fabs(F_des(1)/F_des(2)) > std::tan(uav_utils::toRad(25.0)))
	{
        ROS_INFO("pitch agnle (%f) too tilt", uav_utils::toDeg(std::atan2(F_des(1),F_des(2))) );
		F_des(1) = F_des(1)/std::fabs(F_des(1)) * F_des(2) * std::tan(uav_utils::toRad(25.0));	
	}

	// n3 api control in forward-left-up frame
    // F_des是位于ENU坐标系的,F_c是FLU
    Eigen::Vector3d F_c = wRc.transpose() * F_des;
    // 无人机姿态的矩阵形式
    Eigen::Matrix3d wRb_odom = odom.q.toRotationMatrix();
    // 第三列
    Eigen::Vector3d z_b_curr = wRb_odom.col(2);
    // 机体系下的推力合力 相当于Rb * F_enu 惯性系到机体系
    double u1 = F_des.dot(z_b_curr);
    double fx = F_c(0);
    double fy = F_c(1);
    double fz = F_c(2);
    // 期望roll和pitch，单位是弧度
    u_att(0)  = std::atan2(-fy, fz);
    u_att(1) = std::atan2( fx, fz);
    // 油门 = 期望推力/最大推力
    // full_thrust = mass * gra / hov_percent;
    // 这里相当于认为油门是线性的,满足某种比例关系,即认为某个重量 = 悬停油门
    // 悬停油门与电机参数有关系,也取决于质量
    u_att(2) = des.yaw;

    double full_thrust = Quad_MASS * gravity / hov_percent;
    u_att(3) = u1 / full_thrust;
}



#endif
