#pragma once

#include <pid.h>
#include <eigen3/Eigen/Dense>

////足端编号
#define FL_FLAG (0)
#define FR_FLAG (1)
#define BL_FLAG (2)
#define BR_FLAG (3)

#define Xrw (0)
#define Yrw (1)
#define Zrw (2)

typedef struct 
{
	float x;
	float y;
	float z;
}END_POS;

typedef struct
{
	float x;
	float y;
	float z;
}Vect3;

//单腿信息结构体
typedef struct
{
	Vect3 tar_force_qp;
	Vect3 tar_force_qp_b;
	Vect3 epos_b;
	Vect3 epos_n;
	Vect3 spd_b;
	Vect3 spd_n;
    double is_ground;
}LegTypeDef;

typedef struct
{
	float roll;       //横滚，x轴
	float pitch;      //俯仰，y轴
	float yaw;        //偏航，z轴
}eulerAngleTypeDef;

typedef struct
{
    pid_type_def pitch_;
	pid_type_def roll_;
	pid_type_def yaw_;
	pid_type_def spdx_;
	pid_type_def x_;
	pid_type_def z_;
}walk_controller_TypeDef;

typedef struct
{
    float last_phase;
    float phase;
	int ground_num;
	
	int8_t vx;
	int8_t vz;

	LegTypeDef Leg[4];

	walk_controller_TypeDef trot;

	eulerAngleTypeDef   now_att;        		//机器人当前欧拉角
	eulerAngleTypeDef   now_rate;       		//机器机体欧拉角速度
	eulerAngleTypeDef   now_rate_n_withoutyaw;  //机器机体欧拉角速度
	
	Eigen::Vector3f	accel_b; 		//机体加速度
	Eigen::Vector3f accel_n; 		//世界加速度

	Vect3 exp_force;    //全局力
	Vect3 exp_torque;   //全局扭矩

	Eigen::Matrix3f Rn_b;
	Eigen::Matrix3f Rb_n;

	Eigen::Matrix3f Rn_b_only_yaw;
	Eigen::Matrix3f Rb_n_only_yaw;

	Eigen::Matrix3f Rn_b_without_yaw;
	Eigen::Matrix3f Rb_n_without_yaw;

	Eigen::Vector3f Centroid_position;
	Eigen::Vector3f Centroid_velocity;

}robotTypeDef;
