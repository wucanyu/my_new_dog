#ifndef CPP_HARDWAREDOG_H
#define CPP_HARDWAREDOG_H

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
 
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <regex>
#include <string>

#include "pid.h"
#include "OsqpEigen/OsqpEigen.h"
#include "RobotControl.h"

// std
#include <eigen3/Eigen/Dense>
#include <memory>
#include <set>
#include <chrono>
#include <map>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <fstream>

// control parameters
#include "base_struct.h"
#include "CtrlStates.h"
#include "RobotControl.h"
#include "BasicEKF.h"
#include "Utils.h"
#include "my_usb.h"
#include "MJ_interface.h"
#include "GLFW_callbacks.h"

class HardwareDog {
public:
    HardwareDog();
    ~HardwareDog() {
        destruct = true;
    }

    bool update_foot_forces_grf(double dt);

    bool main_update(double dt,const UIctr &uiController,double t) ;

    void read_data(const data_bus &robotState,double t ,double start_t);

    void write_data(data_bus &robotState);

    CtrlStates ctrl_states;
private:
    bool destruct = false;
    std::thread thread_;

    // joystic command 遥控的指令
    double joy_cmd_velx = 0.0;
    double joy_cmd_vely = 0.0;
    double joy_cmd_velz = 0.0;
    double joy_cmd_roll_rate = 0.0;
    double joy_cmd_pitch_rate = 0.0;
    double joy_cmd_yaw_rate = 0.0;
    double joy_cmd_pitch_ang = 0.0;
    double joy_cmd_roll_ang = 0.0;
    double joy_cmd_body_height = 0.30;

    //  0 is standing, 1 is walking
    int joy_cmd_ctrl_state = 0;
    int prev_joy_cmd_ctrl_state = 0;
    bool joy_cmd_ctrl_state_change_request = false;

    // variables related to control and estimation
    //CtrlStates ctrl_states;
    RobotControl _root_control;
    BasicEKF estimate;
};


bool HardwareDog::update_foot_forces_grf(double dt) {
    ctrl_states.foot_forces_grf = _root_control.compute_grf(ctrl_states, dt);
    
    ctrl_states.joint_torques.block<3, 1>(0, 0) = -ctrl_states.J3_FR_body.transpose()  * ctrl_states.foot_forces_grf.block<3, 1>(0, 0);
    ctrl_states.joint_torques.block<3, 1>(3, 0) = -ctrl_states.J3_FL_body.transpose()  * ctrl_states.foot_forces_grf.block<3, 1>(0, 1);
    ctrl_states.joint_torques.block<3, 1>(6, 0) = -ctrl_states.J3_BR_body.transpose()  * ctrl_states.foot_forces_grf.block<3, 1>(0, 2);
    ctrl_states.joint_torques.block<3, 1>(9, 0) = -ctrl_states.J3_BL_body.transpose()  * ctrl_states.foot_forces_grf.block<3, 1>(0, 3);

    return true;
}

//更新指令
bool HardwareDog::main_update(double dt,const UIctr &uiController,double t) {
    joy_cmd_body_height = 0.3;
    if (joy_cmd_body_height >= JOY_CMD_BODY_HEIGHT_MAX) {
        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MAX;
    }
    if (joy_cmd_body_height <= JOY_CMD_BODY_HEIGHT_MIN) {
        joy_cmd_body_height = JOY_CMD_BODY_HEIGHT_MIN;
    }

    prev_joy_cmd_ctrl_state = joy_cmd_ctrl_state;
    if (joy_cmd_ctrl_state_change_request) {
        // toggle joy_cmd_ctrl_state
        joy_cmd_ctrl_state = joy_cmd_ctrl_state + 1;
        joy_cmd_ctrl_state = joy_cmd_ctrl_state % 2;
        joy_cmd_ctrl_state_change_request = false;
    }

    // root_lin_vel_d is in robot frame
    double filter = 0.1;
    ctrl_states.root_lin_vel_d[0] = ctrl_states.root_lin_vel_d[0] * (1 - filter) + joy_cmd_velx * 0.1;
    ctrl_states.root_lin_vel_d[1] = ctrl_states.root_lin_vel_d[1] * (1 - filter) + joy_cmd_vely * 0.1;
    // root_lin_vel_d_world is in world frame    
    ctrl_states.root_lin_vel_d_world = ctrl_states.root_rot_mat * ctrl_states.root_lin_vel_d;
    
    // static double rpy_int[2];
    // if(fabs(ctrl_states.root_lin_vel(1)) > 0.1)
    // {
    //     rpy_int[0] += dt*(0 - ctrl_states.root_euler(0))/ctrl_states.root_lin_vel(1);
    // }
    // if(fabs(ctrl_states.root_lin_vel(0)) > 0.2)   //avoid dividing by zero
    // {
    //     rpy_int[1] += dt*(0 - ctrl_states.root_euler(1))/ctrl_states.root_lin_vel(0);
    // }
    // rpy_int[0] = fminf(fmaxf(rpy_int[0], -0.25), 0.25);
    // rpy_int[1] = fminf(fmaxf(rpy_int[1], -0.25), 0.25);
    // ctrl_states.root_euler_d[1] = ctrl_states.root_lin_vel(0) * rpy_int[1];
    // ctrl_states.root_euler_d[0] = ctrl_states.root_lin_vel(1) * rpy_int[0];

    // root_ang_vel_d is in robot frame
    // ctrl_states.root_ang_vel_d[0] = joy_cmd_roll_rate;
    // ctrl_states.root_ang_vel_d[1] = joy_cmd_pitch_rate;
    ctrl_states.root_ang_vel_d[0] = 0;
    ctrl_states.root_ang_vel_d[1] = 0;
    ctrl_states.root_ang_vel_d[2] = joy_cmd_yaw_rate;
    // ctrl_states.root_euler_d[0] += joy_cmd_roll_rate * dt;
    // ctrl_states.root_euler_d[1] += joy_cmd_pitch_rate * dt;
    ctrl_states.root_euler_d[0] = 0;// 默认横滚角
    ctrl_states.root_euler_d[1] = 0;// 默认俯仰角
    ctrl_states.root_euler_d[2] += joy_cmd_yaw_rate * dt;
    ctrl_states.root_pos_d[2] = joy_cmd_body_height;


    // determine movement mode
    ctrl_states.early_contacts[0] = ctrl_states.contacts[0];
    ctrl_states.early_contacts[1] = ctrl_states.contacts[1]; 
    ctrl_states.early_contacts[2] = ctrl_states.contacts[2];
    ctrl_states.early_contacts[3] = ctrl_states.contacts[3];

    // determine movement mode
    if (joy_cmd_ctrl_state == 1) {
        ctrl_states.movement_mode = 1;
        if(ctrl_states.robot_time > 0.5 * ctrl_states.max_time)
        {
            ctrl_states.contacts[0] = true;
            ctrl_states.contacts[1] = false; 
            ctrl_states.contacts[2] = false;
            ctrl_states.contacts[3] = true;
        }
        else
        {
            ctrl_states.contacts[0] = false;
            ctrl_states.contacts[1] = true; 
            ctrl_states.contacts[2] = true;
            ctrl_states.contacts[3] = false;
        }
    } 
    else if (joy_cmd_ctrl_state == 0 && prev_joy_cmd_ctrl_state == 1) {
        ctrl_states.movement_mode = 0;
        ctrl_states.contacts[0] = true;
        ctrl_states.contacts[1] = true; 
        ctrl_states.contacts[2] = true;
        ctrl_states.contacts[3] = true;
    } else {
        ctrl_states.movement_mode = 0;
        ctrl_states.contacts[0] = true;
        ctrl_states.contacts[1] = true; 
        ctrl_states.contacts[2] = true;
        ctrl_states.contacts[3] = true;
    }

    

    // in walking mode, do position locking if no root_lin_vel_d, otherwise do not lock position
    if (ctrl_states.movement_mode == 1) {
        // has nonzero velocity, keep refreshing position target, but just xy
        // ctrl_states.root_pos_d(0) += dt * ctrl_states.root_lin_vel_d_world(0);
        // ctrl_states.root_pos_d(1) += dt * ctrl_states.root_lin_vel_d_world(1);
    }
    else if (ctrl_states.movement_mode == 0) {
        ctrl_states.root_pos_d(0) = -0.02;
        ctrl_states.root_pos_d(1) = 0;
    }


    _root_control.update_plan(ctrl_states, dt);
    _root_control.generate_swing_legs_ctrl(ctrl_states, dt);
    // _root_control.cout_data(ctrl_states);

    return true;
}

/*
    constructor
*/
HardwareDog::HardwareDog(){
    joy_cmd_ctrl_state = 0;
    joy_cmd_ctrl_state_change_request = false;
    prev_joy_cmd_ctrl_state = 0;
    _root_control = RobotControl();
    ctrl_states.reset();
}

void HardwareDog::read_data(const data_bus &robotState,double t ,double start_t){
    //仿真状态切换
    static int flag = 0;
    if(start_t < t && flag == 0)
    {
        joy_cmd_ctrl_state_change_request = true;
        flag = 1;
    }
    //身体速度指令
    joy_cmd_velx = 0;
    joy_cmd_vely = 0;
    joy_cmd_roll_rate = 0;
    joy_cmd_pitch_rate = 0;
    joy_cmd_yaw_rate = 0;      
    //时间 相位计算
    ctrl_states.robot_time += 0.002;
    if(ctrl_states.robot_time >= ctrl_states.max_time)
    {
        ctrl_states.robot_time = 0;
    }
    if(ctrl_states.robot_time > ctrl_states.max_time * 0.5)
    {
        ctrl_states.robot_time_half = ctrl_states.robot_time - ctrl_states.max_time * 0.5;
    }
    else
    {
        ctrl_states.robot_time_half = ctrl_states.robot_time;
    }

    if(ctrl_states.robot_time_half == 0)
    {
        ctrl_states.robot_phase = 0;
    }
    else
    {
        ctrl_states.robot_phase = ctrl_states.robot_time_half / 0.5;
    }
    ctrl_states.robot_time_half_remain = ctrl_states.max_time_half - ctrl_states.robot_time_half;
    ctrl_states.robot_phase_remain = 1 - ctrl_states.robot_phase;
    //基础数据
    ctrl_states.root_quat = robotState.quaternion;
    ctrl_states.root_rot_mat = ctrl_states.root_quat.toRotationMatrix();//w to b
    ctrl_states.root_euler = Utils::quat_to_euler(ctrl_states.root_quat);
    ctrl_states.root_euler_360(0) = ctrl_states.root_euler(0)*57.2f;
    ctrl_states.root_euler_360(1) = ctrl_states.root_euler(1)*57.2f;
    ctrl_states.root_euler_360(2) = ctrl_states.root_euler(2)*57.2f;
    double yaw_angle = ctrl_states.root_euler[2];

    ctrl_states.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());//b to w
    ctrl_states.imu_acc     = robotState.imu_acc;
    ctrl_states.imu_ang_vel = robotState.imu_ang_vel;
    ctrl_states.root_ang_vel = ctrl_states.root_rot_mat * ctrl_states.imu_ang_vel;

    if (!estimate.is_inited()) {
        estimate.init_state(ctrl_states);
    } else {
        estimate.update_estimation(ctrl_states,0.002);
    }

    // use estimation pos and vel to get foot pos and foot vel in world frame
    for (int i = 0; i < NUM_LEG; ++i) {
        //给足端位置赋值
        ctrl_states.foot_pos_rel_last_time.block<3, 1>(0, i) = ctrl_states.foot_pos_rel.block<3, 1>(0, i);
        ctrl_states.foot_pos_rel.block<3, 1>(0, i) = robotState.fe_pos_body[i];
        ctrl_states.foot_vel_rel.block<3, 1>(0, i) = robotState.fe_spd_body[i];

        //解算世界坐标系下的数据
        ctrl_states.foot_pos_abs.block<3, 1>(0, i) = ctrl_states.root_rot_mat * ctrl_states.foot_pos_rel.block<3, 1>(0, i);
        ctrl_states.foot_vel_abs.block<3, 1>(0, i) = ctrl_states.root_rot_mat * ctrl_states.foot_vel_rel.block<3, 1>(0, i);
        // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
        // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
        // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
        ctrl_states.foot_pos_world.block<3, 1>(0, i) = ctrl_states.foot_pos_abs.block<3, 1>(0, i) + ctrl_states.root_pos;
        ctrl_states.foot_vel_world.block<3, 1>(0, i) = ctrl_states.foot_vel_abs.block<3, 1>(0, i) + ctrl_states.root_lin_vel;
    }

    //给足端雅可比赋值
    ctrl_states.J_FR_body = robotState.J_FR_body; 
    ctrl_states.J_FL_body = robotState.J_FL_body;
    ctrl_states.J_BR_body = robotState.J_BR_body;
    ctrl_states.J_BL_body = robotState.J_BL_body;

    ctrl_states.J3_FR_body = robotState.J_FR_body.block<3,3>(0,0);
    ctrl_states.J3_FL_body = robotState.J_FL_body.block<3,3>(0,3);
    ctrl_states.J3_BR_body = robotState.J_BR_body.block<3,3>(0,6);
    ctrl_states.J3_BL_body = robotState.J_BL_body.block<3,3>(0,9);

}
 
void HardwareDog::write_data(data_bus &robotState)
{
    robotState.joy_cmd_ctrl_state = joy_cmd_ctrl_state;

    robotState.base_rpy_des = ctrl_states.root_euler_d;
    robotState.base_pos_des = ctrl_states.root_pos_d;

    for (int i = 0; i < 4; i++)
    {   
        robotState.contacts[i] = ctrl_states.contacts[i]; 
    }
 
    for (int i = 0; i < 4; i++)
    {  
        robotState.fe_pos_world[i] = ctrl_states.foot_pos_world.block<3, 1>(0, i);
    }

    for (int i = 0; i < 4; i++)
    {  
        robotState.fe_pos_world_des[i] = ctrl_states.foot_pos_target_world.block<3, 1>(0, i);
    }

    robotState.Fr_ff = Eigen::VectorXd::Zero(12);
    robotState.Fr_ff.block<3, 1>(0, 0) = ctrl_states.foot_forces_grf_world.block<3, 1>(0, 0);
    robotState.Fr_ff.block<3, 1>(3, 0) = ctrl_states.foot_forces_grf_world.block<3, 1>(0, 1);
    robotState.Fr_ff.block<3, 1>(6, 0) = ctrl_states.foot_forces_grf_world.block<3, 1>(0, 2);
    robotState.Fr_ff.block<3, 1>(9, 0) = ctrl_states.foot_forces_grf_world.block<3, 1>(0, 3);

    robotState.des_ddq = Eigen::VectorXd::Zero(18);
    robotState.des_dq = Eigen::VectorXd::Zero(18);
    robotState.des_delta_q = Eigen::VectorXd::Zero(18);

    //mpc算出来的xy方向的加速度速度
    robotState.des_ddq.block<2, 1>(0, 0) << 0, 0;
    //mpc算出来的xy方向的位置
    robotState.des_dq.block<3, 1>(0, 0) << ctrl_states.root_lin_vel_d_world[0], ctrl_states.root_lin_vel_d_world[1], 0;
    //
    robotState.des_delta_q.block<2, 1>(0, 0) = robotState.des_dq.block<2, 1>(0, 0) * 0.002;

}
 

#endif //CPP_HARDWAREA1ROS_H
