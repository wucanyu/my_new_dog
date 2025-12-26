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
#include "mid_data.h"

// one of the most important thing: A1 hardware return info with FR, FL, RR, RL order and receives info in this order
// we need to take of this order in this function
class HardwareDog {
public:
    HardwareDog();

    ~HardwareDog() {
        destruct = true;
        thread_.join();
    }

    bool update_foot_forces_grf(double dt);

    bool main_update(double t,double dt);

    void result_to_mid_data(double dt);

    bool send_data();

private:
    //hardware reading thread
    std::thread thread_;
    void receive_low_state();
    bool destruct = false;

    //hardware switch foot order
    Eigen::Matrix<int, NUM_DOF, 1> swap_joint_indices;
    Eigen::Matrix<int, NUM_LEG, 1> swap_foot_indices;

    //hardware foot force filter
    Eigen::Matrix<int, NUM_LEG, 1> foot_force_filters_idx;
    Eigen::Matrix<double, NUM_LEG, 1> foot_force_filters_sum;


    // joystic command 遥控的指令
    double joy_cmd_velx = 0.0;
    double joy_cmd_vely = 0.0;
    double joy_cmd_velz = 0.0;
    double joy_cmd_roll_rate = 0.0;
    double joy_cmd_pitch_rate = 0.0;
    double joy_cmd_yaw_rate = 0.0;
    double joy_cmd_pitch_ang = 0.0;
    double joy_cmd_roll_ang = 0.0;
    double joy_cmd_body_height = 0.5;

    //  0 is standing, 1 is walking
    int joy_cmd_ctrl_state = 0;
    int prev_joy_cmd_ctrl_state = 0;
    bool joy_cmd_ctrl_state_change_request = false;

    //接受下位机的数据
    DOGUART dog_usb;
    // variables related to control and estimation
    CtrlStates ctrl_states;
    RobotControl _root_control;
    BasicEKF estimate;
};

bool HardwareDog::send_data() {
    float data[8];
    data[0] = ctrl_states.foot_forces_grf(0);
    data[1] = ctrl_states.foot_forces_grf(2);
    data[2] = ctrl_states.foot_forces_grf(3);
    data[3] = ctrl_states.foot_forces_grf(5);
    data[4] = ctrl_states.foot_forces_grf(6);
    data[5] = ctrl_states.foot_forces_grf(8);
    data[6] = ctrl_states.foot_forces_grf(9);
    data[7] = ctrl_states.foot_forces_grf(11);
    dog_usb.send_usb(data,8);
    return true;
} 

bool HardwareDog::update_foot_forces_grf(double dt) {
    ctrl_states.foot_forces_grf = _root_control.compute_grf(ctrl_states, dt);
    _root_control.cout_data(ctrl_states);
    return true;
}
/*
    将usb读到的数据放到mid_data里
*/
void HardwareDog::result_to_mid_data(double dt)
{
    ctrl_states.mid_data.now_att.roll  = dog_usb.getArrayElement(0);
    ctrl_states.mid_data.now_att.pitch = dog_usb.getArrayElement(1);
    ctrl_states.mid_data.now_att.yaw   = dog_usb.getArrayElement(2);

    ctrl_states.mid_data.now_rate.roll = dog_usb.getArrayElement(3);   
    ctrl_states.mid_data.now_rate.pitch= dog_usb.getArrayElement(4);
    ctrl_states.mid_data.now_rate.yaw  = dog_usb.getArrayElement(5);    

    ctrl_states.mid_data.Leg[0].epos_b.x = dog_usb.getArrayElement(6);
    ctrl_states.mid_data.Leg[0].epos_b.y = 0.153;
    ctrl_states.mid_data.Leg[0].epos_b.z = dog_usb.getArrayElement(7);

    ctrl_states.mid_data.Leg[1].epos_b.x = dog_usb.getArrayElement(8);
    ctrl_states.mid_data.Leg[1].epos_b.y = -0.153;
    ctrl_states.mid_data.Leg[1].epos_b.z = dog_usb.getArrayElement(9);

    ctrl_states.mid_data.Leg[2].epos_b.x = dog_usb.getArrayElement(10);
    ctrl_states.mid_data.Leg[2].epos_b.y = 0.153;
    ctrl_states.mid_data.Leg[2].epos_b.z = dog_usb.getArrayElement(11);

    ctrl_states.mid_data.Leg[3].epos_b.x = dog_usb.getArrayElement(12);
    ctrl_states.mid_data.Leg[3].epos_b.y = -0.153;
    ctrl_states.mid_data.Leg[3].epos_b.z = dog_usb.getArrayElement(13);

    ctrl_states.mid_data.Leg[0].spd_b.x = dog_usb.getArrayElement(14);
    ctrl_states.mid_data.Leg[0].spd_b.y = 0;
    ctrl_states.mid_data.Leg[0].spd_b.z = dog_usb.getArrayElement(15);

    ctrl_states.mid_data.Leg[1].spd_b.x = dog_usb.getArrayElement(16);
    ctrl_states.mid_data.Leg[1].spd_b.y = 0;
    ctrl_states.mid_data.Leg[1].spd_b.z = dog_usb.getArrayElement(17);

    ctrl_states.mid_data.Leg[2].spd_b.x = dog_usb.getArrayElement(18);
    ctrl_states.mid_data.Leg[2].spd_b.y = 0;
    ctrl_states.mid_data.Leg[2].spd_b.z = dog_usb.getArrayElement(19);

    ctrl_states.mid_data.Leg[3].spd_b.x = dog_usb.getArrayElement(20);
    ctrl_states.mid_data.Leg[3].spd_b.y = 0;
    ctrl_states.mid_data.Leg[3].spd_b.z = dog_usb.getArrayElement(21);

    ctrl_states.mid_data.last_phase = ctrl_states.mid_data.phase;
    ctrl_states.mid_data.phase = dog_usb.getArrayElement(22);

    ctrl_states.mid_data.accel_b(0) = dog_usb.getArrayElement(23);
    ctrl_states.mid_data.accel_b(1) = dog_usb.getArrayElement(24);
    ctrl_states.mid_data.accel_b(2) = dog_usb.getArrayElement(25);

    ctrl_states.mid_data.vx = dog_usb.get_rxData(105);
    ctrl_states.mid_data.vz = dog_usb.get_rxData(106);

    //根据相位的足端开环触地效果
    if(ctrl_states.mid_data.phase < 0)
    {
        for (int i = 0; i < NUM_LEG; ++i) 
        {
            ctrl_states.mid_data.Leg[i].is_ground = 1;
        }
    }
    else
    {
        if(ctrl_states.mid_data.phase <= 0.5)
        {
            ctrl_states.mid_data.Leg[FL_FLAG].is_ground = 1;
            ctrl_states.mid_data.Leg[FR_FLAG].is_ground = 0;
            ctrl_states.mid_data.Leg[BL_FLAG].is_ground = 0;
            ctrl_states.mid_data.Leg[BR_FLAG].is_ground = 1;
        }
        else
        {
            ctrl_states.mid_data.Leg[FL_FLAG].is_ground = 0;
            ctrl_states.mid_data.Leg[FR_FLAG].is_ground = 1;
            ctrl_states.mid_data.Leg[BL_FLAG].is_ground = 1;
            ctrl_states.mid_data.Leg[BR_FLAG].is_ground = 0;             
        }
    }    

    for (int i = 0; i < NUM_LEG; ++i) 
    {
        ctrl_states.contacts[i] = ctrl_states.mid_data.Leg[i].is_ground;
    }    

}
/*
    更新控制指令
*/
bool HardwareDog::main_update(double t,double dt) {
    // process joy cmd data to get desired height, velocity, yaw, etc
    // save the result into ctrl_states
    // joy_cmd_body_height += joy_cmd_velz * dt;

    joy_cmd_body_height = 0.20;
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
    ctrl_states.root_lin_vel_d[0] = joy_cmd_velx;
    ctrl_states.root_lin_vel_d[1] = joy_cmd_vely;

    // root_ang_vel_d is in robot frame
    ctrl_states.root_ang_vel_d[0] = joy_cmd_roll_rate;
    ctrl_states.root_ang_vel_d[1] = joy_cmd_pitch_rate;
    ctrl_states.root_ang_vel_d[2] = joy_cmd_yaw_rate;
    ctrl_states.root_euler_d[0] += joy_cmd_roll_rate * dt;
    ctrl_states.root_euler_d[1] += joy_cmd_pitch_rate * dt;
    ctrl_states.root_euler_d[2] += joy_cmd_yaw_rate * dt;
    ctrl_states.root_pos_d[2] = joy_cmd_body_height;

    // determine movement mode
    if (joy_cmd_ctrl_state == 1) {
        // in walking mode, in this mode the robot should execute gait
        ctrl_states.movement_mode = 1;
    } else if (joy_cmd_ctrl_state == 0 && prev_joy_cmd_ctrl_state == 1) {
        // leave walking mode
        // lock current position, should just happen for one instance
        ctrl_states.movement_mode = 0;
        // ctrl_states.root_pos_d.segment<2>(0) = ctrl_states.root_pos.segment<2>(0);
    } else {
        ctrl_states.movement_mode = 0;
    }

    // in walking mode, do position locking if no root_lin_vel_d, otherwise do not lock position
    if (ctrl_states.movement_mode == 1) {
        if (ctrl_states.root_lin_vel_d.segment<2>(0).norm() > 0.05) {
            // has nonzero velocity, keep refreshing position target, but just xy
            // ctrl_states.root_pos_d.segment<2>(0) = ctrl_states.root_pos.segment<2>(0);
            ctrl_states.kp_linear.segment<2>(0).setZero();
        } else {

        }
    }

    _root_control.update_plan(ctrl_states, dt);
    _root_control.generate_swing_legs_ctrl(ctrl_states, dt);

    return true;
}
/*
    接受硬件底层数据
*/
void HardwareDog::receive_low_state() {
    dog_usb.scan_usb();
    auto start = std::chrono::high_resolution_clock::now();
    auto prev = start;
    while (destruct == false) {
        dog_usb.receive_usb();
        // get t and dt
        auto now = std::chrono::high_resolution_clock::now();
        auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(now - prev);
        prev = now;
        std::cout << "dt: " << dt.count() << std::endl;
        result_to_mid_data(dt.count());

        if(ctrl_states.mid_data.last_phase == -1 && ctrl_states.mid_data.phase != -1)
        {
            joy_cmd_ctrl_state_change_request = true;
        }
        else if(ctrl_states.mid_data.last_phase != -1 && ctrl_states.mid_data.phase == -1)
        {
            joy_cmd_ctrl_state_change_request = true;
        }

        joy_cmd_velx = ctrl_states.mid_data.vx * 0.005;
        joy_cmd_vely = 0;
        joy_cmd_roll_rate = 0;
        joy_cmd_pitch_rate = 0;
        joy_cmd_yaw_rate = ctrl_states.mid_data.vz * 0.01;      

        // fill data to ctrl_states, notice the order in ctrl_states is FL, FR, RL, RR
        Eigen::AngleAxisd rollAngle (Eigen::AngleAxisd(ctrl_states.mid_data.now_att.roll,Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(ctrl_states.mid_data.now_att.pitch,Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle  (Eigen::AngleAxisd(ctrl_states.mid_data.now_att.yaw,Eigen::Vector3d::UnitZ()));
        
        Eigen::Quaterniond quaternion;
        quaternion=yawAngle*pitchAngle*rollAngle;
        ctrl_states.root_quat = quaternion;
        ctrl_states.root_rot_mat = ctrl_states.root_quat.toRotationMatrix();//w to b
        ctrl_states.root_euler = Utils::quat_to_euler(ctrl_states.root_quat);
        ctrl_states.root_euler_360(0) = ctrl_states.root_euler(0)*57.2f;
        ctrl_states.root_euler_360(1) = ctrl_states.root_euler(1)*57.2f;
        ctrl_states.root_euler_360(2) = ctrl_states.root_euler(2)*57.2f;
        double yaw_angle = ctrl_states.root_euler[2];

        ctrl_states.root_rot_mat_z = Eigen::AngleAxisd(yaw_angle, Eigen::Vector3d::UnitZ());//b to w
        ctrl_states.imu_acc     = Eigen::Vector3d(ctrl_states.mid_data.accel_b(0), ctrl_states.mid_data.accel_b(1), ctrl_states.mid_data.accel_b(2));
        ctrl_states.imu_ang_vel = Eigen::Vector3d(ctrl_states.mid_data.now_rate.roll, ctrl_states.mid_data.now_rate.pitch, ctrl_states.mid_data.now_rate.yaw);
        ctrl_states.root_ang_vel = ctrl_states.root_rot_mat * ctrl_states.imu_ang_vel;

        if (!estimate.is_inited()) {
            estimate.init_state(ctrl_states);
        } else {
            estimate.update_estimation(ctrl_states, dt.count());
        }

        //joy_cmd_velx 输入的x
        //joy_cmd_velz 输入的z
        //根据相位改变修改 joy_cmd_ctrl_state_change_request 的标志位

        // FL, FR, RL, RR
        // use estimation pos and vel to get foot pos and foot vel in world frame
        for (int i = 0; i < NUM_LEG; ++i) {
            //给足端位置赋值
            ctrl_states.foot_pos_rel.block<3, 1>(0, i) =  Eigen::Vector3d(  ctrl_states.mid_data.Leg[i].epos_b.x, 
                                                                            ctrl_states.mid_data.Leg[i].epos_b.y, 
                                                                            ctrl_states.mid_data.Leg[i].epos_b.z);

            ctrl_states.foot_vel_rel.block<3, 1>(0, i) = Eigen::Vector3d(   ctrl_states.mid_data.Leg[i].spd_b.x, 
                                                                            ctrl_states.mid_data.Leg[i].spd_b.y, 
                                                                            ctrl_states.mid_data.Leg[i].spd_b.z);

            //解算世界坐标系下的数据
            ctrl_states.foot_pos_abs.block<3, 1>(0, i) = ctrl_states.root_rot_mat * ctrl_states.foot_pos_rel.block<3, 1>(0, i);
            ctrl_states.foot_vel_abs.block<3, 1>(0, i) = ctrl_states.root_rot_mat * ctrl_states.foot_vel_rel.block<3, 1>(0, i);
            // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
            // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
            // !!!!!!!!!!! notice we use estimation pos and vel here !!!!!!!!!!!!!!!!!!!!!!!!
            ctrl_states.foot_pos_world.block<3, 1>(0, i) = ctrl_states.foot_pos_abs.block<3, 1>(0, i) + ctrl_states.root_pos;
            ctrl_states.foot_vel_world.block<3, 1>(0, i) = ctrl_states.foot_vel_abs.block<3, 1>(0, i) + ctrl_states.root_lin_vel;
        }

        float data[8];
        for (int i = 0; i < NUM_LEG; ++i) {
            if(ctrl_states.contacts[i] == 1)
            {
                data[0 + 2 * i] = ctrl_states.foot_forces_grf(0 + 3 * i);
                data[1 + 2 * i] = ctrl_states.foot_forces_grf(2 + 3 * i);
            }
            else
            {
                data[0 + 2 * i] = ctrl_states.foot_forces_kin(0 + 3 * i);
                data[1 + 2 * i] = ctrl_states.foot_forces_kin(2 + 3 * i);              
            }
        }

        dog_usb.send_usb(data,8);
    };
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

    //init swap order, very important
    swap_joint_indices << 3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8;
    swap_foot_indices << 1, 0, 3, 2;

    // start hardware reading thread after everything initialized 硬件接受数据的线程
    thread_ = std::thread(&HardwareDog::receive_low_state, this);
}


#endif //CPP_HARDWAREA1ROS_H
