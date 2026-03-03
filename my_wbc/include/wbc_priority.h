#pragma once

#include "qpOASES.hpp"
#include "MJ_interface.h"
#include <algorithm>
#include <Eigen/Dense>
#include "Utils.h"
#include "priority_tasks.h"
#include "calc_dyn.h"
#include <iostream>
#include <iomanip>

class WBC_priority 
{
    public:
        WBC_priority(int model_nv_In, int QP_nvIn, int QP_ncIn, double miu_In, double dt);
        // size of the system generalized coordinate dq
        int model_nv; 
        //控制状态
        int joy_cmd_ctrl_state;
        //
        double f_z_low, f_z_upp, mu;
        //动力学M矩阵，M的逆，质心动量矩阵，质心动量矩阵的时间导数
        Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_Ag, dyn_dAg;
        // dyn_Non= c*dq+g = b + g
        Eigen::VectorXd dyn_Non; 
        //四个腿的MPC足端支撑力
        Eigen::VectorXd Fr_ff; 

        //关节加速度与期望误差权重
        Eigen::MatrixXd Q1;
        //外部接触力与期望误差权重
        Eigen::MatrixXd Q2;

        //广义位置，广义速度，广义加速度原始数据
        Eigen::VectorXd q, dq, ddq;
        Eigen::VectorXd delta_q_final_kin, dq_final_kin, ddq_final_kin, tauJointRes;

        //所有支撑足雅可比
        Eigen::MatrixXd Jc, dJc;
        //所有摆动足雅可比
        Eigen::MatrixXd Jsw, dJsw;
        //所有足端雅可比
        Eigen::MatrixXd Jfe, dJfe;
        //所有足端雅可比
        Eigen::MatrixXd J_FR, J_FL,J_BR, J_BL;

        Eigen::Vector3d pCoMDes, pCoMCur;

        PriorityTasks kinwbc_tasks_walk, kinwbc_tasks_stand;

        void computeTau();
        void computeDdq(Pin_KinDyn &pinKinDynIn);
        void dataBusRead(const data_bus &robotState);
        void dataBusWrite(data_bus &robotState);
        void setQini(const Eigen::VectorXd &qIniDes, const Eigen::VectorXd &qIniCur);
        void copy_Eigen_to_real_t(qpOASES::real_t* target, const Eigen::MatrixXd &source, int nRows, int nCols);
    private:
        double timeStep{0.002};

        //6*18 上6*6为单位阵 浮动基座选择矩阵
        Eigen::MatrixXd Sf;
        //18*6 上6*6为单位阵
        Eigen::MatrixXd St_qpV1;
        //18*12 下12*12为单位阵
        Eigen::MatrixXd St_qpV2; 
        //足端是否触地
        bool contacts[4]; 
        //足端位置
        Eigen::Vector3d fe_pos_world[4];
        //位置，期望位置
        Eigen::Vector3d base_pos, base_pos_des;
        //角度，期望角度
        Eigen::Vector3d base_rpy_cur, base_rpy_des;
        //当前的旋转矩阵
        Eigen::Matrix3d base_rot;
        //摆动期望位置，摆动当前位置    
        Eigen::VectorXd swing_fe_pos_des_W,swing_fe_pos_cur_W;
        //摆动期望速度，摆动当前速度    
        Eigen::VectorXd swing_fe_vel_des_W,swing_fe_vel_cur_W;
        //摆动期望加速度
        Eigen::VectorXd swing_fe_acc_des_W;

        Eigen::VectorXd des_delta_q, des_q, des_dq, des_ddq;

        //初始化关节位置,期望位置
        Eigen::VectorXd qIniDes, qIniCur;

        /**********************二次规划相关变量************************** */
        //QP问题求解器
        qpOASES::QProblem QP_prob;
        //最大迭代次数
        qpOASES::int_t nWSR = 100, last_nWSR{0};
        //允许计算的最大时间
        qpOASES::real_t cpu_time = 0.1, last_cpu_time{0};

        int qpStatus{0};
        int QP_nv;
        int QP_nc;

        static const int QP_nv_des=18;
        static const int QP_nc_des=22;

        qpOASES::real_t qp_H[QP_nv_des*QP_nv_des];
        qpOASES::real_t qp_A[QP_nc_des*QP_nv_des];
        qpOASES::real_t qp_g[QP_nv_des];
        qpOASES::real_t qp_lbA[QP_nc_des];
        qpOASES::real_t qp_ubA[QP_nc_des];
        qpOASES::real_t xOpt_iniGuess[QP_nv_des];

        Eigen::VectorXd eigen_xOpt;
        Eigen::VectorXd eigen_ddq_Opt;
        Eigen::VectorXd eigen_tau_Opt;
        Eigen::VectorXd eigen_fr_Opt;
};


