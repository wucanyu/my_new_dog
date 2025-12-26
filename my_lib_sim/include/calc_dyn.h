#pragma once
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "MJ_interface.h"

class Pin_KinDyn
{
public:
    //浮动基模型
    pinocchio::Model model_biped;
    //固定基模型
    pinocchio::Model model_biped_fixed;
    //用于存储机器人模型的广义速度维度
    int model_nv;
    int model_biped_nv;
    std::vector<bool> motorReachLimit;

    const std::vector<std::string> motorName = {"leg1_1_joint", "leg1_2_joint", "leg1_3_joint"
                                                "leg2_1_joint", "leg2_2_joint", "leg2_3_joint"
                                                "leg3_1_joint", "leg3_2_joint", "leg3_3_joint"
                                                "leg4_1_joint", "leg4_2_joint", "leg4_3_joint"}; 

    //上一次的目标力矩
    Eigen::VectorXd tauJointOld;
    //限制电机的参数
    Eigen::VectorXd motorMaxTorque;
    Eigen::VectorXd motorMaxPos;
    Eigen::VectorXd motorMinPos;
    //足端坐标的表示
    pinocchio::JointIndex FL_joint, FR_joint, BL_joint, BR_joint, base_joint;
    pinocchio::JointIndex FL_joint_fixed, FR_joint_fixed, BL_joint_fixed, BR_joint_fixed;
    //大腿坐标的表示
    pinocchio::JointIndex FL_hip_joint, FR_hip_joint, BL_hip_joint, BR_hip_joint;
    pinocchio::JointIndex FL_hip_joint_fixed, FR_hip_joint_fixed, BL_hip_joint_fixed, BR_hip_joint_fixed;
    //
    Eigen::VectorXd q, dq, ddq, q_fixed, dq_fixed;
    //旋转矩阵
    Eigen::Matrix3d Rcur;
    //现在的四元数
    Eigen::Quaternion<double> quatCur;
    //雅可比矩阵 世界
    Eigen::Matrix<double, 6, -1> J_FL, J_FR, J_BL, J_BR, J_base;
    //雅可比矩阵 身体
    Eigen::Matrix<double, 6, -1> J_FL_body, J_FR_body, J_BL_body, J_BR_body;
    Eigen::Matrix<double, 6, -1> dJ_FL, dJ_FR, dJ_BL, dJ_BR, dJ_base;
    Eigen::Matrix<double, 3, -1> Jcom;
    Eigen::Vector3d fe_fl_pos, fe_fr_pos, fe_bl_pos, fe_br_pos, base_pos; // foot-end position in world frame
    Eigen::Matrix3d fe_fl_rot, fe_fr_rot, fe_bl_rot, fe_br_rot, base_rot; 
    Eigen::Vector3d fe_fl_pos_body, fe_fr_pos_body, fe_bl_pos_body, fe_br_pos_body; // foot-end position in body frame
    Eigen::Vector3d fe_fl_vel_body, fe_fr_vel_body, fe_bl_vel_body, fe_br_vel_body; // foot-end velcity in body frame
    //大腿数据
    Eigen::Matrix<double, 6, -1> J_hip_FL_body, J_hip_FR_body, J_hip_BL_body, J_hip_BR_body;
    Eigen::Vector3d hip_fl_pos, hip_fr_pos, hip_bl_pos, hip_br_pos;
    Eigen::Vector3d hip_fl_vel, hip_fr_vel, hip_bl_vel, hip_br_vel;
    
    Eigen::MatrixXd dyn_M, dyn_M_inv, dyn_C, dyn_G, dyn_Ag, dyn_dAg;
    Eigen::VectorXd dyn_Non;
    Eigen::Vector3d CoM_pos;
    Eigen::Matrix3d inertia;

    struct IkRes
    {
        int status;
        int itr;
        Eigen::VectorXd err;
        Eigen::VectorXd jointPosRes;
    };

    Pin_KinDyn(std::string urdf_pathIn);
    //接收仿真数据
    void data_Read(const data_bus robotState);
    //计算机器人的正运动学,计算雅可比矩阵和雅可比矩阵的时间变化率。
    void computeJ_dJ();
    //计算机器人的动力学参数，如惯性矩阵 dyn_M、科里奥利力矩阵 dyn_C、重力向量 dyn_G 等。
    void computeDyn();
    //计算足端位置逆运动学
    IkRes computeInK_Leg(   const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, 
                            const Eigen::Matrix3d &Rdes_R, const Eigen::Vector3d &Pdes_R);
   
    Eigen::VectorXd integrateDIY(const Eigen::VectorXd &qI, const Eigen::VectorXd &dqI);
    static Eigen::Quaterniond intQuat(const Eigen::Quaterniond &quat, const Eigen::Matrix<double, 3, 1> &w);
    void workspaceConstraint(Eigen::VectorXd &qFT, Eigen::VectorXd &tauJointFT);
    void send_data(void);
    void update_data_to_MJ_interface(data_bus &robotState);
private:
    //用于存储运动学计算过程中的中间结果和最终结果
    pinocchio::Data data_biped, data_biped_fixed;

};
