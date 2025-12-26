#ifndef CPP_CTRLSTATES_H
#define CPP_CTRLSTATES_H

#include <eigen3/Eigen/Dense>

#include "base_struct.h"
#include "mid_data.h"

class CtrlStates {
public:
    // this init function sets all variables to that used in orbit.issac.a1 controller
    CtrlStates() {
        reset();
    }

    void reset() {
        stance_leg_control_type = 1;
        use_terrain_adapt = 0;
        movement_mode = 0;

        root_pos_d.setZero();
        root_euler_d.setZero();
        root_lin_vel_d.setZero();
        root_ang_vel_d.setZero();
        
        robot_mass = 15.0;
        robot_time = 0;
        max_time = 1.0;
        max_time_half = max_time * 0.5;

        a1_trunk_inertia << 0.0076, 0.0, 0.0,
                            0.0, 0.0133, 0.0,
                            0.0, 0.0, 0.0185;

        default_foot_pos << 0.211, 0.211, -0.211, -0.211,
                            0.166, -0.166, 0.166, -0.166,
                            -0.25, -0.25, -0.25, -0.25;

        q_weights.resize(13);
        r_weights.resize(12);
        q_weights_trot.resize(13);
        r_weights_trot.resize(12);
        //MIT 
        // q_weights <<    0.25, 0.25, 10, 
        //                 2, 2, 50, 
        //                 0, 0, 0.3, 
        //                 0.2, 0.2, 0.1,
        //                 0.0;

        // r_weights <<    4e-5, 4e-5, 4e-5,
        //                 4e-5, 4e-5, 4e-5,
        //                 4e-5, 4e-5, 4e-5,
        //                 4e-5, 4e-5, 4e-5;

        //DEEPROBOTICS 
        // roll pitch yaw 
        // x y z 
        // wx wy wz 
        // vx vy vz
        q_weights_trot <<   20, 20, 5, 
                            60, 60, 120, 
                            0.5,0.5,10.0, 
                            5,  5,  1,
                            0.0;
        r_weights_trot <<   4e-6, 4e-6, 4e-6,
                            4e-6, 4e-6, 4e-6,
                            4e-6, 4e-6, 4e-6,
                            4e-6, 4e-6, 4e-6;

        q_weights << 20, 40, 5, 
                     40, 60, 100, 
                     0.0, 0, 0.5, 
                     5, 5, 1,
                     0.0;                     
        r_weights <<    4e-6, 4e-6, 4e-6,
                        4e-6, 4e-6, 4e-6,
                        4e-6, 4e-6, 4e-6,
                        4e-6, 4e-6, 4e-6;

        root_pos.setZero();
        root_quat.setIdentity();
        root_euler.setZero();
        root_rot_mat.setZero();
        root_rot_mat_z.setZero();
        root_lin_vel.setZero();
        root_ang_vel.setZero();

        foot_force.setZero();

        foot_pos_start_rel.setZero();
        foot_pos_start_abs.setZero();
        foot_pos_target_rel.setZero();
        foot_pos_target_abs.setZero(); 
        foot_pos_target_world.setZero(); 

        foot_pos_rel.setZero();
        foot_pos_abs.setZero();
        foot_pos_world.setZero();
        foot_pos_cur.setZero();

        foot_vel_rel.setZero();
        foot_vel_abs.setZero();
        foot_vel_world.setZero();

        foot_hip_pos_rel.setZero();
        foot_hip_pos_abs.setZero();
        foot_hip_vel_rel.setZero();
        foot_hip_vel_abs.setZero();

        
        for (int i = 0; i < NUM_LEG; ++i) {
            contacts[i] = false;
            plan_contacts[i] = false;
            early_contacts[i] = false;
        }

        // double kp_foot_x = 80.0;
        // double kp_foot_y = 80.0;
        // double kp_foot_z = 80.0;

        // double kd_foot_x = 0.8;
        // double kd_foot_y = 0.8;
        // double kd_foot_z = 0.8;
        double kp_foot_x = 300.0;
        double kp_foot_y = 300.0;
        double kp_foot_z = 300.0;

        double kd_foot_x = 10;
        double kd_foot_y = 10;
        double kd_foot_z = 10;

        kp_foot <<
                kp_foot_x, kp_foot_x, kp_foot_x, kp_foot_x,
                kp_foot_y, kp_foot_y, kp_foot_y, kp_foot_y,
                kp_foot_z, kp_foot_z, kp_foot_z, kp_foot_z;
        kd_foot <<
                kd_foot_x, kd_foot_x, kd_foot_x, kd_foot_x,
                kd_foot_y, kd_foot_y, kd_foot_y, kd_foot_y,
                kd_foot_z, kd_foot_z, kd_foot_z, kd_foot_z;

        kp_linear = Eigen::Vector3d(1000.0, 1000.0, 1000.0);
        kd_linear = Eigen::Vector3d(65.0, 70.0, 120.0);
        kp_angular = Eigen::Vector3d(40.0, 60.0, 1.0);
        kd_angular = Eigen::Vector3d(4.5, 4.5, 30.0);

        joint_torques.setZero();
    }

    //数据中转站
    robotTypeDef mid_data;

    // variables
    int stance_leg_control_type; // 0: QP, 1: MPC 选择控制方式
    int movement_mode;  // 0: standstill, 1: start to locomote 选择控制状态
    int use_terrain_adapt; //选择是否使用地形估计的标志位

    //机器人在机体坐标系下的期望位置
    Eigen::Vector3d root_pos_d;
    //机器人在机体坐标系下的期望角度
    Eigen::Vector3d root_euler_d;
    //机器人在机体坐标系下的期望线性速度
    Eigen::Vector3d root_lin_vel_d;
    //机器人在世界坐标系下的期望线性速度
    Eigen::Vector3d root_lin_vel_d_world;
    //机器人在机体坐标系下的期望线性角速度
    Eigen::Vector3d root_ang_vel_d;
    //机器人在世界坐标系下的期望线性角速度
    Eigen::Vector3d root_ang_vel_d_world;

    Eigen::Matrix<double, MPC_STATE_DIM, 1> mpc_states;
    //13*步数 保存每一次mpc预测的状态
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> mpc_states_d;

    //机器人重量
    double robot_mass;
    //机器人内部时间
    double max_time;
    double max_time_half;
    double robot_phase;
    double robot_phase_remain;
    double robot_time;
    double robot_time_half;
    double robot_time_half_remain;
    //惯性张量
    Eigen::Matrix3d a1_trunk_inertia;
    //摆动相计算用的默认足端位置
    Eigen::Matrix<double, 3, NUM_LEG> default_foot_pos;

    // MPC parameters 权重
    Eigen::VectorXd q_weights;
    Eigen::VectorXd r_weights;

    Eigen::VectorXd q_weights_trot;
    Eigen::VectorXd r_weights_trot;

    // 观测地形角度 默认为零
    double terrain_pitch_angle;  

    //卡尔曼估计出的机体位置 世界
    Eigen::Vector3d root_pos;
    //机体欧拉角 弧度    
    Eigen::Vector3d root_euler;
    //机体欧拉角 角度
    Eigen::Vector3d root_euler_360;
    //机体四元数   
    Eigen::Quaterniond root_quat;
    //旋转矩阵
    Eigen::Matrix3d root_rot_mat;
    //绕 Z 轴旋转的旋转矩阵
    Eigen::Matrix3d root_rot_mat_z;
    //卡尔曼估计出的世界坐标系速度
    Eigen::Vector3d root_lin_vel;
    //陀螺仪的世界坐标系角速度
    Eigen::Vector3d root_ang_vel;
    //测量得到的足底力
    Eigen::Vector4d foot_force;

    //存储摆动相的力
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin_world;
    //存储支撑相的力
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf_world;

    //摆动相位启动位置
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_start_rel;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_start_abs;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_start_world;
    //摆动相位结束位置  
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_end_rel;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_end_abs;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_end_world;
    //摆动相目标数值 
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_rel;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_abs; 
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_world; 
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target_rel;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target_abs; 
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target_world; 
    
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel; // 足端当前的位置，在机器人坐标系
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_abs; // 足端当前的位置，在世界坐标系
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_world; // 足端位置加上质心位置，在世界坐标系方向
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur; // 足端当前的位置，在世界坐标系，只有yaw

    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_rel; // 足端当前的速度，在机器人坐标系
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_abs; // 足端当前的速度，在世界坐标系
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_world; // 足端速度加上质心速度，在世界坐标系方向

    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel_last_time;   //足端上一时刻的位置
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_last_time;//足端上一时刻的目标位置
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_recent_contact;//记录足端最近的触地点，用于估计地形

    Eigen::Matrix<double, 3, NUM_LEG> foot_hip_pos_rel; // 大腿关节当前的位置，在机器人坐标系
    Eigen::Matrix<double, 3, NUM_LEG> foot_hip_pos_abs; // 大腿关节当前的位置，在世界坐标系
    Eigen::Matrix<double, 3, NUM_LEG> foot_hip_vel_rel; // 大腿关节当前的速度，在机器人坐标系
    Eigen::Matrix<double, 3, NUM_LEG> foot_hip_vel_abs; // 大腿关节当前的速度，在世界坐标系
    
    Eigen::Matrix<double, 6, 12>  J_FL_body;
    Eigen::Matrix<double, 6, 12>  J_FR_body; 
    Eigen::Matrix<double, 6, 12>  J_BL_body;
    Eigen::Matrix<double, 6, 12>  J_BR_body;
    Eigen::Matrix3d J3_FL_body;
    Eigen::Matrix3d J3_FR_body;
    Eigen::Matrix3d J3_BL_body;
    Eigen::Matrix3d J3_BR_body;

    bool contacts[NUM_LEG];         // flag to decide leg in the stance/swing mode
    bool early_contacts[NUM_LEG];   // true if foot hit objects during swing
    bool plan_contacts[NUM_LEG];    // planed flag for stance/swing mode

    // controller variables
    double kp_lin_x;
    double kd_lin_x;                                                                             
    double kf_lin_x;
    double kp_lin_y;
    double kd_lin_y;
    double kf_lin_y;

    //摆动相位参数
    Eigen::Matrix<double, NUM_DOF_PER_LEG, NUM_LEG> kp_foot;
    Eigen::Matrix<double, NUM_DOF_PER_LEG, NUM_LEG> kd_foot;

    //纯QP的vmc参数
    Eigen::Vector3d kp_linear;
    Eigen::Vector3d kd_linear;
    Eigen::Vector3d kp_angular;
    Eigen::Vector3d kd_angular;

    Eigen::Matrix<double, NUM_DOF, 1> joint_torques;
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques_swing;
    Eigen::Matrix<double, NUM_DOF, 1> joint_torques_out;

    // IMU sensor data
    Eigen::Vector3d imu_acc;
    Eigen::Vector3d imu_ang_vel;

    // state estimation true if the estimator thinks the foot has contact
    bool estimated_contacts[NUM_LEG];  
    // 卡尔曼估计出的质心位置
    Eigen::Vector3d estimated_root_pos;
    // 卡尔曼估计出的质心速度
    Eigen::Vector3d estimated_root_vel;

};

#endif //A1_CPP_A1CTRLSTATES_H
