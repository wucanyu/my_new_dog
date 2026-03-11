#include "calc_dyn.h"
#include <utility>

Pin_KinDyn::Pin_KinDyn(std::string urdf_pathIn)
{
    pinocchio::JointModelFreeFlyer root_joint;
    //创建模型
    pinocchio::urdf::buildModel(urdf_pathIn, root_joint, model_biped);
    pinocchio::urdf::buildModel(urdf_pathIn, model_biped_fixed);
    //初始化数据存储
    data_biped = pinocchio::Data(model_biped);
    data_biped_fixed = pinocchio::Data(model_biped_fixed); 
    model_nv = model_biped.nv;
    model_biped_nv = model_biped_fixed.nv;
    //代码里定义了多个雅可比矩阵（J_ 开头）和雅可比矩阵的时间导数（dJ_ 开头），并将它们初始化为零矩阵。
    //雅可比矩阵用于描述机器人关节空间和任务空间之间的线性映射关系，在机器人运动学和动力学计算中十分关键
    J_FL = Eigen::MatrixXd::Zero(6, model_nv);//12
    J_FR = Eigen::MatrixXd::Zero(6, model_nv);
    J_BL = Eigen::MatrixXd::Zero(6, model_nv);
    J_BR = Eigen::MatrixXd::Zero(6, model_nv);
    J_base = Eigen::MatrixXd::Zero(6, model_nv);

    dJ_FL = Eigen::MatrixXd::Zero(6, model_nv);//12
    dJ_FR = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_BL = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_BR = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_base = Eigen::MatrixXd::Zero(6, model_nv);

    dJ_FL = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_FR = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_BL = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_BR = Eigen::MatrixXd::Zero(6, model_nv);
    dJ_base = Eigen::MatrixXd::Zero(6, model_nv);

    J_FL_body = Eigen::MatrixXd::Zero(6, model_biped_nv);//12
    J_FR_body = Eigen::MatrixXd::Zero(6, model_biped_nv);
    J_BL_body = Eigen::MatrixXd::Zero(6, model_biped_nv);
    J_BR_body = Eigen::MatrixXd::Zero(6, model_biped_nv);

    J_hip_FL_body = Eigen::MatrixXd::Zero(6, model_biped_nv);
    J_hip_FR_body = Eigen::MatrixXd::Zero(6, model_biped_nv);
    J_hip_BL_body = Eigen::MatrixXd::Zero(6, model_biped_nv);
    J_hip_BR_body = Eigen::MatrixXd::Zero(6, model_biped_nv);

    q.resize(model_nv+1);    q.setZero();
    dq.resize(model_nv);    dq.setZero();
    ddq.resize(model_nv);    ddq.setZero();
    q_fixed.resize(model_biped_nv);    q_fixed.setZero();
    dq_fixed.resize(model_biped_nv);    dq_fixed.setZero();
    Rcur.setIdentity();

    dyn_M = Eigen::MatrixXd::Zero(model_nv, model_nv);
    dyn_M_inv = Eigen::MatrixXd::Zero(model_nv, model_nv);
    dyn_C = Eigen::MatrixXd::Zero(model_nv, model_nv);
    dyn_G = Eigen::MatrixXd::Zero(model_nv, 1);

    // get joint index for Pinocchio Lib, need to redefined the joint name for new model
    FR_joint = model_biped.getFrameId("leg1_joint4");
    FL_joint = model_biped.getFrameId("leg2_joint4");
    BR_joint = model_biped.getFrameId("leg3_joint4");
    BL_joint = model_biped.getFrameId("leg4_joint4");
    base_joint = model_biped.getJointId("root_joint");
    FR_joint_fixed = model_biped_fixed.getFrameId("leg1_joint4");
    FL_joint_fixed = model_biped_fixed.getFrameId("leg2_joint4");
    BR_joint_fixed = model_biped_fixed.getFrameId("leg3_joint4");
    BL_joint_fixed = model_biped_fixed.getFrameId("leg4_joint4");

    FL_hip_joint = model_biped.getJointId("leg2_joint2");
    FR_hip_joint = model_biped.getJointId("leg1_joint2");
    BL_hip_joint = model_biped.getJointId("leg4_joint2");
    BR_hip_joint = model_biped.getJointId("leg3_joint2");
    FL_hip_joint_fixed = model_biped_fixed.getJointId("leg2_joint2");
    FR_hip_joint_fixed = model_biped_fixed.getJointId("leg1_joint2");
    BL_hip_joint_fixed = model_biped_fixed.getJointId("leg4_joint2");
    BR_hip_joint_fixed = model_biped_fixed.getJointId("leg3_joint2");

    //设置电机数据
    motorMaxTorque = Eigen::VectorXd::Zero(motorName.size());
    motorMaxPos = Eigen::VectorXd::Zero(motorName.size());
    motorMinPos = Eigen::VectorXd::Zero(motorName.size());
    for (int i = 0; i < motorName.size(); i++)
    {
        motorMaxTorque(i) = 19.94;
        motorMaxPos(i) = 3.1415;
        motorMinPos(i) = -3.1415;
    }

    //假定所有电机都未达到限制条件
    motorReachLimit.assign(motorName.size(), false);
    //假定所有力矩
    tauJointOld = Eigen::VectorXd::Zero(motorName.size());
}

void Pin_KinDyn::data_Read(const data_bus robotState)
{
    //  For Pinocchio: The base translation part is expressed in the parent frame (here the world coordinate system)
    //  while its velocity is expressed in the body coordinate system.
    //  https://github.com/stack-of-tasks/pinocchio/issues/1137
    //  q = [global_base_position, global_base_quaternion, joint_positions]
    //  v = [local_base_velocity_linear, local_base_velocity_angular, joint_velocities]
    q = robotState.q;   // 关节位置（含浮动基）
    dq = robotState.dq; // 关节速度（含浮动基）
    // 转换浮动基速度到本体坐标系
    dq.block(0, 0, 3, 1) = robotState.base_rot.transpose() * robotState.dq.block(0, 0, 3, 1);
    dq.block(3, 0, 3, 1) = robotState.base_rot.transpose() * robotState.dq.block(3, 0, 3, 1);
    ddq = robotState.ddq;
    // std::cout << robotState.q << std::endl;  
    // std::cout << robotState.dq << std::endl;  
    int size_q_fixed = robotState.q.size();//19
    q_fixed = robotState.q.block(7,0,12,1);
    int size_dq_fixed = robotState.dq.size();
    dq_fixed = robotState.dq.block(6, 0, 12, 1);
}

// update jacobians and joint positions
void Pin_KinDyn::computeJ_dJ()
{
    // 1. 正运动学：必须传入q+dq（为雅可比时变率铺垫）
    pinocchio::forwardKinematics(model_biped, data_biped, q, dq); 
    // 2. 核心：计算基础关节雅可比矩阵（getJointJacobian的前置依赖，必须加！）
    pinocchio::computeJointJacobians(model_biped, data_biped);
    // 3. 更新全局位姿：替换为新版接口
    pinocchio::updateFramePlacements(model_biped, data_biped); 
    // 4. 质心雅可比：移到全局位姿更新后（保证质心计算用最新位姿）
    pinocchio::crba(model_biped, data_biped, q);
    //pinocchio::LOCAL_WORLD_ALIGNED 用于指定雅可比矩阵的参考坐标系。LOCAL_WORLD_ALIGNED 表示雅可比矩阵的列向量是相对于世界坐标系的方向
    pinocchio::getFrameJacobian(model_biped, data_biped, FR_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_FR);
    pinocchio::getFrameJacobian(model_biped, data_biped, FL_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_FL);
    pinocchio::getFrameJacobian(model_biped, data_biped, BR_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_BR);
    pinocchio::getFrameJacobian(model_biped, data_biped, BL_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_BL);
    //pinocchio::LOCAL_WORLD_ALIGNED 用于指定雅可比矩阵的参考坐标系。LOCAL_WORLD_ALIGNED 表示雅可比矩阵的列向量是相对于世界坐标系的方向
    pinocchio::getFrameJacobianTimeVariation(model_biped, data_biped, FR_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_FR);
    pinocchio::getFrameJacobianTimeVariation(model_biped, data_biped, FL_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_FL);
    pinocchio::getFrameJacobianTimeVariation(model_biped, data_biped, BR_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_BR);
    pinocchio::getFrameJacobianTimeVariation(model_biped, data_biped, BL_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_BL);
    //计算质心的雅可比
    pinocchio::getJointJacobian(model_biped, data_biped, base_joint, pinocchio::LOCAL_WORLD_ALIGNED, J_base);
    pinocchio::getJointJacobianTimeVariation(model_biped, data_biped, base_joint, pinocchio::LOCAL_WORLD_ALIGNED, dJ_base);
    // 质心雅可比矩阵描述了关节速度和质心线速度之间的线性映射关系。
    // 借助质心雅可比矩阵，能够把关节空间的速度信息转换为质心的线速度信息
    Jcom = data_biped.Jcom;

    // transform into world frame, and accept dq that in world frame
    Eigen::MatrixXd Mpj; 
    Mpj = Eigen::MatrixXd::Identity(model_nv, model_nv);
    Mpj.block(0, 0, 3, 3) = base_rot.transpose();
    Mpj.block(3, 3, 3, 3) = base_rot.transpose();
    J_FL = J_FL * Mpj;
    J_FR = J_FR * Mpj;
    J_BL = J_BL * Mpj;
    J_BR = J_BR * Mpj;
    J_base = J_base * Mpj;
    dJ_FL = dJ_FL * Mpj;
    dJ_FR = dJ_FR * Mpj;
    dJ_BL = dJ_BL * Mpj;
    dJ_BR = dJ_BR * Mpj;
    dJ_base = dJ_base * Mpj;
    Jcom = Jcom * Mpj;

    fe_fl_pos = data_biped.oMf[FL_joint].translation();
    fe_fl_rot = data_biped.oMf[FL_joint].rotation();
    fe_fr_pos = data_biped.oMf[FR_joint].translation();
    fe_fr_rot = data_biped.oMf[FR_joint].rotation();
    fe_bl_pos = data_biped.oMf[BL_joint].translation();
    fe_bl_rot = data_biped.oMf[BL_joint].rotation();
    fe_br_pos = data_biped.oMf[BR_joint].translation();
    fe_br_rot = data_biped.oMf[BR_joint].rotation();
    base_pos  = data_biped.oMf[base_joint].translation();
    base_rot  = data_biped.oMf[base_joint].rotation(); 

    // 步骤1：更新运动学状态（建议传入dq_fixed，预留时变率计算）
    pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, q_fixed); 
    // 步骤2：更新所有Frame的全局位姿（关键修复）
    pinocchio::updateFramePlacements(model_biped_fixed, data_biped_fixed); 
    // 步骤3：计算基础关节雅可比（getFrameJacobian的前置依赖，必须保留）
    pinocchio::computeJointJacobians(model_biped_fixed, data_biped_fixed, q_fixed); 
    // 步骤4：获取Frame雅可比（此时oMf已更新，结果正确）
    pinocchio::getFrameJacobian(model_biped_fixed, data_biped_fixed, FR_joint_fixed, pinocchio::LOCAL_WORLD_ALIGNED, J_FR_body);
    pinocchio::getFrameJacobian(model_biped_fixed, data_biped_fixed, FL_joint_fixed, pinocchio::LOCAL_WORLD_ALIGNED, J_FL_body);
    pinocchio::getFrameJacobian(model_biped_fixed, data_biped_fixed, BR_joint_fixed, pinocchio::LOCAL_WORLD_ALIGNED, J_BR_body);
    pinocchio::getFrameJacobian(model_biped_fixed, data_biped_fixed, BL_joint_fixed, pinocchio::LOCAL_WORLD_ALIGNED, J_BL_body);
    //oMi是关节位姿，oMf是Frame位姿
    fe_fl_pos_body = data_biped_fixed.oMf[FL_joint_fixed].translation();
    fe_fr_pos_body = data_biped_fixed.oMf[FR_joint_fixed].translation();
    fe_bl_pos_body = data_biped_fixed.oMf[BL_joint_fixed].translation();
    fe_br_pos_body = data_biped_fixed.oMf[BR_joint_fixed].translation();
    fe_fl_vel_body = (J_FL_body * dq_fixed).block(0, 0, 3, 1);
    fe_fr_vel_body = (J_FR_body * dq_fixed).block(0, 0, 3, 1);
    fe_bl_vel_body = (J_BL_body * dq_fixed).block(0, 0, 3, 1);
    fe_br_vel_body = (J_BR_body * dq_fixed).block(0, 0, 3, 1);

    // pinocchio::getJointJacobian(model_biped_fixed, data_biped_fixed, FL_hip_joint_fixed, pinocchio::LOCAL_WORLD_ALIGNED, J_hip_FL_body);
    // pinocchio::getJointJacobian(model_biped_fixed, data_biped_fixed, FR_hip_joint_fixed, pinocchio::LOCAL_WORLD_ALIGNED, J_hip_FR_body);
    // pinocchio::getJointJacobian(model_biped_fixed, data_biped_fixed, BL_hip_joint_fixed, pinocchio::LOCAL_WORLD_ALIGNED, J_hip_BL_body);
    // pinocchio::getJointJacobian(model_biped_fixed, data_biped_fixed, BR_hip_joint_fixed, pinocchio::LOCAL_WORLD_ALIGNED, J_hip_BR_body);
    // hip_fl_pos = data_biped_fixed.oMi[FL_hip_joint_fixed].translation();
    // hip_fr_pos = data_biped_fixed.oMi[FR_hip_joint_fixed].translation();
    // hip_bl_pos = data_biped_fixed.oMi[BL_hip_joint_fixed].translation();
    // hip_br_pos = data_biped_fixed.oMi[BR_hip_joint_fixed].translation();
    // hip_fl_vel = (J_hip_FL_body * dq_fixed).block(0, 0, 3, 1);
    // hip_fr_vel = (J_hip_FR_body * dq_fixed).block(0, 0, 3, 1);
    // hip_bl_vel = (J_hip_BL_body * dq_fixed).block(0, 0, 3, 1);
    // hip_br_vel = (J_hip_BR_body * dq_fixed).block(0, 0, 3, 1);
}

Eigen::Quaterniond Pin_KinDyn::intQuat(const Eigen::Quaterniond &quat, const Eigen::Matrix<double, 3, 1> &w)
{
    Eigen::Matrix3d Rcur = quat.normalized().toRotationMatrix();
    Eigen::Matrix3d Rinc = Eigen::Matrix3d::Identity();
    double theta = w.norm();
    if (theta > 1e-8)
    {
        Eigen::Vector3d w_norm;
        w_norm = w / theta;
        Eigen::Matrix3d a;
        a << 0, -w_norm(2), w_norm(1),
            w_norm(0), 0, -w_norm(0),
            -w_norm(1), w_norm(0), 0;
        Rinc = Eigen::Matrix3d::Identity() + a * sin(theta) + a * a * (1 - cos(theta));
    }
    Eigen::Matrix3d Rend = Rcur * Rinc;
    Eigen::Quaterniond quatRes;
    quatRes = Rend;
    return quatRes;
}

// intergrate the q with dq, for floating base dynamics
// 状态积分（integrateDIY）
Eigen::VectorXd Pin_KinDyn::integrateDIY(const Eigen::VectorXd &qI, const Eigen::VectorXd &dqI)
{
    Eigen::VectorXd qRes = Eigen::VectorXd::Zero(model_nv + 1);
    Eigen::Vector3d wDes;
    wDes << dqI(3), dqI(4), dqI(5);
    Eigen::Quaterniond quatNew, quatNow;
    quatNow.x() = qI(3);
    quatNow.y() = qI(4);
    quatNow.z() = qI(5);
    quatNow.w() = qI(6);

    quatNew = intQuat(quatNow, wDes);
    qRes = qI;
    qRes(0) += dqI(0);
    qRes(1) += dqI(1);
    qRes(2) += dqI(2);
    qRes(3) = quatNew.x();
    qRes(4) = quatNew.y();
    qRes(5) = quatNew.z();
    qRes(6) = quatNew.w();
    for (int i = 0; i < model_nv - 6; i++)
        qRes(7 + i) += dqI(6 + i);
    return qRes;
}

// update dynamic parameters, M*ddq+C*dq+G=tau
// 计算四足机器人的动力学参数 惯性矩阵 M、逆惯性矩阵 Minv、科里奥利力矩阵 C、重力向量 G
void Pin_KinDyn::computeDyn()
{
    // cal M 惯性矩阵
    pinocchio::crba(model_biped, data_biped, q);
    // Pinocchio only gives half of the M, needs to restore it here
    data_biped.M.triangularView<Eigen::Lower>() = data_biped.M.transpose().triangularView<Eigen::Lower>();
    dyn_M = data_biped.M;

    // cal Minv 逆惯性矩阵
    pinocchio::computeMinverse(model_biped, data_biped, q);
    data_biped.Minv.triangularView<Eigen::Lower>() = data_biped.Minv.transpose().triangularView<Eigen::Lower>();
    dyn_M_inv = data_biped.Minv;

    // cal C 科里奥利力矩阵
    pinocchio::computeCoriolisMatrix(model_biped, data_biped, q, dq);
    dyn_C = data_biped.C;

    // cal G 重力向量
    pinocchio::computeGeneralizedGravity(model_biped, data_biped, q);
    dyn_G = data_biped.g;

    // cal Ag, Centroidal Momentum Matrix. First three rows: linear, other three rows: angular
    // 计算质心动量矩阵 Ag 及其导数 dAg
    pinocchio::dccrba(model_biped, data_biped, q, dq);
    pinocchio::computeCentroidalMomentum(model_biped, data_biped, q, dq);
    dyn_Ag = data_biped.Ag;
    dyn_dAg = data_biped.dAg;

    // cal nonlinear item
    // 计算非线性项 Non
    dyn_Non = dyn_C * dq + dyn_G;

    // cal I
    // 计算惯性张量 I
    pinocchio::ccrba(model_biped, data_biped, q, dq);
    inertia = data_biped.Ig.inertia().matrix();

    // cal CoM
    // 计算质心位置 CoM_pos
    CoM_pos = data_biped.com[0];
    //    std::cout<<"CoM_W"<<std::endl;
    //    std::cout<<CoM_pos.transpose()<<std::endl;
    // 将之前计算得到的动力学参数从局部坐标系转换到世界坐标系。
    Eigen::MatrixXd Mpj, Mpj_inv; 
    Mpj = Eigen::MatrixXd::Identity(model_nv, model_nv);
    Mpj_inv = Eigen::MatrixXd::Identity(model_nv, model_nv);
    Mpj.block(0, 0, 3, 3) = base_rot.transpose();
    Mpj.block(3, 3, 3, 3) = base_rot.transpose();
    Mpj_inv.block(0, 0, 3, 3) = base_rot;
    Mpj_inv.block(3, 3, 3, 3) = base_rot;
    dyn_M = Mpj_inv * dyn_M * Mpj;
    dyn_M_inv = Mpj_inv * dyn_M_inv * Mpj;
    dyn_C = Mpj_inv * dyn_C * Mpj;
    dyn_G = Mpj_inv * dyn_G;
    dyn_Non = Mpj_inv * dyn_Non;


}

// // 核心IK求解函数（修复版）
// Pin_KinDyn::IkRes Pin_KinDyn::computeInK_Quadruped_3DOF(Eigen::Vector3d fe_pos_target_body[4])
// {
//     // 定义四足关节名（务必与URDF中的关节名完全一致）9 17 25 33
//     const std::string frame_names[4] = {"leg1_joint4", "leg2_joint4", "leg3_joint4", "leg4_joint4"};
//     pinocchio::FrameIndex J_IDs[4];
//     const pinocchio::FrameIndex J_ID_RF = 9;
//     const pinocchio::FrameIndex J_ID_LF = 17;
//     const pinocchio::FrameIndex J_ID_RH = 25;
//     const pinocchio::FrameIndex J_ID_LH = 33;

//     // -------------------------- 2. 初始化配置 --------------------------
//     Eigen::VectorXd qIk = Eigen::VectorXd::Zero(model_biped_fixed.nv); 
//     // 初始化四足12个关节（3DOF/腿 × 4腿）
//     qIk[0] = 0.0;     // 左前髋关节
//     qIk[1] = -0.5;    // 左前大腿（接近自然下垂角度）
//     qIk[2] = 0.5;     // 左前小腿
//     qIk[3] = 0.0;     // 右前髋关节
//     qIk[4] = -0.5;    // 右前大腿
//     qIk[5] = 0.5;     // 右前小腿
//     qIk[6] = 0.0;     // 左后髋关节
//     qIk[7] = -0.5;    // 左后大腿
//     qIk[8] = 0.5;     // 左后小腿
//     qIk[9] = 0.0;     // 右后髋关节
//     qIk[10] = -0.5;   // 右后大腿
//     qIk[11] = 0.5;    // 右后小腿

//     // IK求解参数（适配3DOF腿）
//     const double eps    = 1e-4;    // 位置误差收敛阈值（单位：m）
//     const int IT_MAX    = 200;     // 最大迭代次数
//     const double DT     = 0.1;     // 步长（3DOF腿可适当增大）
//     const double damp   = 1e-2;    // 阻尼项（避免雅克比奇异）
//     // -------------------------- 3. 变量初始化（完整初始化，避免未定义行为） --------------------------
//     Eigen::MatrixXd J_RF(3, model_biped_fixed.nv);
//     Eigen::MatrixXd J_LF(3, model_biped_fixed.nv); 
//     Eigen::MatrixXd J_RH(3, model_biped_fixed.nv);
//     Eigen::MatrixXd J_LH(3, model_biped_fixed.nv);
//     Eigen::MatrixXd J_total(12, model_biped_fixed.nv); // 总雅克比（12行）
//     J_LF.setZero(); J_RF.setZero(); J_LH.setZero(); J_RH.setZero(); J_total.setZero();

//     Eigen::Vector3d err_LF = Eigen::Vector3d::Zero();
//     Eigen::Vector3d err_RF = Eigen::Vector3d::Zero();
//     Eigen::Vector3d err_LH = Eigen::Vector3d::Zero();
//     Eigen::Vector3d err_RH = Eigen::Vector3d::Zero();
//     Eigen::VectorXd err_total = Eigen::VectorXd::Zero(12); // 显式初始化12维0
//     Eigen::VectorXd v = Eigen::VectorXd::Zero(model_biped_fixed.nv);

//     bool success = false;
//     int itr_count = 0;

//     // 加权矩阵：仅让3个腿部电机参与求解，屏蔽其他关节
//     Eigen::MatrixXd W = Eigen::MatrixXd::Identity(model_biped_fixed.nv, model_biped_fixed.nv);
//     // -------------------------- 4. 迭代求解核心 --------------------------
//     for (itr_count = 0;; itr_count++)
//     {
//         // 1. 正向运动学：计算当前关节配置下的足端位置
//         pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, qIk);
//         pinocchio::updateFramePlacements(model_biped_fixed, data_biped_fixed);

//         // 2. 计算每条腿的位置误差（直接位置差，无需姿态误差）
//         const Eigen::Vector3d pos_RF = data_biped_fixed.oMf[J_ID_RF].translation();
//         const Eigen::Vector3d pos_LF = data_biped_fixed.oMf[J_ID_LF].translation();
//         const Eigen::Vector3d pos_RH = data_biped_fixed.oMf[J_ID_RH].translation();
//         const Eigen::Vector3d pos_LH = data_biped_fixed.oMf[J_ID_LH].translation();

//         // 修正：目标位置索引与腿的对应关系（原代码索引错位）
//         err_RF = fe_pos_target_body[0] - pos_RF;  // RF=右前 → 索引0
//         err_LF = fe_pos_target_body[1] - pos_LF;  // LF=左前 → 索引1
//         err_RH = fe_pos_target_body[2] - pos_RH;  // RH=右后 → 索引2
//         err_LH = fe_pos_target_body[3] - pos_LH;  // LH=左后 → 索引3

//         // 拼接12维总误差（按RF/LF/RH/LH顺序）
//         err_total << err_RF, err_LF, err_RH, err_LH;

//         // 3. 收敛判断：总位置误差小于阈值则退出
//         if (err_total.norm() < eps) {
//             success = true;
//             break;
//         }
//         if (itr_count >= IT_MAX) {
//             success = false;
//             break;
//         }

//         // 4. 计算每条腿的位置雅克比（仅前3行，丢弃姿态部分）
//         pinocchio::computeFrameJacobian(model_biped_fixed, data_biped_fixed, qIk, J_ID_LF, data_biped_fixed.J);
//         J_LF = data_biped_fixed.J.topRows<3>(); // 仅保留位置雅克比
//         pinocchio::computeFrameJacobian(model_biped_fixed, data_biped_fixed, qIk, J_ID_RF, data_biped_fixed.J);
//         J_RF = data_biped_fixed.J.topRows<3>();
//         pinocchio::computeFrameJacobian(model_biped_fixed, data_biped_fixed, qIk, J_ID_LH, data_biped_fixed.J);
//         J_LH = data_biped_fixed.J.topRows<3>();
//         pinocchio::computeFrameJacobian(model_biped_fixed, data_biped_fixed, qIk, J_ID_RH, data_biped_fixed.J);
//         J_RH = data_biped_fixed.J.topRows<3>();

//         // 5. 拼接12维总雅克比（与误差顺序一致）
//         J_total.block(0,  0, 3, model_biped_fixed.nv) = J_RF;
//         J_total.block(3,  0, 3, model_biped_fixed.nv) = J_LF;
//         J_total.block(6,  0, 3, model_biped_fixed.nv) = J_RH;
//         J_total.block(9,  0, 3, model_biped_fixed.nv) = J_LH;

//         // 6. 阻尼最小二乘求解关节增量（数值稳定性优化）
//         Eigen::MatrixXd JJt = J_total * W * J_total.transpose();
//         JJt.diagonal().array() += damp; // 添加阻尼避免奇异
//         v = -W * J_total.transpose() * JJt.ldlt().solve(err_total);

//         // 7. 更新关节配置 + 限制关节限位（关键：3DOF腿必须限幅）
//         qIk = pinocchio::integrate(model_biped_fixed, qIk, v * DT);
//         for (int i = 0; i < model_biped_fixed.nv; ++i) {
//             qIk(i) = std::max(model_biped_fixed.lowerPositionLimit[i],
//                               std::min(model_biped_fixed.upperPositionLimit[i], qIk(i)));
//         }
//     }

//     // -------------------------- 5. 结果封装 --------------------------
//     IkRes res;
//     res.err = err_total;       // 12维位置误差
//     res.itr = itr_count;       // 迭代次数
//     res.status = success ? 0 : -1; // 0成功/-1失败
//     res.jointPosRes = qIk;     // 求解后的关节配置
//     qIk_static = qIk;
//     std::cout << "qIk求解结果：" << std::endl;
//     std::cout << qIk.transpose() << std::endl;
//     std::cout << "q" << std::endl;
//     std::cout << q.transpose() << std::endl;
//     return res;
// }

// must call computeDyn() first!
void Pin_KinDyn::workspaceConstraint(Eigen::VectorXd &qFT, Eigen::VectorXd &tauJointFT)
{
    for (int i = 0; i < motorName.size(); i++)
        if (qFT(i + 7) > motorMaxPos(i))
        {
            qFT(i + 7) = motorMaxPos(i);
            motorReachLimit[i] = true;
            tauJointFT(i) = tauJointOld(i);
        }
        else if (qFT(i + 7) < motorMinPos(i))
        {
            qFT(i + 7) = motorMinPos(i);
            motorReachLimit[i] = true;
            tauJointFT(i) = tauJointOld(i);
        }
        else
            motorReachLimit[i] = false;

    tauJointOld = tauJointFT;
}

// 打印数据
void Pin_KinDyn::send_data(void)
{
    std::cout << "Foot positions target in body FL:" << std::endl;
    std::cout << fe_fl_pos_body << std::endl;

    std::cout << "Foot positions target in body FR:" << std::endl;
    std::cout << fe_fr_pos_body << std::endl;

    std::cout << "Foot positions target in body BL:" << std::endl;
    std::cout << fe_bl_pos_body << std::endl;

    std::cout << "Foot positions target in body BR:" << std::endl;
    std::cout << fe_br_pos_body << std::endl;


    std::cout << "Foot speed target in body FL:" << std::endl;
    std::cout << fe_fl_vel_body << std::endl;

    std::cout << "Foot speed target in body FR:" << std::endl;
    std::cout << fe_fr_vel_body << std::endl;

    std::cout << "Foot speed target in body BL:" << std::endl;
    std::cout << fe_bl_vel_body << std::endl;

    std::cout << "Foot speed target in body BR:" << std::endl;
    std::cout << fe_br_vel_body << std::endl;

}

//
void Pin_KinDyn::update_data_to_MJ_interface(data_bus &robotState)
{
    robotState.fe_pos_body[0] = fe_fr_pos_body;
    robotState.fe_pos_body[1] = fe_fl_pos_body;
    robotState.fe_pos_body[2] = fe_br_pos_body;
    robotState.fe_pos_body[3] = fe_bl_pos_body;

    robotState.fe_spd_body[0] = fe_fr_vel_body;
    robotState.fe_spd_body[1] = fe_fl_vel_body;
    robotState.fe_spd_body[2] = fe_br_vel_body;
    robotState.fe_spd_body[3] = fe_bl_vel_body;

    Eigen::AngleAxisd rollAngle (Eigen::AngleAxisd(robotState.rpy[0],Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(robotState.rpy[1],Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle  (Eigen::AngleAxisd(robotState.rpy[2],Eigen::Vector3d::UnitZ()));  
    Eigen::Quaterniond quaternion;
    quaternion=yawAngle*pitchAngle*rollAngle;
    robotState.quaternion = quaternion;

    //动力学M矩阵，与加速度有关的系数
    robotState.dyn_M = dyn_M;
    //动力学M矩阵的逆
    robotState.dyn_M_inv = dyn_M_inv;
    //质心动量矩阵，与系统动量有关
    robotState.dyn_Ag = dyn_Ag;
    //质心动量矩阵的导数
    robotState.dyn_dAg = dyn_dAg;
    //M*ddg+C*dg+G=tau, dyn_Non=dyn_c*dg+dyn_G
    robotState.dyn_Non = dyn_Non;

    robotState.J_FL_body = J_FL_body;
    robotState.J_FR_body = J_FR_body; 
    robotState.J_BL_body = J_BL_body;
    robotState.J_BR_body = J_BR_body;

    robotState.J_FL = J_FL.block(0, 0, 3, model_nv);
    robotState.J_FR = J_FR.block(0, 0, 3, model_nv);
    robotState.J_BL = J_BL.block(0, 0, 3, model_nv);
    robotState.J_BR = J_BR.block(0, 0, 3, model_nv);

    robotState.dJ_FL = dJ_FL.block(0, 0, 3, model_nv);
    robotState.dJ_FR = dJ_FR.block(0, 0, 3, model_nv);
    robotState.dJ_BL = dJ_BL.block(0, 0, 3, model_nv);
    robotState.dJ_BR = dJ_BR.block(0, 0, 3, model_nv);
}