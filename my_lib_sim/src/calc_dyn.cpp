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

// // Inverse kinematics for leg posture. Note: the Rdes and Pdes are both w.r.t the baselink coordinate in body frame!
// // 逆运动学
// Pin_KinDyn::IkRes
// Pin_KinDyn::computeInK_Leg(const Eigen::Matrix3d &Rdes_L, const Eigen::Vector3d &Pdes_L, const Eigen::Matrix3d &Rdes_R,
//                            const Eigen::Vector3d &Pdes_R)
// {
//     const pinocchio::SE3 oMdesL(Rdes_L, Pdes_L);
//     const pinocchio::SE3 oMdesR(Rdes_R, Pdes_R);
//     // arm-l: 0-6, arm-r: 7-13, head: 14,15 waist: 16-18, leg-l: 19-24, leg-r: 25-30
//     Eigen::VectorXd qIk = Eigen::VectorXd::Zero(model_biped_fixed.nv); // initial guess
//     qIk[22] = -0.1;
//     qIk[28] = -0.1;
//     const double eps = 1e-4;
//     const int IT_MAX = 100;
//     const double DT = 7e-1;
//     const double damp = 5e-3;
//     Eigen::MatrixXd JL(6, model_biped_fixed.nv);
//     Eigen::MatrixXd JR(6, model_biped_fixed.nv);
//     Eigen::MatrixXd JCompact(12, model_biped_fixed.nv);
//     JL.setZero();
//     JR.setZero();
//     JCompact.setZero();
//     bool success = false;
//     Eigen::Matrix<double, 6, 1> errL, errR;
//     Eigen::Matrix<double, 12, 1> errCompact;
//     Eigen::VectorXd v(model_biped_fixed.nv);
//     pinocchio::JointIndex J_Idx_l, J_Idx_r;
//     J_Idx_l = l_ankle_joint_fixed;
//     J_Idx_r = r_ankle_joint_fixed;
//     int itr_count{0};
//     for (itr_count = 0;; itr_count++)
//     {
//         pinocchio::forwardKinematics(model_biped_fixed, data_biped_fixed, qIk);
//         const pinocchio::SE3 iMdL = data_biped_fixed.oMi[J_Idx_l].actInv(oMdesL);
//         const pinocchio::SE3 iMdR = data_biped_fixed.oMi[J_Idx_r].actInv(oMdesR);
//         errL = pinocchio::log6(iMdL).toVector(); // in joint frame
//         errR = pinocchio::log6(iMdR).toVector(); // in joint frame
//         errCompact.block<6, 1>(0, 0) = errL;
//         errCompact.block<6, 1>(6, 0) = errR;
//         if (errCompact.norm() < eps)
//         {
//             success = true;
//             break;
//         }
//         if (itr_count >= IT_MAX)
//         {
//             success = false;
//             break;
//         }
//         pinocchio::computeJointJacobian(model_biped_fixed, data_biped_fixed, qIk, J_Idx_l, JL); // JL in joint frame
//         pinocchio::computeJointJacobian(model_biped_fixed, data_biped_fixed, qIk, J_Idx_r, JR); // JR in joint frame
//         Eigen::MatrixXd W;
//         W = Eigen::MatrixXd::Identity(model_biped_fixed.nv, model_biped_fixed.nv); // weighted matrix
//         // arm-l: 0-6, arm-r: 7-13, head: 14,15 waist: 16-18, leg-l: 19-24, leg-r: 25-30
//         //        W(16,16)=0.001;  // use a smaller value to make the solver try not to use waist joint
//         //        W(17,17)=0.001;
//         //        W(18,18)=0.001;
//         JL.block(0, 16, 6, 3).setZero();
//         JR.block(0, 16, 6, 3).setZero();
//         pinocchio::Data::Matrix6 JlogL;
//         pinocchio::Data::Matrix6 JlogR;
//         pinocchio::Jlog6(iMdL.inverse(), JlogL);
//         pinocchio::Jlog6(iMdR.inverse(), JlogR);
//         JL = -JlogL * JL;
//         JR = -JlogR * JR;
//         JCompact.block(0, 0, 6, model_biped_fixed.nv) = JL;
//         JCompact.block(6, 0, 6, model_biped_fixed.nv) = JR;
//         // pinocchio::Data::Matrix6 JJt;
//         Eigen::Matrix<double, 12, 12> JJt;
//         JJt.noalias() = JCompact * W * JCompact.transpose();
//         JJt.diagonal().array() += damp;
//         v.noalias() = -W * JCompact.transpose() * JJt.ldlt().solve(errCompact);
//         qIk = pinocchio::integrate(model_biped_fixed, qIk, v * DT);
//     }
//     IkRes res;
//     res.err = errCompact;
//     res.itr = itr_count;
//     if (success)
//     {
//         res.status = 0;
//     }
//     else
//     {
//         res.status = -1;
//     }
//     res.jointPosRes = qIk;
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