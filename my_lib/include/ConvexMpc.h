#ifndef A1_CPP_CONVEXMPC_H
#define A1_CPP_CONVEXMPC_H

#include <vector>
#include <chrono>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
#include "OsqpEigen/OsqpEigen.h"

#include "CtrlStates.h"
#include "base_struct.h"
#include "Utils.h"

class ConvexMpc {
public:
    //构造函数
    ConvexMpc(Eigen::VectorXd &q_weights_, Eigen::VectorXd &r_weights_);
    //重置函数
    void reset();
    // 计算 A 矩阵
    void calculate_A_mat_c(Eigen::Vector3d root_euler);
    // 计算 B 矩阵
    void calculate_B_mat_c(double robot_mass, const Eigen::Matrix3d &a1_trunk_inertia, Eigen::Matrix3d root_rot_mat,
                           Eigen::Matrix<double, 3, NUM_LEG> foot_pos);
    // 状态空间离散化
    void state_space_discretization(double dt);
    // 计算 QP 矩阵
    void calculate_qp_mats(CtrlStates &state);

//private:
    // 足端摩擦系数
    double mu;
    double fz_min;
    double fz_max;
// 权重向量和矩阵
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, 1> q_weights_mpc;
    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, 1> r_weights_mpc;

    Eigen::DiagonalMatrix<double, MPC_STATE_DIM * PLAN_HORIZON> Q;
    Eigen::DiagonalMatrix<double, NUM_DOF * PLAN_HORIZON> R;
    //130 * 130
    Eigen::SparseMatrix<double> Q_sparse;
    //120 * 120
    Eigen::SparseMatrix<double> R_sparse;
    //13 * 13
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A_mat_c;
    //13 * 12
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B_mat_c;
    //130 * 12
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF> B_mat_c_list;
    //25 * 25
    Eigen::Matrix<double, MPC_STATE_DIM + NUM_DOF, MPC_STATE_DIM + NUM_DOF> AB_mat_c;
    //13 * 13
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A_mat_d;
    //13 * 12
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B_mat_d;
    //130 * 12
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF> B_mat_d_list;
    //25 * 25
    Eigen::Matrix<double, MPC_STATE_DIM + NUM_DOF, MPC_STATE_DIM + NUM_DOF> AB_mat_d;
    //130 * 12
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, MPC_STATE_DIM> A_qp;
    //130 * 120
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON> B_qp;
    Eigen::SparseMatrix<double> A_qp_sparse;
    Eigen::SparseMatrix<double> B_qp_sparse;

    // standard QP formulation
    // minimize 1/2 * x' * P * x + q' * x
    // subject to lb <= Ac * x <= ub
    Eigen::SparseMatrix<double> hessian; // P
    // Eigen::VectorXd gradient; // q
    Eigen::SparseMatrix<double> linear_constraints; // Ac
    //    Eigen::VectorXd lb;
    //    Eigen::VectorXd ub;
    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, 1> gradient; // q
    Eigen::Matrix<double, MPC_CONSTRAINT_DIM * PLAN_HORIZON, 1> lb;
    Eigen::Matrix<double, MPC_CONSTRAINT_DIM * PLAN_HORIZON, 1> ub;

};

#endif //A1_CPP_CONVEXMPC_H
