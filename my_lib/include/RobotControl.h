#ifndef CPP_ROBOTCONTROL_H
#define CPP_ROBOTCONTROL_H

#include <iostream>
#include <string>
#include <chrono>

// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"
#include <eigen3/Eigen/Dense>

#include "base_struct.h"
#include "CtrlStates.h"
#include "Utils.h"
#include "ConvexMpc.h"

#include "filter.h"


class RobotControl {
public:
    RobotControl();

    void update_plan(CtrlStates &state, double dt);

    void generate_swing_legs_ctrl(CtrlStates &state, double dt);

    Eigen::Matrix<double, 3, NUM_LEG> compute_grf(CtrlStates &state, double dt);

    Eigen::Vector3d compute_walking_surface(CtrlStates &state);

    void cout_data(CtrlStates &state);

private:
    BezierUtils bezierUtils[NUM_LEG];
    //期望加速度 
    Eigen::Matrix<double, 6, 1> root_acc;
    //S_CONTROL权重
    Eigen::DiagonalMatrix<double, 6> Q;

    //  足端力大小的权重矩阵
    Eigen::Matrix<double, 12, 12> _W;
    //  足端力改变量的权重矩阵
    Eigen::Matrix<double, 12, 12> _U;
    //W的权重系数 
    double _alpha;
    //U的权重系数 
    double _beta;
    // ground friction coefficient
    double mu;
    double F_min;
    double F_max;
    // allocate QP problem matrices and vectors
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    //20*12
    Eigen::SparseMatrix<double> linearMatrix;
    //12*1
    Eigen::VectorXd lowerBound;
    //12*1
    Eigen::VectorXd upperBound;

    OsqpEigen::Solver solver;

    //MPC does not start for the first 10 ticks to prevent uninitialized NAN goes into joint_torques
    int mpc_init_counter;

    // filters
    MovingWindowFilter terrain_angle_filter;
    MovingWindowFilter recent_contact_x_filter[NUM_LEG];
    MovingWindowFilter recent_contact_y_filter[NUM_LEG];
    MovingWindowFilter recent_contact_z_filter[NUM_LEG];
};


#endif //A1_CPP_A1ROBOTCONTROL_H
