#include "ConvexMpc.h"

ConvexMpc::ConvexMpc(Eigen::VectorXd &q_weights_, Eigen::VectorXd &r_weights_) {
    mu = 0.4;
    fz_min = 0.0;
    fz_max = 0.0;

    // reserve size for sparse matrix 预留稀疏矩阵的空间 Q R为权重矩阵 分别用于惩罚状态误差和控制输入
    //130 * 130
    Q_sparse = Eigen::SparseMatrix<double>(MPC_STATE_DIM * PLAN_HORIZON,MPC_STATE_DIM * PLAN_HORIZON);
    //120 * 120
    R_sparse = Eigen::SparseMatrix<double>(NUM_DOF * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON);
    // 初始化 q_weights_mpc 并设置 Q 矩阵的对角元素
    q_weights_mpc.resize(MPC_STATE_DIM * PLAN_HORIZON);
    for (int i = 0; i < PLAN_HORIZON; ++i) {
        q_weights_mpc.segment(i * MPC_STATE_DIM, MPC_STATE_DIM) = q_weights_;
    }
    Q.diagonal() = 2*q_weights_mpc;
    for (int i = 0; i < MPC_STATE_DIM*PLAN_HORIZON; ++i) {
        Q_sparse.insert(i,i) = 2*q_weights_mpc(i);
    }
    // 初始化 r_weights_mpc 并设置 R 矩阵的对角元素
    r_weights_mpc.resize(NUM_DOF * PLAN_HORIZON);
    for (int i = 0; i < PLAN_HORIZON; ++i) {
        r_weights_mpc.segment(i * NUM_DOF, NUM_DOF) = r_weights_;
    }
    R.diagonal() = 2*r_weights_mpc;
    for (int i = 0; i < NUM_DOF*PLAN_HORIZON; ++i) {
        R_sparse.insert(i,i) = 2*r_weights_mpc(i);
    }
    // 初始化线性约束矩阵
    //200 * 120
    // | 1  0  mu |
    // | 1  0 -mu |
    // | 0  1  mu |
    // | 0  1 -mu |
    // | 0  0  1  |
    linear_constraints.resize(MPC_CONSTRAINT_DIM * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON);    
    for (int i = 0; i < NUM_LEG * PLAN_HORIZON; ++i) {
        linear_constraints.insert(0 + 5 * i, 0 + 3 * i) = 1;
        linear_constraints.insert(1 + 5 * i, 0 + 3 * i) = 1;
        linear_constraints.insert(2 + 5 * i, 1 + 3 * i) = 1;
        linear_constraints.insert(3 + 5 * i, 1 + 3 * i) = 1;
        linear_constraints.insert(4 + 5 * i, 2 + 3 * i) = 1;

        linear_constraints.insert(0 + 5 * i, 2 + 3 * i) = mu;
        linear_constraints.insert(1 + 5 * i, 2 + 3 * i) = -mu;
        linear_constraints.insert(2 + 5 * i, 2 + 3 * i) = mu;
        linear_constraints.insert(3 + 5 * i, 2 + 3 * i) = -mu;
    }

}

void ConvexMpc::reset() {
 
    A_mat_c.setZero();
    B_mat_c.setZero();
    B_mat_c_list.setZero();
    AB_mat_c.setZero();

    A_mat_d.setZero();
    B_mat_d.setZero();
    B_mat_d_list.setZero();

    AB_mat_d.setZero();
    A_qp.setZero();
    B_qp.setZero();
    gradient.setZero();
    lb.setZero();
    ub.setZero();

}
//https://blog.csdn.net/Kalenee/article/details/126440918?spm=1001.2101.3001.6650.2&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-2-126440918-blog-123293994.235%5Ev38%5Epc_relevant_sort_base3&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-2-126440918-blog-123293994.235%5Ev38%5Epc_relevant_sort_base3&utm_relevant_index=3
//xhat = a*x +b*u;  质心运动学  更新a方程
void ConvexMpc::calculate_A_mat_c(Eigen::Vector3d root_euler) {
    double cos_yaw = cos(root_euler[2]);
    double sin_yaw = sin(root_euler[2]);

    Eigen::Matrix3d ang_vel_to_rpy_rate;

    ang_vel_to_rpy_rate <<  cos_yaw, sin_yaw, 0,
                            -sin_yaw, cos_yaw, 0,
                            0, 0, 1;

    A_mat_c.block<3, 3>(0, 6) = ang_vel_to_rpy_rate;
    A_mat_c.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    //z轴方向受x轴方向速度的影响程度
    A_mat_c(11, NUM_DOF) = 1;
}

//xhat = a*x +b*u;  质心运动学  更新b方程
void ConvexMpc::calculate_B_mat_c(double robot_mass, const Eigen::Matrix3d &a1_trunk_inertia, Eigen::Matrix3d root_rot_mat,
                                  Eigen::Matrix<double, 3, NUM_LEG> foot_pos) {
    // we need to calculate PLAN_HORIZON B matrices 
    Eigen::Matrix3d a1_trunk_inertia_world;
    a1_trunk_inertia_world = root_rot_mat * a1_trunk_inertia * root_rot_mat.transpose();
    for (int i = 0; i < NUM_LEG; ++i) {
        // 世界坐标系的惯性矩阵*足端位置的反对称矩阵
        B_mat_c.block<3, 3>(6, 3 * i) = a1_trunk_inertia_world.inverse() * Utils::skew(foot_pos.block<3, 1>(0, i));
        //质量倒数
        B_mat_c.block<3, 3>(9, 3 * i) = (1 / robot_mass) * Eigen::Matrix3d::Identity();
    }
}

//将方程前向欧拉离散化
void ConvexMpc::state_space_discretization(double dt) {
    // simplified exp 
    // TODO: this function is not necessary because A is actually sparse
    auto t1 = std::chrono::high_resolution_clock::now();
    // AB_mat_d = (dt * AB_mat_c).exp();
    A_mat_d = Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() + A_mat_c*dt;
    B_mat_d = B_mat_c*dt;
    auto t2 = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
//    std::cout << "IN DISCRETIZATION: matrix exp: " << ms_double_1.count() << "ms" << std::endl;
}

void ConvexMpc::calculate_qp_mats(CtrlStates &state) {
    // standard QP formulation
    // minimize 1/2 * x' * P * x + q' * x
    // subject to lb <= Ac * x <= ub
    // P: hessian
    // q: gradient
    // Ac: linear constraints

    // A_qp = [A,
    //         A^2,
    //         A^3,
    //         ...
    //         A^k]'

    // B_qp = [A^0*B(0),
    //         A^1*B(0),     B(1),
    //         A^2*B(0),     A*B(1),       B(2),
    //         ...
    //         A^(k-1)*B(0), A^(k-2)*B(1), A^(k-3)*B(2), ... B(k-1)]

    // auto t1 = std::chrono::high_resolution_clock::now();
    // calculate A_qp and B_qp
    // TODO: THIS PART TAKES AROUND 0.2MS!

    // keep A_qp as a storage list 
    for (int i = 0; i < PLAN_HORIZON; ++i) 
    {
        //给MPC多次离散化的A矩阵赋值
        if (i == 0) 
        {
            A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * i, 0) = A_mat_d;
        }
        else 
        {
            A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * i, 0) = 
            A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * (i-1), 0)*A_mat_d;
        }
        //给MPC多次离散化的B矩阵赋值
        for (int j = 0; j < i + 1; ++j) {
            if (i-j == 0) {
                    B_qp.block<MPC_STATE_DIM, NUM_DOF>(MPC_STATE_DIM * i, NUM_DOF * j) = B_mat_d_list.block<MPC_STATE_DIM, NUM_DOF>(j * MPC_STATE_DIM, 0);
            } else {
                B_qp.block<MPC_STATE_DIM, NUM_DOF>(MPC_STATE_DIM * i, NUM_DOF * j) =
                        A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * (i-j-1), 0) 
                        * B_mat_d_list.block<MPC_STATE_DIM, NUM_DOF>(j * MPC_STATE_DIM, 0);
            }
        }
    }

    // auto t2 = std::chrono::high_resolution_clock::now();
    // calculate hessian
    // TODO: THIS PART TAKES AROUND 0.4MS!
    Eigen::Matrix<double, NUM_DOF * PLAN_HORIZON, NUM_DOF * PLAN_HORIZON> dense_hessian;
    //dense_hessian = (B_qp.transpose() * Q * B_qp + R);
    dense_hessian = (B_qp.transpose() * Q * B_qp);
    dense_hessian += R;
    //将矩阵表示为稀疏矩阵可以显著提高存储和计算效率，
    //因为在许多情况下，矩阵中的大部分元素可能为零。
    //sparseView用于将稠密矩阵转换为稀疏矩阵
    //H = 2(Bqp.T * Q(L) * Bqp + R(K))
    hessian = dense_hessian.sparseView();
    // auto t3 = std::chrono::high_resolution_clock::now();
    // calculate gradient
    //得到预测的状态向量
    Eigen::Matrix<double, 13*PLAN_HORIZON, 1> tmp_vec = A_qp* state.mpc_states;
    //计算预测状态 tmp_vec 与期望状态 state.mpc_states_d 的差，得到状态误差
    tmp_vec -= state.mpc_states_d;
    //g = 2 * Bqp.T * Q(L) *(Aqp*x0 - y)
    gradient = B_qp.transpose() * Q * tmp_vec;

    // auto t4 = std::chrono::high_resolution_clock::now();
    // auto t5 = std::chrono::high_resolution_clock::now();
    // calculate lower bound and upper bound
    //垂直力的最小值 fz_min 为 0，最大值 fz_max 为 180
    fz_min = 0;
    fz_max = 150;

    Eigen::VectorXd lb_one_horizon(MPC_CONSTRAINT_DIM);
    Eigen::VectorXd ub_one_horizon(MPC_CONSTRAINT_DIM);
    for (int i = 0; i < NUM_LEG; ++i) {
        lb_one_horizon.segment<5>(i * 5) << 
                0,
                -OsqpEigen::INFTY,
                0,
                -OsqpEigen::INFTY,
                fz_min * state.contacts[i];
        ub_one_horizon.segment<5>(i * 5) << 
                OsqpEigen::INFTY,
                0,
                OsqpEigen::INFTY,
                0,
                fz_max * state.contacts[i];
    }
    // std:: cout << lb_one_horizon.transpose() << std::endl;
    // std:: cout << ub_one_horizon.transpose() << std::endl;
    for (int i = 0; i < PLAN_HORIZON; ++i) {
        lb.segment<MPC_CONSTRAINT_DIM>(i * MPC_CONSTRAINT_DIM) = lb_one_horizon;
        ub.segment<MPC_CONSTRAINT_DIM>(i * MPC_CONSTRAINT_DIM) = ub_one_horizon;
    }

}
