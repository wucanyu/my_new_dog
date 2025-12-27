#include "RobotControl.h"

RobotControl::RobotControl() {
    std::cout << "init RobotControl" << std::endl;
    // init QP solver
    // init some parameters
    //权重矩阵
    Q.diagonal() << 20.0, 20.0, 50.0, 450.0, 450.0, 450.0;
    mu = 0.4;
    F_min = 0;
    F_max = 150;
    hessian.resize(3 * NUM_LEG, 3 * NUM_LEG);
    gradient.resize(3 * NUM_LEG);
    linearMatrix.resize(NUM_LEG + 4 * NUM_LEG, 3 * NUM_LEG);
    lowerBound.resize(20);
    lowerBound.setZero();
    upperBound.resize(20);
    upperBound.setZero();
    root_acc.setZero();

    Eigen::Matrix<double, 12, 1> w, u;
    w << 10, 10, 4, 10, 10, 4, 10, 10, 4, 10, 10, 4;
    u << 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3;
    _W = w.asDiagonal();
    _U = u.asDiagonal();

    // init mpc skip counter
    mpc_init_counter = 0;

//  1.0000  0.0000 -0.3500  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000
//  0.0000  1.0000 -0.3500  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000
//  0.0000  1.0000  0.3500  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000
//  1.0000  0.0000  0.3500  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000
//  0.0000  0.0000  1.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000
//  0.0000  0.0000  0.0000  1.0000  0.0000 -0.3500  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000
//  0.0000  0.0000  0.0000  0.0000  1.0000 -0.3500  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000
//  0.0000  0.0000  0.0000  0.0000  1.0000  0.3500  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000
//  0.0000  0.0000  0.0000  1.0000  0.0000  0.3500  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000
//  0.0000  0.0000  0.0000  0.0000  0.0000  1.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000
//  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  1.0000  0.0000 -0.3500  0.0000  0.0000  0.0000
//  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  1.0000 -0.3500  0.0000  0.0000  0.0000
//  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  1.0000  0.3500  0.0000  0.0000  0.0000
//  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  1.0000  0.0000  0.3500  0.0000  0.0000  0.0000
//  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  1.0000  0.0000  0.0000  0.0000
//  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  1.0000  0.0000 -0.3500
//  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  1.0000 -0.3500
//  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  1.0000  0.3500
//  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  1.0000  0.0000  0.3500
//  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000 -0.0000  0.0000  0.0000 -0.0000

    for (int i = 0; i < NUM_LEG; ++i) {

        linearMatrix.insert(i * 5, i * 3) = 1;

        linearMatrix.insert( 1 + i * 5, 1 + i * 3) = 1;
        linearMatrix.insert( 2 + i * 5, 1 + i * 3) = 1;

        linearMatrix.insert( 3 + i * 5,  i * 3) = 1;
        linearMatrix.insert( 4 + i * 5,  2 + i * 3) = 1;

        linearMatrix.insert( 0 + i * 5, 2 + i * 3) = -mu;
        linearMatrix.insert( 1 + i * 5, 2 + i * 3) = -mu;
        linearMatrix.insert( 2 + i * 5, 2 + i * 3) = mu;
        linearMatrix.insert( 3 + i * 5, 2 + i * 3) = mu;
    }

    terrain_angle_filter = MovingWindowFilter(100);
    for (int i = 0; i < NUM_LEG; ++i) {
        recent_contact_x_filter[i] = MovingWindowFilter(60);
        recent_contact_y_filter[i] = MovingWindowFilter(60);
        recent_contact_z_filter[i] = MovingWindowFilter(60);
    }
}

void RobotControl::cout_data(CtrlStates &state) {
    std::cout << "state.contacts :" << std::endl;
    std::cout << state.contacts[0] << state.contacts[1] << state.contacts[2] << state.contacts[3] <<  std::endl;
   
    std::cout << "taget pos:" << std::endl;
    std::cout << state.root_pos_d.transpose() << std::endl;
    
    // 打印卡尔曼估计出的质心位置
    std::cout << "Estimated root position:" << std::endl;
    std::cout << state.estimated_root_pos.transpose() << std::endl;

    std::cout << "taget speed world:" << std::endl;
    std::cout << state.root_lin_vel_d_world.transpose() << std::endl;

    // 打印卡尔曼估计出的质心速度
    std::cout << "Estimated root velocity:" << std::endl;
    std::cout << state.estimated_root_vel.transpose() << std::endl;

    // 打印欧拉角
    std::cout << "Root Euler angles:" << std::endl;
    std::cout << state.root_euler_360.transpose() << std::endl;

    // 打印 IMU 角速度
    std::cout << "IMU angular velocity:" << std::endl;
    std::cout << state.imu_ang_vel.transpose() << std::endl;

    // 打印 IMU 加速度
    std::cout << "IMU acceleration:" << std::endl;
    std::cout << state.imu_acc.transpose() << std::endl;

    /***************************************************************/
    // 打印足端的力
    std::cout << "Foot stance force in world system:" << std::endl;
    std::cout << state.foot_forces_grf_world << std::endl;

    // 打印足端的力
    std::cout << "Foot swing force in world system:" << std::endl;
    std::cout << state.foot_forces_kin_world << std::endl;

    std::cout << "Foot positions in body coordinate system:" << std::endl;
    std::cout << state.foot_pos_rel << std::endl;

    // 打印足端在机器人坐标系下的位置
    std::cout << "Foot positions in world coordinate system:" << std::endl;
    std::cout << state.foot_pos_world << std::endl;

    // 打印摆动相目标位置，在机器人坐标系下的位置
    std::cout << "Foot positions target in world coordinate system:" << std::endl;
    std::cout << state.foot_pos_target_world << std::endl;

    // 打印足端在机器人坐标系下的位置
    std::cout << "Foot positions start in world system:" << std::endl;
    std::cout << state.foot_pos_start_world << std::endl;

    // 打印摆动相目标位置，在机器人坐标系下的位置
    std::cout << "Foot positions end in world coordinate system:" << std::endl;
    std::cout << state.foot_pos_end_world << std::endl;
    
    // 打印足端的力
    std::cout << "Foot torgue in body system:" << std::endl;
    std::cout << state.joint_torques_out.transpose() << std::endl;
    
}

Eigen::Matrix<double, 3, NUM_LEG> RobotControl::compute_grf(CtrlStates &state, double dt) {
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;
    // first get parameters needed to construct the solver hessian and gradient
    // use euler angle to get desired angle
    Eigen::Vector3d euler_error = state.root_euler_d - state.root_euler;

    // limit euler error to pi/2
    if (euler_error(2) > 3.1415926 * 1.5) {
        euler_error(2) = state.root_euler_d(2) - 3.1415926 * 2 - state.root_euler(2);
    } 
    else if (euler_error(2) < -3.1415926 * 1.5) {
        euler_error(2) = state.root_euler_d(2) + 3.1415926 * 2 - state.root_euler(2);
    }

    if (state.stance_leg_control_type == 0) { // 0: QP 不旋转
        // desired acc in world frame
        // 将根加速度向量 root_acc 初始化为零向量
        root_acc.setZero();
        // 计算根位置误差（期望位置减去当前位置），并与线性比例增益 kp_linear 逐元素相乘，结果赋给 root_acc 的前三个元素
        root_acc.block<3, 1>(0, 0) = state.kp_linear.cwiseProduct(state.root_pos_d - state.root_pos);
        // 计算根线速度误差（期望线速度减去当前线速度），与线性微分增益 kd_linear 逐元素相乘，再乘以根旋转矩阵，将结果累加到 root_acc 的前三个元素
        root_acc.block<3, 1>(0, 0) += state.root_rot_mat * state.kd_linear.cwiseProduct(state.root_lin_vel_d - state.root_rot_mat.transpose() * state.root_lin_vel);
        // 计算欧拉角误差与角比例增益 kp_angular 逐元素相乘，结果赋给 root_acc 的第 4 到第 6 个元素
        root_acc.block<3, 1>(3, 0) = state.kp_angular.cwiseProduct(euler_error);
        // 计算根角速度误差（期望角速度减去当前角速度），与角微分增益 kd_angular 逐元素相乘，将结果累加到 root_acc 的第 4 到第 6 个元素
        root_acc.block<3, 1>(3, 0) += state.kd_angular.cwiseProduct(state.root_ang_vel_d - state.root_rot_mat.transpose() * state.root_ang_vel);

        // add gravity
        root_acc(2) += state.robot_mass * 9.8;

        // Update the A matrix in the controller notation A*f = b
        Eigen::Matrix<double, 6, DIM_GRF> inertia_inv;
        for (int i = 0; i < NUM_LEG; ++i) {
            inertia_inv.block<3, 3>(0, i * 3).setIdentity();
            inertia_inv.block<3, 3>(3, i * 3) = Utils::skew(state.foot_pos_abs.block<3, 1>(0, i));
        }

        Eigen::Matrix<double, DIM_GRF, DIM_GRF> dense_hessian;
        dense_hessian.setIdentity();
        dense_hessian = inertia_inv.transpose() * Q * inertia_inv + _alpha *_W;
        hessian = dense_hessian.sparseView();
        gradient.block<3 * NUM_LEG, 1>(0, 0) = -root_acc.transpose() * Q * inertia_inv;

        // Eigen::Matrix<double, DIM_GRF, DIM_GRF> dense_hessian;
        // dense_hessian.setIdentity();
        // dense_hessian = inertia_inv.transpose() * Q * inertia_inv + _alpha*_W + _beta*_U;
        // hessian = dense_hessian.sparseView();
        // gradient.block<3 * NUM_LEG, 1>(0, 0) = -root_acc.transpose() * Q * inertia_inv; - _beta*_Fprev.transpose()*_U;

        //lbA <= Cf <= ubA
        for (int i = 0; i < NUM_LEG; i++) {
            double c_flag = state.contacts[i] ? 1.0 : 0.0;
            lowerBound[5 * i + 0] = -c_flag * OsqpEigen::INFTY;
            lowerBound[5 * i + 1] = -c_flag * OsqpEigen::INFTY;
            lowerBound[5 * i + 2] = 0;
            lowerBound[5 * i + 3] = 0;
            lowerBound[5 * i + 4] = c_flag * F_min;

            upperBound[5 * i + 0] = 0;
            upperBound[5 * i + 1] = 0;
            upperBound[5 * i + 2] = c_flag * OsqpEigen::INFTY;
            upperBound[5 * i + 3] = c_flag * OsqpEigen::INFTY;
            upperBound[5 * i + 4] = c_flag * F_max;
        }
 
        // instantiate the solver
        OsqpEigen::Solver solver;
        // settings
        //求解器不会输出过多的中间信息。
        solver.settings()->setVerbosity(false);
        //关闭求解器的热启动功能
        solver.settings()->setWarmStart(false);
        //获取求解器的数据对象
        solver.data()->setNumberOfVariables(12);
        //设置优化问题中的约束数量
        solver.data()->setNumberOfConstraints(20);
        //设置线性约束矩阵
        solver.data()->setLinearConstraintsMatrix(linearMatrix);
        //用于描述目标函数的二次项部分
        solver.data()->setHessianMatrix(hessian);
        //设置梯度向量。梯度向量用于描述目标函数的一次项部分
        solver.data()->setGradient(gradient);
        //下界
        solver.data()->setLowerBound(lowerBound);
        //上界       
        solver.data()->setUpperBound(upperBound);

        auto t1 = std::chrono::high_resolution_clock::now();
        solver.initSolver();
        auto t2 = std::chrono::high_resolution_clock::now();
        solver.solve();
        auto t3 = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
        std::chrono::duration<double, std::milli> ms_double_2 = t3 - t2;

        // std::cout << "qp solver init time: " << ms_double_1.count() << "ms; solve time: " << ms_double_2.count() << "ms" << std::endl;

        Eigen::VectorXd QPSolution = solver.getSolution(); //12x1
        for (int i = 0; i < NUM_LEG; ++i) {
            state.foot_forces_grf_world.block<3, 1>(0, i) = QPSolution.segment<3>(i * 3);
            foot_forces_grf.block<3, 1>(0, i) = state.root_rot_mat.transpose() * QPSolution.segment<3>(i * 3);
            //foot_forces_grf.block<3, 1>(0, i) = QPSolution.segment<3>(i * 3);
        }
    }
    else if (state.stance_leg_control_type == 1) { // 1: MPC
        Eigen::VectorXd q_weight,r_weight;
        q_weight.resize(13);
        r_weight.resize(12);

        if(state.movement_mode == 0)
        {
            q_weight = state.q_weights;
            r_weight = state.r_weights;
        }
        else if(state.movement_mode == 1)
        {
            q_weight = state.q_weights_trot;
            r_weight = state.r_weights_trot;
        }
        ConvexMpc mpc_solver = ConvexMpc(q_weight, r_weight);
        mpc_solver.reset();

        // initialize the mpc state at the first time step
        // state.mpc_states.resize(13);
        state.mpc_states << state.root_euler[0], state.root_euler[1], state.root_euler[2],
                            state.root_pos[0], state.root_pos[1], state.root_pos[2],
                            state.root_ang_vel[0], state.root_ang_vel[1], state.root_ang_vel[2],
                            state.root_lin_vel[0], state.root_lin_vel[1], state.root_lin_vel[2],
                            -9.8;

        double mpc_dt = 0.04;

        // state.mpc_states_d.resize(13 * PLAN_HORIZON);
        if(state.movement_mode == 0)
        {
            for (int i = 0; i < PLAN_HORIZON; ++i) {
            state.mpc_states_d.segment(i * 13, 13)
                    <<
                    state.root_euler_d[0],
                    state.root_euler_d[1],
                    state.root_euler[2],
                    state.root_pos_d[0],
                    state.root_pos_d[1],
                    state.root_pos_d[2],
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    -9.8;
            }
        }
        else if(state.movement_mode == 1)
        {
            const float max_pos_error = 0.1;
            double xstart = state.root_pos_d[0];
            double ystart = state.root_pos_d[1];
            if(xstart - state.root_pos[0] > max_pos_error) 
            {
                xstart= state.root_pos[0] + max_pos_error;
            }
            if(state.root_pos[0] - xstart > max_pos_error) 
            {
                xstart = state.root_pos[0] - max_pos_error;
            }

            if(ystart - state.root_pos[1] > max_pos_error)
            {
                ystart = state.root_pos[1] + max_pos_error;
            }
            if(state.root_pos[1] - ystart > max_pos_error)
            {
                ystart = state.root_pos[1] - max_pos_error;
            } 
            state.root_pos_d[0] = xstart;
            state.root_pos_d[1] = ystart;

            for (int i = 0; i < PLAN_HORIZON; ++i) {
            state.mpc_states_d.segment(i * 13, 13)
                    <<
                    state.root_euler_d[0],
                    state.root_euler_d[1],
                    state.root_euler[2] + state.root_ang_vel_d[2] * mpc_dt * (i + 1),
                    state.root_pos[0] + state.root_lin_vel_d_world[0] * mpc_dt * (i + 1),
                    state.root_pos[1] + state.root_lin_vel_d_world[1] * mpc_dt * (i + 1),
                    state.root_pos_d[2],
                    0,
                    0,
                    state.root_ang_vel_d[2],
                    state.root_lin_vel_d_world[0],
                    state.root_lin_vel_d_world[1],
                    0,
                    -9.8;
            }
        }

        // a single A_c is computed for the entire reference trajectory
        auto t1 = std::chrono::high_resolution_clock::now();
        mpc_solver.calculate_A_mat_c(state.root_euler);

        // for each point in the reference trajectory, an approximate B_c matrix is computed using desired values of euler angles and feet positions
        // from the reference trajectory and foot placement controller

        auto t2 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < PLAN_HORIZON; i++) {
            // calculate current B_c matrix
            mpc_solver.calculate_B_mat_c(state.robot_mass,
                                         state.a1_trunk_inertia,
                                         state.root_rot_mat,
                                         state.foot_pos_abs);

            // state space discretization, calculate A_d and current B_d
            mpc_solver.state_space_discretization(mpc_dt);

            // store current B_d matrix
            mpc_solver.B_mat_d_list.block<13, 12>(i * 13, 0) = mpc_solver.B_mat_d;
        }

        // calculate QP matrices
        auto t3 = std::chrono::high_resolution_clock::now();
        mpc_solver.calculate_qp_mats(state);

        // solve
        auto t4 = std::chrono::high_resolution_clock::now();
        if (!solver.isInitialized()) {
            solver.settings()->setVerbosity(false);
            solver.settings()->setWarmStart(true);
            solver.data()->setNumberOfVariables(NUM_DOF * PLAN_HORIZON);
            solver.data()->setNumberOfConstraints(MPC_CONSTRAINT_DIM * PLAN_HORIZON);
            solver.data()->setLinearConstraintsMatrix(mpc_solver.linear_constraints);
            solver.data()->setHessianMatrix(mpc_solver.hessian);
            solver.data()->setGradient(mpc_solver.gradient);
            solver.data()->setLowerBound(mpc_solver.lb);
            solver.data()->setUpperBound(mpc_solver.ub);
            solver.initSolver();
        } else {
            solver.updateHessianMatrix(mpc_solver.hessian);
            solver.updateGradient(mpc_solver.gradient);
            solver.updateLowerBound(mpc_solver.lb);
            solver.updateUpperBound(mpc_solver.ub);
        }
        auto t5 = std::chrono::high_resolution_clock::now();
        solver.solve();
        auto t6 = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> ms_double_1 = t2 - t1;
        std::chrono::duration<double, std::milli> ms_double_2 = t3 - t2;
        std::chrono::duration<double, std::milli> ms_double_3 = t4 - t3;
        std::chrono::duration<double, std::milli> ms_double_4 = t5 - t4;
        std::chrono::duration<double, std::milli> ms_double_5 = t6 - t5;

    //    std::cout << "mpc cal A_mat_c: " << ms_double_1.count() << "ms" << std::endl;
    //    std::cout << "mpc cal B_mat_d_list: " << ms_double_2.count() << "ms" << std::endl;
    //    std::cout << "mpc cal qp mats: " << ms_double_3.count() << "ms" << std::endl;
    //    std::cout << "mpc init time: " << ms_double_4.count() << "ms" << std::endl;
    //    std::cout << "mpc solve time: " << ms_double_5.count() << "ms" << std::endl << std::endl;

        Eigen::VectorXd solution = solver.getSolution();
        // std::cout << solution.transpose() << std::endl;

        for (int i = 0; i < NUM_LEG; ++i) {
            if (!isnan(solution.segment<3>(i * 3).norm()))
            {
                state.foot_forces_grf_world.block<3, 1>(0, i) = solution.segment<3>(i * 3);
                foot_forces_grf.block<3, 1>(0, i) = state.root_rot_mat.transpose() * solution.segment<3>(i * 3);
            }
        }

        for (int i = 0; i < NUM_LEG; ++i) {      
            if(!state.contacts[i])
            {
                state.foot_forces_grf_world.block<3, 1>(0, i) << 0 , 0 , 0;
                foot_forces_grf.block<3, 1>(0, i) << 0 , 0 , 0;
            }
        }
                    
    }
    
    return foot_forces_grf;
}

//其作用是根据机器人腿部最近接触点的位置信息来计算行走表面的系数向量。
Eigen::Vector3d RobotControl::compute_walking_surface(CtrlStates &state) {
    Eigen::Matrix<double, NUM_LEG, 3> W;
    Eigen::VectorXd foot_pos_z;
    Eigen::Vector3d a;
    Eigen::Vector3d surf_coef;

    W.block<NUM_LEG, 1>(0, 0).setOnes();
    W.block<NUM_LEG, 2>(0, 1) = state.foot_pos_recent_contact.block<2, NUM_LEG>(0, 0).transpose();

    foot_pos_z.resize(NUM_LEG);
    foot_pos_z = state.foot_pos_recent_contact.block<1, NUM_LEG>(2, 0).transpose();

    a = Utils::pseudo_inverse(W.transpose() * W) * W.transpose() * foot_pos_z;
    // surface: a1 * x + a2 * y - z + a0 = 0, coefficient vector: [a1, a2, -1]
    surf_coef << a[1], a[2], -1;
    return surf_coef;
}



void RobotControl::update_plan(CtrlStates &state, double dt) {
    //更新摆动相足端起点
    for (int i = 0; i < NUM_LEG; ++i) 
    {
        if ((state.contacts[i] != state.early_contacts[i]) && state.contacts[i] == true)
        {
            //时刻更新足端位置
            state.foot_pos_start_world.block<3, 1>(0, i) = state.foot_pos_world.block<3, 1>(0, i);
        }
        else
        {
            state.foot_pos_start_world.block<3, 1>(0, i) << 0,0,0;              
        }
    }
    for (int i = 0; i < NUM_LEG; ++i) 
    {
        if (!state.contacts[i])
        {
            Eigen::Vector3d pos_body_hip[4];
            Eigen::Vector3d pos_body_hip_offset;
            pos_body_hip_offset << -0.018 , 0, 0;
            pos_body_hip[0] <<  0.211332  ,-0.1668  ,0;
            pos_body_hip[1] <<  0.211332  , 0.1668  ,0;
            pos_body_hip[2] << -0.211332  ,-0.1668  ,0;
            pos_body_hip[3] << -0.211332  , 0.1668  ,0;

            for (int i = 0; i < NUM_LEG; ++i)
            {
                pos_body_hip[i] += pos_body_hip_offset;
            }
            Eigen::Vector3d ptouchcom = state.root_pos +
            state.root_rot_mat * state.root_lin_vel_d * state.robot_phase_remain * state.robot_time_half;
            Eigen::AngleAxisd yaw_touch = (Eigen::AngleAxisd(state.root_euler(2) 
                                        + state.root_ang_vel_d_world(2) * state.robot_phase_remain * state.robot_time_half,
                                          Eigen::Vector3d::UnitZ()));
            Eigen::AngleAxisd yaw_vary  (Eigen::AngleAxisd(0.5 * state.robot_time_half * state.root_ang_vel_d_world(2),
                                        Eigen::Vector3d::UnitZ()));
            Eigen::Vector3d ptouchhip = ptouchcom + yaw_touch * pos_body_hip[i];

            Eigen::Vector3d padd1 = 0.5 * state.robot_time_half * state.root_lin_vel_d_world;
            Eigen::Vector3d padd2 = yaw_touch * (yaw_vary * pos_body_hip[i] - pos_body_hip[i]);
            Eigen::Vector3d padd3 = 0.15 * (state.root_lin_vel - state.root_lin_vel_d_world);
            Eigen::Vector3d padd4 = Utils::skew((state.root_pos(2) / 9.81f) * state.root_lin_vel) * state.root_ang_vel;
            
            state.foot_pos_end_world.block<3, 1>(0, i) = ptouchhip + padd1 + padd2 + padd3 + padd4;

            Eigen::Vector3d footPos;
            Eigen::Vector3d footVel;         
            footPos(0) = bezierUtils[i].cubicBezier(state.foot_pos_start_world(0,i), state.foot_pos_end_world(0,i), state.robot_phase);
            footPos(1) = bezierUtils[i].cubicBezier(state.foot_pos_start_world(1,i), state.foot_pos_end_world(1,i), state.robot_phase);
            footVel(0) = bezierUtils[i].cubicBezier_v(state.foot_pos_start_world(0,i), state.foot_pos_end_world(0,i), state.robot_phase)/state.max_time_half;
            footVel(1) = bezierUtils[i].cubicBezier_v(state.foot_pos_start_world(1,i), state.foot_pos_end_world(1,i), state.robot_phase)/state.max_time_half;

            if(state.robot_phase < 0.5)
            {
                footPos(2) = bezierUtils[i].cubicBezier(state.foot_pos_start_world(2,i), state.foot_pos_start_world(2,i) + 0.07, state.robot_phase*2 );
                footVel(2) = bezierUtils[i].cubicBezier_v(state.foot_pos_start_world(2,i),state.foot_pos_start_world(2,i) + 0.07, state.robot_phase*2 )*2/state.max_time_half;
            }
            else
            {
                footPos(2) = bezierUtils[i].cubicBezier(state.foot_pos_start_world(2,i) + 0.07, 0.0, state.robot_phase * 2 - 1);
                footVel(2) = bezierUtils[i].cubicBezier_v(state.foot_pos_start_world(2,i) + 0.07, 0.0, state.robot_phase * 2 - 1)*2/state.max_time_half;
            }
            state.foot_pos_target_world.block<3, 1>(0, i) = footPos;
            state.foot_vel_target_world.block<3, 1>(0, i) = footVel;
        }
    }
}

void RobotControl::generate_swing_legs_ctrl(CtrlStates &state, double dt) {
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target;  foot_pos_target.setZero();
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target;  foot_vel_target.setZero();
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin;
    Eigen::Matrix<double, 3, NUM_LEG> foot_forces_kin_world;
    for (int i = 0; i < NUM_LEG; ++i) {
        //时刻计算足端摆动相的力
        if (state.contacts[i]) 
        {
            foot_forces_kin_world.block<3, 1>(0, i) << 0,0,0;
            foot_forces_kin.block<3, 1>(0, i) << 0,0,0;
        }
        else 
        {
            foot_pos_target.block<3, 1>(0, i) = state.foot_pos_target_world.block<3, 1>(0, i);
            foot_vel_target.block<3, 1>(0, i) = state.foot_vel_target_world.block<3, 1>(0, i);

            foot_pos_error.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i) - state.foot_pos_world.block<3, 1>(0, i);
            foot_vel_error.block<3, 1>(0, i) = foot_vel_target.block<3, 1>(0, i) - state.foot_vel_world.block<3, 1>(0, i);
        
            foot_forces_kin_world.block<3, 1>(0, i) = foot_pos_error.block<3, 1>(0, i).cwiseProduct(state.kp_foot.block<3, 1>(0, i)) +
                                                      foot_vel_error.block<3, 1>(0, i).cwiseProduct(state.kd_foot.block<3, 1>(0, i));

            foot_forces_kin.block<3, 1>(0, i) = state.root_rot_mat.transpose() * foot_forces_kin_world.block<3, 1>(0, i);
            
        }
    }
    state.foot_forces_kin = foot_forces_kin;
    state.foot_forces_kin_world = foot_forces_kin_world;

    state.joint_torques_swing.block<3, 1>(0, 0) = state.J3_FR_body.transpose()  * state.foot_forces_kin.block<3, 1>(0, 0);
    state.joint_torques_swing.block<3, 1>(3, 0) = state.J3_FL_body.transpose()  * state.foot_forces_kin.block<3, 1>(0, 1);
    state.joint_torques_swing.block<3, 1>(6, 0) = state.J3_BR_body.transpose()  * state.foot_forces_kin.block<3, 1>(0, 2);
    state.joint_torques_swing.block<3, 1>(9, 0) = state.J3_BL_body.transpose()  * state.foot_forces_kin.block<3, 1>(0, 3);

    for (int i = 0; i < NUM_LEG; ++i) {
        if(state.contacts[i] == true)
        {
            state.joint_torques_out.block<3, 1>(i*3, 0) = state.joint_torques.block<3, 1>(i*3, 0);
        }
        else
        {
            state.joint_torques_out.block<3, 1>(i*3, 0) = state.joint_torques_swing.block<3, 1>(i*3, 0);
        }
    }
}
