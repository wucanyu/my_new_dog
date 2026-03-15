#include "wbc_priority.h"
#include "iostream"

//QP_nvIn=18, QP_ncIn=22
WBC_priority::WBC_priority(int model_nv_In, int QP_nvIn, int QP_ncIn, double mu_In, double dt) : QP_prob(QP_nvIn,QP_ncIn)
{
    timeStep = dt;
    model_nv = model_nv_In;//18
    mu = mu_In;

    QP_nv = QP_nvIn;
    QP_nc = QP_ncIn;

    //取消浮动基关节加速度 6*18
    Sf = Eigen::MatrixXd::Zero(6, model_nv);
    Sf.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);
    // 6 means the dims of delta_b
    St_qpV1 = Eigen::MatrixXd::Zero(model_nv, 6); 
    St_qpV1.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);
    // 6 means the dims of floating base
    St_qpV2 = Eigen::MatrixXd::Zero(model_nv, model_nv - 6); 
    St_qpV2.block(6, 0, model_nv - 6, model_nv - 6) = Eigen::MatrixXd::Identity(model_nv - 6, model_nv - 6);

    // defined in body frame
    f_z_low = 0;
    f_z_upp = 180;

    qpOASES::Options options;
    options.setToMPC();
    // options.setToReliable();
    options.printLevel = qpOASES::PL_LOW;
    QP_prob.setOptions(options);

    eigen_xOpt = Eigen::VectorXd::Zero(QP_nv);
    eigen_ddq_Opt = Eigen::VectorXd::Zero(model_nv);
    eigen_tau_Opt = Eigen::VectorXd::Zero(model_nv - 6);

    delta_q_final_kin = Eigen::VectorXd::Zero(model_nv);
    dq_final_kin = Eigen::VectorXd::Zero(model_nv);
    ddq_final_kin = Eigen::VectorXd::Zero(model_nv);
    
    base_rpy_cur = Eigen::VectorXd::Zero(3);

    //  WBC task defined and order build
    ///------------ trot --------------
    std::vector<std::string> taskOrder_walk;
    taskOrder_walk.emplace_back("StanceLeg");
    taskOrder_walk.emplace_back("BodyOriTask");
    taskOrder_walk.emplace_back("BodyPosTask");
    taskOrder_walk.emplace_back("SwingLeg");

    kinwbc_tasks_walk.addTask("StanceLeg");
    kinwbc_tasks_walk.addTask("BodyOriTask");
    kinwbc_tasks_walk.addTask("BodyPosTask");
    kinwbc_tasks_walk.addTask("SwingLeg");
    kinwbc_tasks_walk.buildPriority(taskOrder_walk);

    ///-------- stand ------------
    std::vector<std::string> taskOrder_stand;
    taskOrder_stand.emplace_back("StanceLeg");
    taskOrder_stand.emplace_back("BodyOriTask");
    taskOrder_stand.emplace_back("BodyPosTask");

    kinwbc_tasks_stand.addTask("StanceLeg");
    kinwbc_tasks_stand.addTask("BodyOriTask");
    kinwbc_tasks_stand.addTask("BodyPosTask");
    kinwbc_tasks_stand.buildPriority(taskOrder_stand);
}

//读取数据
void WBC_priority::dataBusRead(const data_bus &robotState)
{
    //状态: body在世界的当前的rpy                           
    base_rpy_cur = robotState.imu_euler;
    //Body的在世界的期望rpy
    base_rpy_des = robotState.base_rpy_des;  
    //body在世界位置期望=
    base_pos_des = robotState.base_pos_des;      
    //基座(body)的在世界坐标系下的旋转矩阵
    base_rot = robotState.base_rot;

    for (int i = 0; i < 4; i++)
    {
        contacts[i] = robotState.contacts[i]; 
        fe_pos_world[i] = robotState.fe_pos_world[i];
    }

    //期望的广义(带虚拟浮动基六自由度的关节)坐标(关节坐标)的加速度
    des_ddq = robotState.des_ddq;
    des_dq = robotState.des_dq;
    des_q = robotState.des_q;
    des_delta_q = robotState.des_delta_q;

    J_FR = robotState.J_FR;
    J_FL = robotState.J_FL;
    J_BR = robotState.J_BR;
    J_BL = robotState.J_BL;

    //所有足端的世界方向的雅可比矩阵
    //1234
    Jfe = Eigen::MatrixXd::Zero(12, model_nv);
    Jfe.block(0, 0, 3, model_nv) = robotState.J_FR;
    Jfe.block(3, 0, 3, model_nv) = robotState.J_FL;
    Jfe.block(6, 0, 3, model_nv) = robotState.J_BR;
    Jfe.block(9, 0, 3, model_nv) = robotState.J_BL;
    //1234
    dJfe = Eigen::MatrixXd::Zero(12, model_nv);
    dJfe.block(0, 0, 3, model_nv) = robotState.dJ_FR;
    dJfe.block(3, 0, 3, model_nv) = robotState.dJ_FL;
    dJfe.block(6, 0, 3, model_nv) = robotState.dJ_BR;
    dJfe.block(9, 0, 3, model_nv) = robotState.dJ_BL;

    //由MPC计算得出的地面反力 0123
    Fr_ff = robotState.Fr_ff;
    //动力学M矩阵，与加速度有关的系数
    dyn_M = robotState.dyn_M;
    //动力学M矩阵的逆
    dyn_M_inv = robotState.dyn_M_inv;
    //质心动量矩阵，与系统动量有关
    dyn_Ag = robotState.dyn_Ag;
    //质心动量矩阵的导数
    dyn_dAg = robotState.dyn_dAg;
    //M*ddg+C*dg+G=tau, dyn_Non=dyn_c*dg+dyn_G
    dyn_Non = robotState.dyn_Non;
    //广义速度
    dq = robotState.dq;
    //广义位置
    q = robotState.q;

    //获取当前状态
    joy_cmd_ctrl_state = robotState.joy_cmd_ctrl_state;
    //摆动足的期望位置
    swing_fe_pos_des_W = Eigen::VectorXd::Zero(6);
    swing_fe_vel_des_W = Eigen::VectorXd::Zero(6);
    swing_fe_acc_des_W = Eigen::VectorXd::Zero(6);
    //摆动足的当前位置
    swing_fe_pos_cur_W = Eigen::VectorXd::Zero(6);
    swing_fe_vel_cur_W = Eigen::VectorXd::Zero(6);
    if(joy_cmd_ctrl_state == 1)
    {
        if(!contacts[0])
        {
            swing_fe_pos_des_W.block(0, 0, 3, 1) = robotState.fe_pos_world_des[0];
            swing_fe_vel_des_W.block(0, 0, 3, 1) = robotState.fe_vel_world_des[0];
            swing_fe_acc_des_W.block(0, 0, 3, 1) = robotState.fe_acc_world_des[0];
            swing_fe_pos_cur_W.block(0, 0, 3, 1) = robotState.fe_pos_world[0];
            swing_fe_vel_cur_W.block(0, 0, 3, 1) = robotState.fe_vel_world[0];
        }
        if(!contacts[1])
        {
            swing_fe_pos_des_W.block(0, 0, 3, 1) = robotState.fe_pos_world_des[1];
            swing_fe_vel_des_W.block(0, 0, 3, 1) = robotState.fe_vel_world_des[1];
            swing_fe_acc_des_W.block(0, 0, 3, 1) = robotState.fe_acc_world_des[1];
            swing_fe_pos_cur_W.block(0, 0, 3, 1) = robotState.fe_pos_world[1];
            swing_fe_vel_cur_W.block(0, 0, 3, 1) = robotState.fe_vel_world[1];
        }
        if(!contacts[2])
        {
            swing_fe_pos_des_W.block(3, 0, 3, 1) = robotState.fe_pos_world_des[2]; 
            swing_fe_vel_des_W.block(3, 0, 3, 1) = robotState.fe_vel_world_des[2];
            swing_fe_acc_des_W.block(3, 0, 3, 1) = robotState.fe_acc_world_des[2];
            swing_fe_pos_cur_W.block(3, 0, 3, 1) = robotState.fe_pos_world[2];      
            swing_fe_vel_cur_W.block(3, 0, 3, 1) = robotState.fe_vel_world[2];
        }
        if(!contacts[3])
        {
            swing_fe_pos_des_W.block(3, 0, 3, 1) = robotState.fe_pos_world_des[3]; 
            swing_fe_vel_des_W.block(3, 0, 3, 1) = robotState.fe_vel_world_des[3];
            swing_fe_acc_des_W.block(3, 0, 3, 1) = robotState.fe_acc_world_des[3];
            swing_fe_pos_cur_W.block(3, 0, 3, 1) = robotState.fe_pos_world[3];    
            swing_fe_vel_cur_W.block(3, 0, 3, 1) = robotState.fe_vel_world[3];
        } 
    }

    if (joy_cmd_ctrl_state == 1)
    {
        Jc      = Eigen::MatrixXd::Zero(6, model_nv);
        dJc     = Eigen::MatrixXd::Zero(6, model_nv);
        Jsw     = Eigen::MatrixXd::Zero(6, model_nv);
        dJsw    = Eigen::MatrixXd::Zero(6, model_nv);
        if(contacts[0])//摆动相
        {
            Jc.block(0, 0, 3, model_nv) = robotState.J_FR;
            dJc.block(0, 0, 3, model_nv) = robotState.dJ_FR;      
        }
        else
        {
            Jsw.block(0, 0, 3, model_nv) = robotState.J_FR;
            dJsw.block(0, 0, 3, model_nv) = robotState.dJ_FR;               
        }

        if(contacts[1])//摆动相
        {
            Jc.block(0, 0, 3, model_nv) = robotState.J_FL;
            dJc.block(0, 0, 3, model_nv) = robotState.dJ_FL;             
        }
        else
        {
            Jsw.block(0, 0, 3, model_nv) = robotState.J_FL;
            dJsw.block(0, 0, 3, model_nv) = robotState.dJ_FL;     
        }

        if(contacts[2])//摆动相
        {
            Jc.block(3, 0, 3, model_nv) = robotState.J_BR;
            dJc.block(3, 0, 3, model_nv) = robotState.dJ_BR;  
        }
        else
        { 
            Jsw.block(3, 0, 3, model_nv) = robotState.J_BR;
            dJsw.block(3, 0, 3, model_nv) = robotState.dJ_BR;          
        }  

        if(contacts[3])//摆动相
        {
            Jc.block(3, 0, 3, model_nv) = robotState.J_BL;
            dJc.block(3, 0, 3, model_nv) = robotState.dJ_BL;   
        }
        else
        {
            Jsw.block(3, 0, 3, model_nv) = robotState.J_BL;
            dJsw.block(3, 0, 3, model_nv) = robotState.dJ_BL;  
        }
    }
    else//站立状态
    {
        Jc = Eigen::MatrixXd::Zero(12, model_nv);
        Jc = Jfe;
        dJc = Eigen::MatrixXd::Zero(12, model_nv);
        dJc = dJfe;
    }

}

// 核心IK求解函数（修复版）0.07 0.1668 0.0968 0.2110 大腿 0.21 0.22
void WBC_priority::computeInK_Quadruped_3DOF(Eigen::Vector3d fe_pos_target_body[4])
{
    qIK = Eigen::VectorXd::Zero(12,1);
    for(int i = 0;i<=3;i++)
    {
        Eigen::Vector3d fe = fe_pos_target_body[i];
        Eigen::Vector3d omega = Eigen::Vector3d::Zero();
        double xp = fe.x();
        if(i == 0 || i == 1)//在前
            xp = xp - 0.211;
        else                //在后
            xp = xp + 0.211; 
        double yp = fe.y();
        if(i == 0 || i == 2)//在右
            yp = yp + 0.07;
        else                //在左
            yp = yp - 0.07; 
        double zp = fe.z();
        double l1 = 0.0968; //在左
        if(i == 0 || i == 2)//在右
        {
            l1 = -l1;
        }
        double l2 = -0.235;  
        double l3 = -0.213;
        double L = sqrt(zp*zp + yp*yp - l1*l1);
        omega(0) = atan2(zp*l1+yp*L , yp*l1-zp*L);
        double ap = sqrt(xp*xp + yp*yp + zp*zp - l1*l1);
        omega(2) = -3.1415926 + acos((l2*l2 + l3*l3 - ap*ap)/(2*l2*l3));
        double a1 = yp * sin(omega(0)) - zp * cos(omega(0));
        double a2 = xp; 
        double m1 = l3*sin(omega(2));
        double m2 = l3*cos(omega(2)) + l2;
        omega(1) = atan2(a1 * m1 + a2 * m2 , a2 * m1 - a1 * m2);
        qIK.block<3,1>(i*3,0) << omega;
    }
}


//计算关节空间的期望 q ，dq ，dqq
void WBC_priority::computeDdq(Pin_KinDyn &pinKinDynIn)
{
    // -------- trot -------------
    if (joy_cmd_ctrl_state == 1)
    {
    /***************支撑腿****************/   
        int id = kinwbc_tasks_walk.getId("StanceLeg");
        kinwbc_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(6);
        kinwbc_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(6);
        kinwbc_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(6);
        kinwbc_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(6);
        kinwbc_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(6, 6) * 0;
        kinwbc_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(6, 6) * 0;
        kinwbc_tasks_walk.taskLib[id].J = Jc;
        kinwbc_tasks_walk.taskLib[id].dJ = dJc;
        kinwbc_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

    /***************身体姿态任务**********/    
        id = kinwbc_tasks_walk.getId("BodyOriTask");
        //rpy状态的状态差
        kinwbc_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(3);
        Eigen::Matrix3d desRot = Utils::eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));// 期望姿态转旋转矩阵
        kinwbc_tasks_walk.taskLib[id].errX = Utils::diffRot(base_rot, desRot);// 计算当前姿态与期望姿态的误差（旋转矩阵差）
        //rpy状态的速度差
        kinwbc_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(3);
        kinwbc_tasks_walk.taskLib[id].derrX = -dq.block<3, 1>(3, 0);// 速度误差= -基座角速度（dq[3:5]是基座rpy角速度）
        //期望状态加速度为零
        kinwbc_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(3);
        //期望速度为零
        kinwbc_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(3);
        //KP权重为100
        kinwbc_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(3, 3) * 500;
        //KP权重为10     
        kinwbc_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(3, 3) * 10;

        kinwbc_tasks_walk.taskLib[id].J = Eigen::MatrixXd::Zero(3, model_nv);
        kinwbc_tasks_walk.taskLib[id].J.block<3, 3>(0, 3) = base_rot.transpose();
        kinwbc_tasks_walk.taskLib[id].dJ = Eigen::MatrixXd::Zero(3, model_nv);
        //该任务对所有关节的权重为一
        kinwbc_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

    /**************身体位置任务***********/
        id = kinwbc_tasks_walk.getId("BodyPosTask");
        kinwbc_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(3);
        kinwbc_tasks_walk.taskLib[id].errX = base_pos_des - q.block<3, 1>(0, 0);
        kinwbc_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(3);
        kinwbc_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(3);
        kinwbc_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(3);
        kinwbc_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(3, 3) * 200; 
        kinwbc_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(3, 3) * 5;
        kinwbc_tasks_walk.taskLib[id].J = Eigen::MatrixXd::Zero(3, model_nv);
        kinwbc_tasks_walk.taskLib[id].J.block<3, 3>(0, 0) = base_rot.transpose();
        kinwbc_tasks_walk.taskLib[id].dJ = Eigen::MatrixXd::Zero(3, model_nv);//由于dj和dq的乘积为零，直接定义dj为零，不需要多加计算 3*18 * 18*1
        kinwbc_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

    /***************摆动腿任务****************/
        id = kinwbc_tasks_walk.getId("SwingLeg");
        kinwbc_tasks_walk.taskLib[id].errX = Eigen::VectorXd::Zero(6);
        kinwbc_tasks_walk.taskLib[id].errX = swing_fe_pos_des_W - swing_fe_pos_cur_W;
        kinwbc_tasks_walk.taskLib[id].derrX = Eigen::VectorXd::Zero(6);
        kinwbc_tasks_walk.taskLib[id].derrX = swing_fe_vel_des_W - swing_fe_vel_cur_W;
        kinwbc_tasks_walk.taskLib[id].ddxDes = Eigen::VectorXd::Zero(6);;
        kinwbc_tasks_walk.taskLib[id].dxDes = Eigen::VectorXd::Zero(6);;
        kinwbc_tasks_walk.taskLib[id].kp = Eigen::MatrixXd::Identity(6, 6) * 100;
        kinwbc_tasks_walk.taskLib[id].kd = Eigen::MatrixXd::Identity(6, 6) * 5;
        kinwbc_tasks_walk.taskLib[id].J = Jsw;
        kinwbc_tasks_walk.taskLib[id].dJ = dJsw;
        kinwbc_tasks_walk.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

    }
    else    /// -------- stand -------------
    {
    /***************支撑腿****************/   
        int id = kinwbc_tasks_stand.getId("StanceLeg");
        kinwbc_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(12);
        kinwbc_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(12);
        kinwbc_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(12);
        kinwbc_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(12);
        kinwbc_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(12, 12) * 0;
        kinwbc_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(12, 12) * 0;
        kinwbc_tasks_stand.taskLib[id].J = Jfe;
        kinwbc_tasks_stand.taskLib[id].dJ = dJfe;
        kinwbc_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

    /***************身体朝向任务**********/    
        id = kinwbc_tasks_stand.getId("BodyOriTask");
        //rpy状态的状态差
        kinwbc_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(3);
        Eigen::Matrix3d desRot = Utils::eul2Rot(base_rpy_des(0), base_rpy_des(1), base_rpy_des(2));
        kinwbc_tasks_stand.taskLib[id].errX = Utils::diffRot(base_rot, desRot);
        //rpy状态的速度差
        kinwbc_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(3);
        kinwbc_tasks_stand.taskLib[id].derrX = -dq.block<3, 1>(3, 0);
        //期望状态加速度为零
        kinwbc_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(3);
        //期望速度为零
        kinwbc_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(3);
        //KP权重为100
        kinwbc_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(3, 3) * 500;
        //KD权重为10     
        kinwbc_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(3, 3) * 10;

        kinwbc_tasks_stand.taskLib[id].J = Eigen::MatrixXd::Zero(3, model_nv);
        kinwbc_tasks_stand.taskLib[id].J.block<3, 3>(0, 3) = base_rot.transpose();
        kinwbc_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(3, model_nv);//由于dj和dq的乘积为零，直接定义dj为零，不需要多加计算 3*18 * 18*1
        kinwbc_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

    /**************身体位置任务***********/
        id = kinwbc_tasks_stand.getId("BodyPosTask");
        kinwbc_tasks_stand.taskLib[id].errX = Eigen::VectorXd::Zero(3);
        kinwbc_tasks_stand.taskLib[id].errX = base_pos_des - q.block<3, 1>(0, 0);
        kinwbc_tasks_stand.taskLib[id].derrX = Eigen::VectorXd::Zero(3);
        kinwbc_tasks_stand.taskLib[id].ddxDes = Eigen::VectorXd::Zero(3);
        kinwbc_tasks_stand.taskLib[id].dxDes = Eigen::VectorXd::Zero(3);
        kinwbc_tasks_stand.taskLib[id].kp = Eigen::MatrixXd::Identity(3, 3) * 220; 
        kinwbc_tasks_stand.taskLib[id].kd = Eigen::MatrixXd::Identity(3, 3) * 10;
        kinwbc_tasks_stand.taskLib[id].J = Eigen::MatrixXd::Zero(3, model_nv);
        kinwbc_tasks_stand.taskLib[id].J.block<3, 3>(0, 0) = base_rot.transpose();
        kinwbc_tasks_stand.taskLib[id].dJ = Eigen::MatrixXd::Zero(3, model_nv);//由于dj和dq的乘积为零，直接定义dj为零，不需要多加计算 3*18 * 18*1
        kinwbc_tasks_stand.taskLib[id].W.diagonal() = Eigen::VectorXd::Ones(model_nv);

    }

    if (joy_cmd_ctrl_state == 1)
    {
        kinwbc_tasks_walk.computeAll(des_delta_q, des_dq, des_ddq, dyn_M, dyn_M_inv, dq);
        delta_q_final_kin = kinwbc_tasks_walk.out_delta_q;
        dq_final_kin = kinwbc_tasks_walk.out_dq;
        ddq_final_kin = kinwbc_tasks_walk.out_ddq;
    }
    else if(joy_cmd_ctrl_state == 0)
    {

        kinwbc_tasks_stand.computeAll(des_delta_q, des_dq, des_ddq, dyn_M, dyn_M_inv, dq);
        delta_q_final_kin = kinwbc_tasks_stand.out_delta_q;
        dq_final_kin = kinwbc_tasks_stand.out_dq;
        ddq_final_kin = kinwbc_tasks_stand.out_ddq;
    }
    else
    {
        delta_q_final_kin = Eigen::VectorXd::Zero(model_nv);
        dq_final_kin = Eigen::VectorXd::Zero(model_nv);
        ddq_final_kin = Eigen::VectorXd::Zero(model_nv);
    }
}

//WBC计算关节前馈力矩
//WBC根据广义加速度和地面反力计算关节前馈力矩
//x的状态变量为6 + 12 = 18
//约束
void WBC_priority::computeTau()
{
    //CTe * X + Ce = 0
    //mit,于宪元的论文中的 -Ce
    Eigen::VectorXd eqRes = Eigen::VectorXd::Zero(6);
    //算出余项优化用于松弛变量的计算           
    // -Ce =   -论文Mf * ddq_cmd - Cf + JCF的T * F;
    eqRes = - Sf * dyn_M * ddq_final_kin - Sf * dyn_Non + Sf * Jfe.transpose() * Fr_ff;
            
    //mit,于宪元的论文中的CTe  A矩阵的上部分  
    Eigen::MatrixXd eigen_qp_A1 = Eigen::MatrixXd::Zero(6, 18); 
    eigen_qp_A1.block<6, 6>(0, 0) = Sf * dyn_M * St_qpV1;//6*18 18*18 18*6 M的浮动基参数
    eigen_qp_A1.block<6, 12>(0, 6) = -Sf * Jfe.transpose();//6*18 18*12

    //这里是建立一个矩阵，实现世界坐标到body坐标系的转换（12*12），
    //将支撑脚的姿态转换到body下表示
    Eigen::Matrix3d Rfe;
    Rfe = Eigen::MatrixXd::Identity(3, 3);//可将此修改
    Eigen::Matrix<double, 12, 12> Mw2b;
    Mw2b.setZero();
    Mw2b.block(0, 0, 3, 3) = Rfe.transpose();
    Mw2b.block(3, 3, 3, 3) = Rfe.transpose();
    Mw2b.block(6, 6, 3, 3) = Rfe.transpose();
    Mw2b.block(9, 9, 3, 3) = Rfe.transpose();

    //建立摩擦约束矩阵 
    //mit,于宪元的论文中的 CA
    //    -1  0  mu
    //     0 -1  mu
    //     1  0  mu
    //     0  1  mu 
    //     0  0   1
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(20, 12);
    W(0, 0) = -1;                   W(0, 2) = mu;
                    W(1, 1) =-1;    W(1, 2) = mu;
    W(2, 0) =  1;                   W(2, 2) = mu;
                    W(3, 1) = 1;    W(3, 2) = mu;
                                    W(4, 2) = 1;

    W.block<5, 3>(5 , 3) = W.block<5, 3>(0, 0);
    W.block<5, 3>(10, 6) = W.block<5, 3>(0, 0);
    W.block<5, 3>(15, 9) = W.block<5, 3>(0, 0);
    //将摩擦转换到body坐标系
    W = W * Mw2b;

    // 定义力的上下界约束
    Eigen::VectorXd f_low = Eigen::VectorXd::Zero(20);
    Eigen::VectorXd f_upp = Eigen::VectorXd::Zero(20);

    // 根据 joy_cmd_ctrl_state 和 contacts 判断足端状态
    if (joy_cmd_ctrl_state == 1)
    {
        for(int i=0; i < 4; ++i)
        {
            if(contacts[i])
            {
                // 支撑相
                f_low.block<5, 1>(i * 5, 0) << 0, 0, 0, 0, f_z_low;
                f_upp.block<5, 1>(i * 5, 0) << 1e10, 1e10, 1e10, 1e10, f_z_upp;
            }
            else
            {
                // 摆动相
                f_low.block<5, 1>(i * 5, 0) << -1e-7, -1e-7, -1e-7, -1e-7, -1e-7;
                f_upp.block<5, 1>(i * 5, 0) <<  1e-7,  1e-7,  1e-7,  1e-7,  1e-7;
            }
        }
    }
    else
    {
        f_low.block<5, 1>(0 , 0) << 0, 0, 0, 0, f_z_low;
        f_low.block<5, 1>(5 , 0) << 0, 0, 0, 0, f_z_low;
        f_low.block<5, 1>(10, 0) << 0, 0, 0, 0, f_z_low;
        f_low.block<5, 1>(15, 0) << 0, 0, 0, 0, f_z_low;

        f_upp.block<5, 1>(0 , 0) << 1e10, 1e10, 1e10, 1e10, f_z_upp;
        f_upp.block<5, 1>(5 , 0) << 1e10, 1e10, 1e10, 1e10, f_z_upp;
        f_upp.block<5, 1>(10, 0) << 1e10, 1e10, 1e10, 1e10, f_z_upp;
        f_upp.block<5, 1>(15, 0) << 1e10, 1e10, 1e10, 1e10, f_z_upp;
    }

    //mit,于宪元的论文中的 CTi
    Eigen::MatrixXd eigen_qp_A2 = Eigen::MatrixXd::Zero(20, 18);
    eigen_qp_A2.block<20, 12>(0, 6) = W;

    //ci限制幅度
    Eigen::VectorXd neqRes_low = Eigen::VectorXd::Zero(20);
    Eigen::VectorXd neqRes_upp = Eigen::VectorXd::Zero(20);
    neqRes_low = f_low - W * Fr_ff;
    neqRes_upp = f_upp - W * Fr_ff;

    Eigen::MatrixXd eigen_qp_A_final = Eigen::MatrixXd::Zero(26, 18);
    eigen_qp_A_final.block<6, 18>(0, 0) = eigen_qp_A1;
    eigen_qp_A_final.block<20, 18>(6, 0) = eigen_qp_A2;

    //上界和下界
    Eigen::VectorXd eigen_qp_lbA = Eigen::VectorXd::Zero(26);
    Eigen::VectorXd eigen_qp_ubA = Eigen::VectorXd::Zero(26);

    //等式约束
    eigen_qp_lbA.block<6, 1>(0, 0) = eqRes;
    eigen_qp_ubA.block<6, 1>(0, 0) = eqRes;
    //上下限约束
    eigen_qp_lbA.block<20, 1>(6, 0) = neqRes_low;
    eigen_qp_ubA.block<20, 1>(6, 0) = neqRes_upp;

    //权重参数
    Eigen::MatrixXd eigen_qp_H = Eigen::MatrixXd::Zero(18, 18);
    Q1 = Eigen::MatrixXd::Identity(6, 6);
    Q2 = Eigen::MatrixXd::Identity(12, 12);
    eigen_qp_H.block<6, 6>(0, 0) = Q1 * 1.0;
    eigen_qp_H.block<12, 12>(6, 6) = Q2 * 0.005;
	
    // obj: (1/2)x'Hx+x'g
    // s.t. lbA<=Ax<=ubA
    //      lb <= x<=ub
    //    qpOASES::real_t qp_H[QP_nv*QP_nv];
    //    qpOASES::real_t qp_A[QP_nc*QP_nv];
    //    qpOASES::real_t qp_g[QP_nv];
    //    qpOASES::real_t qp_lbA[QP_nc];
    //    qpOASES::real_t qp_ubA[QP_nc];
    //    qpOASES::real_t xOpt_iniGuess[QP_nv];

    copy_Eigen_to_real_t(qp_H   , eigen_qp_H        , eigen_qp_H.rows()         , eigen_qp_H.cols()         );
    copy_Eigen_to_real_t(qp_A   , eigen_qp_A_final  , eigen_qp_A_final.rows()   , eigen_qp_A_final.cols()   );
    copy_Eigen_to_real_t(qp_lbA , eigen_qp_lbA      , eigen_qp_lbA.rows()       , eigen_qp_lbA.cols()       );
    copy_Eigen_to_real_t(qp_ubA , eigen_qp_ubA      , eigen_qp_ubA.rows()       , eigen_qp_ubA.cols()       );

    for (int i = 0; i < QP_nv; i++)
    {
        xOpt_iniGuess[i] = 0;//清空解的中间变量，防止将上次的计算结果发送
        qp_g[i] = 0;//QP线性项全部为0  wbc没有线性项      
    }

    //最大迭代次数
    nWSR = 500;
    //算法允许的最大cpu运行时间
    cpu_time = timeStep;
    qpOASES::returnValue res;
    res = QP_prob.init(qp_H, qp_g, qp_A, NULL, NULL, qp_lbA, qp_ubA, nWSR, &cpu_time, xOpt_iniGuess);
    qpStatus = qpOASES::getSimpleStatus(res);
    if (res != qpOASES::SUCCESSFUL_RETURN) {
        std::cerr << "QP_prob.init failed with return value: " << res << std::endl;
        // 可以进一步添加其他调试信息
    }

    //存储优化问题的18个解
    //前六个为浮动基加速度，后12个是足端力
    qpOASES::real_t xOpt[QP_nv];
    //获取解
    QP_prob.getPrimalSolution(xOpt);
    //求解成功了，复制求解出的状态，状态是6行1列的body加速度和12行1列的外力
    if (res == qpOASES::SUCCESSFUL_RETURN)
    {
        for (int i = 0; i < QP_nv; i++)
        {
            eigen_xOpt(i) = xOpt[i];
        }
    }
    std::cout << "eigen_xOpt" << std::endl;
    std::cout << eigen_xOpt.transpose() << std::endl;

    //那么新的广义加速度就是基于任务算出的加速度 + 优化后的增量
    eigen_ddq_Opt = ddq_final_kin;
    eigen_ddq_Opt.block<6, 1>(0, 0) += eigen_xOpt.block<6, 1>(0, 0);
    //地面反力就是 MPC得出的地面反力 +优化后的增量
    eigen_fr_Opt = Fr_ff + eigen_xOpt.block<12, 1>(6, 0);

    //检查优化状态并初始化变量，若求解有错
    if (qpStatus != 0)
    {
        Eigen::VectorXd xOpt_iniGuess_m(QP_nv, 1);
        for (int i = 0; i < QP_nv; i++)
        {
            xOpt_iniGuess_m(i) = xOpt_iniGuess[i];
        }
    }

    //直接使用动力学方程计算实体关节力矩
    Eigen::VectorXd tauRes;
    tauRes = dyn_M * eigen_ddq_Opt + dyn_Non - Jfe.transpose() * eigen_fr_Opt;//18

    tauJointRes = tauRes.block(6, 0, model_nv - 6, 1);
    // std::cout << "des_ddq:" << std::endl;
    // std::cout << des_ddq.transpose() << std::endl;
    // std::cout << "des_dq:" << std::endl;
    // std::cout << des_dq.transpose() << std::endl;
    // std::cout << "dyn_Non:" << std::endl;
    std::cout << "delta_q_final_kin" << std::endl;
    std::cout << delta_q_final_kin.transpose() << std::endl;
    std::cout << "dq_final_kin速度:" << std::endl;
    std::cout << dq_final_kin.transpose() << std::endl;
    std::cout << "delta_q_final_kin加速度:" << std::endl;
    std::cout << delta_q_final_kin.transpose() << std::endl;
    std::cout << "Fr_ff:" << std::endl;
    std::cout << Fr_ff.transpose() << std::endl;
    std::cout << "tauRes:" << std::endl;
    std::cout << tauRes.transpose() << std::endl;
    std::cout << "eigen_fr_Opt:" << std::endl;
    std::cout << eigen_fr_Opt.transpose() << std::endl;

    //将当前迭代次数和cpu时间储存起来，以便未来参考
    last_nWSR = nWSR;
    last_cpu_time = cpu_time;
}

//将Eigen的数据转换为qpOASES的数据类型
void WBC_priority::copy_Eigen_to_real_t(qpOASES::real_t *target, const Eigen::MatrixXd &source, int nRows, int nCols)
{
    int count = 0;

    for (int i = 0; i < nRows; i++)
    {
        for (int j = 0; j < nCols; j++)
        {
            target[count++] = isinf(source(i, j)) ? qpOASES::INFTY : source(i, j);
        }
    }
}

//
void WBC_priority::setQini(const Eigen::VectorXd &qIniDesIn, const Eigen::VectorXd &qIniCurIn)
{
    qIniDes = qIniDesIn;
    qIniCur = qIniCurIn;
}

// 计算矩阵的特征值
bool isPositiveDefinite(const Eigen::MatrixXd& matrix) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(matrix);
    if (eigensolver.info() != Eigen::Success) {
        std::cerr << "特征值计算失败" << std::endl;
        return false;
    }
    // 检查所有特征值是否都大于零
    Eigen::VectorXd eigenvalues = eigensolver.eigenvalues();
    for (int i = 0; i < eigenvalues.size(); ++i) {
        if (eigenvalues(i) <= 0) {
            return false;
        }
    }
    return true;
}

//发送数据
void WBC_priority::dataBusWrite(data_bus &robotState)
{

    robotState.tauJointRes = tauJointRes;
    // robotState.wbc_FrRes = eigen_fr_Opt;
    // robotState.qp_cpuTime = cpu_time;
    // robotState.qp_nWSR = nWSR;
    // robotState.qp_status = qpStatus;

    // robotState.wbc_delta_q_final = delta_q_final_kin;
    // robotState.wbc_dq_final = dq_final_kin;
    // robotState.wbc_ddq_final = ddq_final_kin;

    // robotState.qp_status = qpStatus;
    // robotState.qp_nWSR = nWSR;
    // robotState.qp_cpuTime = cpu_time;
}
