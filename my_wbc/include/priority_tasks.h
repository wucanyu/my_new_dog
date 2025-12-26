#pragma once

#include <Eigen/Dense>
#include "Utils.h"
#include <utility>
#include <vector>
#include <string>
#include <iostream>
#include "Utils.h"
struct Task{
    //任务的名称
    std::string taskName;
    //任务的唯一标识符
    int id;
    //用于构建任务之间的优先级关系，parentId 指向父任务，childId 指向子任务
    int parentId, childId;
    //任务的期望位移和期望加速度
    Eigen::VectorXd dxDes,ddxDes;
    //任务的关节角度变化、关节速度和关节加速度
    Eigen::VectorXd delta_q, dq, ddq;
    //雅可比矩阵、雅可比矩阵的导数和预处理后的雅可比矩阵
    Eigen::MatrixXd J, dJ, Jpre;
    //零空间投影矩阵
    Eigen::MatrixXd N;
    //比例和微分增益矩阵。
    Eigen::MatrixXd kp, kd;
    //用于伪逆计算的加权矩阵
    Eigen::DiagonalMatrix<double,-1> W; // weighted matrix for pseudo inverse
    //任务的位置误差和速度误差
    Eigen::VectorXd errX, derrX;

    Task(std::string name){taskName=name;};
};

class PriorityTasks {
public:
    //存储所有任务的向量
    std::vector<Task> taskLib;
    //存储任务名称
    std::vector<std::string> nameList;
    //任务 ID
    std::vector<int> idList, parentIdList, childIdList;
    //任务 ID、父任务 ID 和子任务 ID 的向量。
    Eigen::VectorXd out_delta_q, out_dq, out_ddq;
    //优先级最高的任务 ID
    int startId;
    //向任务库中添加一个新任务，并分配 ID 和名称
    void addTask(const char* name);
    //根据任务名称查找任务 ID，如果找不到则返回 - 1。
    int getId(const std::string& name);
    int getId(const char* name);
    //根据给定的任务顺序构建任务之间的优先级关系，设置每个任务的 parentId 和 childId。
    void buildPriority(const std::vector<std::string> &taskOrder);
    //计算所有任务的关节角度变化、关节速度和关节加速度。根据任务的优先级关系，依次计算每个任务的相关变量，并更新最终输出。
    void computeAll(const Eigen::VectorXd &des_delta_q,const Eigen::VectorXd &des_dq, const Eigen::VectorXd &des_ddq, const Eigen::MatrixXd &dyn_M
    ,const Eigen::MatrixXd &dyn_M_inv, const Eigen::VectorXd &dq);
    void printTaskInfo();
};



