#include "priority_tasks.h"

void PriorityTasks::addTask(const char* name) 
{
    // 在 taskLib 向量的末尾添加一个新的 Task 对象，使用传入的 name 进行初始化
    taskLib.emplace_back(name);
    // 为新添加的任务分配一个唯一的 ID，ID 的值为 taskLib 向量的当前大小减 1
    taskLib.back().id = (int)(taskLib.size())-1;
    // 将任务的名称添加到 nameList 向量的末尾
    nameList.emplace_back(name);
}


int PriorityTasks::getId(const std::string &name) {
    for (size_t i = 0; i < nameList.size(); ++i) 
    {
        if (nameList[i]==name) {
            return i;
        }
    }
    return -1;
}

int PriorityTasks::getId(const char* name){
    for (size_t i = 0; i < nameList.size(); ++i) 
    {
        if (nameList[i]==name) {
            return i;
        }
    }
    printf("Cannot find wbc task: %s !\n", name);
    return -1;
}

void PriorityTasks::buildPriority(const std::vector<std::string> &taskOrder) {
    startId= getId(taskOrder[0]);
    for (int i=0;i<taskOrder.size();i++)
    {
        int idCur= getId(taskOrder[i]);
        if (i==0)
        {
            taskLib[idCur].parentId=-1;
        }
        else
        {
            int idBef= getId(taskOrder[i-1]);
            taskLib[idCur].parentId=idBef;
        }

        if (i == taskOrder.size()-1)
        {
            taskLib[idCur].childId=-1;
        }
        else
        {
            int idNxt= getId(taskOrder[i+1]);
            taskLib[idCur].childId=idNxt;
        }
    }
}

void PriorityTasks::printTaskInfo() {
    for (int i=0;i<taskLib.size();i++)
    {
        printf("-------------\n");
        printf("taskName=%s\n",taskLib[i].taskName.c_str());
        printf("parentId=%d, childId=%d\n",taskLib[i].parentId,taskLib[i].childId);
    }
}

void PriorityTasks::computeAll(const Eigen::VectorXd &des_delta_q,const Eigen::VectorXd &des_dq, const Eigen::VectorXd &des_ddq, const Eigen::MatrixXd &dyn_M, const Eigen::MatrixXd &dyn_M_inv, const Eigen::VectorXd &dq) {
    int curId=startId;
    int parentId=taskLib[curId].parentId;
    int childId=taskLib[curId].childId;
    for (int i=0;i<taskLib.size();i++)
    {
        //如果是最高优先级
        if (parentId==-1)
        {
            //将最高优先级任务的N(也就是该任务所被允许的零空间)设定为1，可以为所欲为的操作所有关节。雅克比矩阵的列数是关节数量(广义坐标)
            taskLib[curId].N = Eigen::MatrixXd::Identity(taskLib[curId].J.cols(),taskLib[curId].J.cols());
            //该任务的雅克比矩阵就是J*N(对于所有优先级任务的通用迭代写法)，就是该任务在父任务的(或父任务的综合)零空间下的允许的雅克比任务矩阵
            taskLib[curId].Jpre = taskLib[curId].J * taskLib[curId].N;
            //广义位置的增量就是期望的广义位置增量 + 零空间优化后的调整
            //(位置任务)，这里实际上是利用deata_q = q_des - d_cur(这里算增量利用速度雅克比的公式
            //进行公示转换后就是delta_q = des_delta_q (这个一般是父任务算完后的结果)+ 父任务零空任务的解算。(北航论文，应该是MIT原始论文公式)
            //如里是最后-那最终的期望的发给关节的位置就是 前位置(q_cur)+关节位置差(delta_q)
            taskLib[curId].delta_q = des_delta_q + Utils::pseudoInv_right_weighted(taskLib[curId].Jpre,taskLib[curId].W)*taskLib[curId].errX;
            //最高优先级的速度就直接是期望速度，一般是上个任务优化后的期望速鹰
            taskLib[curId].dq = des_dq;
            //这里使用自己定的PD控制设定了加速度指令(显然，这取决于机器人本身调节状态的基本能力)
            //这里是给定了当前任务状态的跟踪力度。
            Eigen::VectorXd ddxcmd = taskLib[curId].ddxDes + taskLib[curId].kp * taskLib[curId].errX + taskLib[curId].kd*taskLib[curId].derrX;
            //广义加速度，这个严格按公式走的
            //第一个任务全为零
            taskLib[curId].ddq = des_ddq + 
            Utils::dyn_pseudoInv(taskLib[curId].Jpre,dyn_M_inv,true) * (ddxcmd - taskLib[curId].dJ * dq);  
        }
        else
        {
            //先算零空间(单位矩阵-高任务零空间合集的伪逆*高任务零空间合集)
            //注:这里是带有权重的零空间，并不是完全的零空间，
            //这里考虑了父任务的权重，如果是1，那就是标准的零空间，Jpre是迭代算的，所以所有高优先级都是这么算的
            taskLib[curId].N = taskLib[parentId].N*
            (Eigen::MatrixXd::Identity(taskLib[parentId].Jpre.cols(),taskLib[parentId].Jpre.cols()) - 
            Utils::pseudoInv_right_weighted(taskLib[parentId].Jpre,taskLib[parentId].W)*taskLib[parentId].Jpre);
            //再算当前任务在零空间优化下的任务雅克比
            taskLib[curId].Jpre = taskLib[curId].J * taskLib[curId].N;
            //广义位置增量严格按公式走，多了当前任务权市
            taskLib[curId].delta_q = taskLib[parentId].delta_q + 
            Utils::pseudoInv_right_weighted(taskLib[curId].Jpre,taskLib[curId].W) * 
            (taskLib[curId].errX - taskLib[curId].J * taskLib[parentId].delta_q);
            ///广义速度:严格按公式走。多了权重
            taskLib[curId].dq = taskLib[parentId].dq + 
            Utils::pseudoInv_right_weighted(taskLib[curId].Jpre,taskLib[curId].W) * 
            (taskLib[curId].dxDes- taskLib[curId].J*taskLib[parentId].dq);
            ///期望的任务空间的加速度
            Eigen::VectorXd ddxcmd = taskLib[curId].ddxDes + 
            taskLib[curId].kp * taskLib[curId].errX + taskLib[curId].kd * taskLib[curId].derrX;
            ////广义加速度，这个严格按公式走的
            taskLib[curId].ddq= taskLib[parentId].ddq + 
            Utils::dyn_pseudoInv(taskLib[curId].Jpre,dyn_M_inv,true) *
            (ddxcmd - taskLib[curId].dJ * dq - taskLib[curId].J * taskLib[parentId].ddq);

        }

        if (childId!=-1){
            parentId=curId;
            curId=childId;
            childId=taskLib[curId].childId;
        }
        else
            break;
    }
    //给关节的期望位置就是当前位置加位置增量
    out_delta_q = taskLib[curId].delta_q;
    //给关节的期望速度就是最后算出来的速度
    out_dq = taskLib[curId].dq;
    //加速度不给关节，用来和外力一起计算关节力矩
    out_ddq = taskLib[curId].ddq;
}













