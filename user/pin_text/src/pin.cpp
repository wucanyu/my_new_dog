#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

int main(int argc, char **argv)
{
    // 加载 URDF 文件，构建机器人模型
    const std::string urdf_filename = "/home/wu/my_dog_8/model/new_dog/meshes/new_dog.urdf";

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);
    // 创建数据结构，存放中间结果
    pinocchio::Data data(model);

    // 定义机器人状态(假设机器人有 model.nq 个自由度)
    Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
    Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

    // 计算前向运动学
    pinocchio::forwardKinematics(model, data, q, v);
    pinocchio::updateFramePlacements(model, data);

    // 输出各关节的位置
    std::cout << "Joint Positions:" << std::endl;
    for (std::size_t i = 0; i < model.njoints; ++i) {
        std::cout << "Joint name: " << model.names[i] << "\n";
        std::cout << "Placement: \n" << data.oMi[i] << "\n\n";
    }

    // 计算雅可比矩阵
    std::cout << "Jacobians:" << std::endl;
    Eigen::MatrixXd jacobian(6, model.nv);
    for (std::size_t i = 0; i < model.njoints; ++i) {
        std::cout << "Joint name: " << model.names[i] << "\n";
        pinocchio::computeJointJacobian(model, data, q, i, jacobian);
        std::cout << "Jacobian for joint " << model.names[i] << ":\n" << jacobian << "\n\n";
    }

    return 0;
}    