//
// Created by shuoy on 10/19/21.
//

#ifndef CPP_UTILS_H
#define CPP_UTILS_H

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

#include "base_struct.h"

class Utils {
public:
    // compare to Eigen's default eulerAngles
    // this function returns yaw angle within -pi to pi
    static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat);
    static Eigen::Matrix3d skew(Eigen::Vector3d vec);
    static Eigen::Matrix3d pseudo_inverse(const Eigen::Matrix3d &mat);
    static double cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2);
    static Eigen::Vector3d matrixLogRot(const Eigen::Matrix3d& R); 
    static Eigen::MatrixXd pseudoInv_right_weighted(const Eigen::MatrixXd &M, const Eigen::DiagonalMatrix<double, -1> &W);
    static Eigen::MatrixXd dyn_pseudoInv(const Eigen::MatrixXd &M, const Eigen::MatrixXd &dyn_M, bool isMinv);
    static Eigen::Matrix3d eul2Rot(double roll, double pitch, double yaw);
    static Eigen::Vector3d diffRot(const Eigen::Matrix3d &Rcur, Eigen::Matrix3d &Rdes);
};

class BezierUtils {
    // TODO: allow degree change? may not be necessary, we can stick to degree 4
public:
    BezierUtils () {
        curve_constructed = false;
        bezier_degree = 4;
    }

    //三次贝塞尔位置
    Eigen::Vector3d cubicBezier(Eigen::Vector3d p0,Eigen::Vector3d pf,float phase);
    //三次贝塞尔速度
    Eigen::Vector3d cubicBezier_v(Eigen::Vector3d p0,Eigen::Vector3d pf,float phase);
    //三次贝塞尔加速度
    Eigen::Vector3d cubicBezier_a(Eigen::Vector3d p0,Eigen::Vector3d pf,float phase);

    //三次贝塞尔位置
    double cubicBezier(double p0,double pf,double phase);
    //三次贝塞尔速度
    double cubicBezier_v(double p0,double pf,double phase);
    //三次贝塞尔加速度
    double cubicBezier_a(double p0,double pf,double phase);

    //  摆动轨迹计算
    //  起点 终点 相位 摆动时间 脚抬起的最大高度 存储输出的足端位置 存储输出的足端速度 存储输出的足端加速度
    void SwingTrajectoryBezier(const Eigen::Vector3d& pf_init, const Eigen::Vector3d& pf_final, double phase, double swingtime, double h,
                               Eigen::Vector3d& pout, Eigen::Vector3d& p_v, Eigen::Vector3d& p_a);

    // set of functions create bezier curves, get points, reset
    Eigen::Vector3d get_foot_pos_curve(float t,
                                       Eigen::Vector3d foot_pos_start,
                                       Eigen::Vector3d foot_pos_final,
                                       double terrain_pitch_angle);

    bool reset_foot_pos_curve() {curve_constructed = false;}

    Eigen::Matrix<double, 3, 1> calFootEndPos(  int legID, 
                                                double dvx, 
                                                double dvy ,
                                                double yaw, 
                                                double dyaw,
                                                double dYawGoal,
                                                Eigen::Matrix<double, 3, 1> root_pos,
                                                Eigen::Matrix<double, 3, 1> root_lin_vel,
                                                Eigen::Matrix<double, 3, 4> default_foot_pos,
                                                double T,
                                                float phase);

private:
    double bezier_curve(double t, const std::vector<double> &P);

    bool curve_constructed;
    float bezier_degree;
};

#endif //A1_CPP_UTILS_H
