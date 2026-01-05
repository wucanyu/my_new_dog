//
// Created by shuoy on 10/19/21.
//

#include "Utils.h"
//将四元数转换为欧拉角（滚转角、俯仰角、偏航角）
Eigen::Vector3d Utils::quat_to_euler(Eigen::Quaterniond quat) {
    Eigen::Vector3d rst;

    // order https://github.com/libigl/eigen/blob/master/Eigen/src/Geometry/Quaternion.h
    Eigen::Matrix<double, 4, 1> coeff = quat.coeffs();
    double x = coeff(0);
    double y = coeff(1);
    double z = coeff(2);
    double w = coeff(3);

    double y_sqr = y*y;

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y_sqr);

    rst[0] = atan2(t0, t1);

    double t2 = +2.0 * (w * y - z * x);
    t2 = t2 > +1.0 ? +1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    rst[1] = asin(t2);

    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y_sqr + z * z);
    rst[2] = atan2(t3, t4);
    return rst;
}

//根据输入的三维向量 vec 生成对应的反对称矩阵
Eigen::Matrix3d Utils::skew(Eigen::Vector3d vec) {
    Eigen::Matrix3d rst; rst.setZero();
    rst <<            0, -vec(2),  vec(1),
            vec(2),             0, -vec(0),
            -vec(1),  vec(0),             0;
    return rst;
}

//计算输入矩阵 mat 的伪逆矩阵
// https://gist.github.com/pshriwise/67c2ae78e5db3831da38390a8b2a209f
Eigen::Matrix3d Utils::pseudo_inverse(const Eigen::Matrix3d &mat) 
{
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double epsilon = std::numeric_limits<double>::epsilon();
    // For a non-square matrix
    // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(mat.cols(), mat.rows()) * svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() *
           svd.matrixU().adjoint();
}

Eigen::MatrixXd Utils::dyn_pseudoInv(const Eigen::MatrixXd &M, const Eigen::MatrixXd &dyn_M, bool isMinv)
{
    double damp = 0;
    Eigen::MatrixXd Minv;

    if (isMinv)
        Minv = dyn_M;
    else
        Minv = dyn_M.llt().solve(Eigen::MatrixXd::Identity(dyn_M.rows(), dyn_M.cols()));

    Eigen::MatrixXd temp = M * Minv * M.transpose();

    temp.diagonal().array() += damp;

    Eigen::MatrixXd res = Minv * M.transpose() * temp.completeOrthogonalDecomposition().pseudoInverse();

    //    Eigen::MatrixXd res = Minv * M.transpose() * temp.inverse();
    return res;
}


Eigen::MatrixXd Utils::pseudoInv_right_weighted(const Eigen::MatrixXd &M, const Eigen::DiagonalMatrix<double, -1> &W)
{
    double damp = 0;
    Eigen::MatrixXd Mres;
    Mres = M * W.inverse() * M.transpose();
    //    Mres=W.inverse()*M.transpose()* pseudoInv_SVD(Mres);
    Mres.diagonal().array() += damp;
    Mres = W.inverse() * M.transpose() * Mres.completeOrthogonalDecomposition().pseudoInverse();
    return Mres;
}


//计算两个平面的二面角
double Utils::cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2) {
    // surface 1: a1 * x + b1 * y + c1 * z + d1 = 0, coef: [a1, b1, c1]
    // surface 2: a2 * x + b2 * y + c2 * z + d2 = 0, coef: [a1, b2, c2]
    double angle_cos =
            abs(surf_coef_1[0] * surf_coef_2[0] + surf_coef_1[1] * surf_coef_2[1] + surf_coef_1[2] * surf_coef_2[2])
            / (sqrt(surf_coef_1[0] * surf_coef_1[0] + surf_coef_1[1] * surf_coef_1[1] + surf_coef_1[2] * surf_coef_1[2]) *
               sqrt(surf_coef_2[0] * surf_coef_2[0] + surf_coef_2[1] * surf_coef_2[1] + surf_coef_2[2] * surf_coef_2[2]));
    return acos(angle_cos);
}

// 该函数实现了矩阵对数旋转的计算，将旋转矩阵 R 转换为轴 - 角表示的向量 omega
Eigen::Vector3d Utils::matrixLogRot(const Eigen::Matrix3d& R) {
    // 计算旋转矩阵的迹相关的值，用于后续计算旋转角度
    double tmp = (R(0, 0) + R(1, 1) + R(2, 2) - 1) / 2;
    double theta;

    // 根据 tmp 的值确定旋转角度 theta
    if (tmp >= 1) {
        // 当 tmp 大于等于 1 时，旋转角度为 0
        theta = 0;
    } else if (tmp <= -1) {
        // 当 tmp 小于等于 -1 时，旋转角度为 π
        theta = 3.1415926;
    } else {
        // 其他情况，使用反余弦函数计算旋转角度
        theta = std::acos(tmp);
    }

    // 初始化轴 - 角表示的向量 omega
    Eigen::Vector3d omega;
    // 计算 omega 的各个分量
    omega << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);

    // 根据旋转角度 theta 的大小进行不同处理
    if (theta > 10e-5) {
        // 当 theta 大于阈值时，使用完整公式计算 omega
        omega = omega * theta / (2 * std::sin(theta));
    } else {
        // 当 theta 小于等于阈值时，使用简化公式计算 omega
        omega = omega / 2;
    }
    return omega;
}    

//欧拉角到旋转矩阵
Eigen::Matrix3d Utils::eul2Rot(double roll, double pitch, double yaw)
{
    Eigen::Matrix3d Rx, Ry, Rz;
    Rz << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;
    Ry << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);
    Rx << 1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);
    return Rz * Ry * Rx;
}

Eigen::Vector3d Utils::diffRot(const Eigen::Matrix3d &Rcur, Eigen::Matrix3d &Rdes)
{
    Eigen::Matrix3d R = Rcur.transpose() * Rdes;
    Eigen::Vector3d w;

    if (R.isDiagonal(1e-5) && fabs(R(0, 0)) + fabs(R(1, 1)) + fabs(R(2, 2)) - 3 < 1e-3)
    {
        w.setZero();
    }
    else if (R.isDiagonal(1e-5))
    {
        w << R(0, 0) + 1, R(1, 1) + 1, R(2, 2) + 1;
        w = w * 3.1415 / 2.0;
    }
    else
    {
        Eigen::Vector3d l;
        l << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
        double sita = atan2(l.norm(), R(0, 0) + R(1, 1) + R(2, 2) - 1);
        w = sita * l / l.norm();
    }
    w = Rcur * w;
    return w;
}

//计算贝塞尔曲线在参数 t 处的值 这里使用的是四阶贝塞尔曲线，其控制点个数为 5 个。
double BezierUtils::bezier_curve(double t, const std::vector<double> &P) {
    std::vector<double> coefficients{1, 4, 6, 4, 1};
    double y = 0;
    for (int i = 0; i <= bezier_degree; i++) {
        y += coefficients[i] * std::pow(t, i) * std::pow(1 - t, bezier_degree - i) * P[i];
    }
    return y;
}

//三次贝塞尔位置
double BezierUtils::cubicBezier(double p0,double pf,double phase)
{
    double pout;
    pout = p0 + (3 * std::pow(phase, 2) - 2 * std::pow(phase, 3) ) * (pf - p0); 
    return pout;
}
//三次贝塞尔速度
double BezierUtils::cubicBezier_v(double p0,double pf,double phase)
{
    double pout;
    pout = 6 * phase * (1 - phase) * (pf - p0); 
    return pout;
}
//三次贝塞尔加速度
double BezierUtils::cubicBezier_a(double p0,double pf,double phase)
{
    double pout;
    pout = (6 - 12 * phase) * (pf - p0);
    return pout;
}

Eigen::Matrix<double, 3, 1> BezierUtils::calFootEndPos( int legID, 
                                                        double dvx, 
                                                        double dvy ,
                                                        double yaw, 
                                                        double dyaw,
                                                        double dYawGoal,
                                                        Eigen::Matrix<double, 3, 1> root_pos,
                                                        Eigen::Matrix<double, 3, 1> root_lin_vel,
                                                        Eigen::Matrix<double, 3, 4> default_foot_pos,
                                                        double T,
                                                        float phase)
{
    Eigen::Matrix<double, 3, 1> _nextStep;
    Eigen::Matrix<double, 3, 1> _footPos;
    Eigen::Matrix<double, 4, 1> _feetRadius;
    Eigen::Matrix<double, 4, 1> _feetInitAngle;

    double _kx = 0.00;
    double _ky = 0.00;
    double _kyaw = 0.00;

    //计算足端中性点的参数
    for(int i(0); i<4; ++i){
        _feetRadius(i)    = sqrt( pow(default_foot_pos(0, i), 2) + pow(default_foot_pos(1, i), 2) );
        _feetInitAngle(i) = atan2(default_foot_pos(1, i), default_foot_pos(0, i));
    }

    _nextStep(0) = root_lin_vel(0) * (1 - phase) * T + root_lin_vel(0) * T * 0.5 + _kx * (root_lin_vel(0) - dvx);
    _nextStep(1) = root_lin_vel(1) * (1 - phase) * T + root_lin_vel(1) * T * 0.5 + _ky * (root_lin_vel(1) - dvy);
    _nextStep(2) = 0;

    double _nextYaw = dyaw * (1 - phase) * T  + dyaw * T * 0.5 + _kyaw * (dYawGoal - dyaw);

    _nextStep(0) += _feetRadius(legID) * cos(yaw + _feetInitAngle(legID) + _nextYaw);
    _nextStep(1) += _feetRadius(legID) * sin(yaw + _feetInitAngle(legID) + _nextYaw);

    //质心位置
    _footPos = root_pos + _nextStep;
    _footPos(2) = 0.0;//假设地面平的

    return _footPos;
}

