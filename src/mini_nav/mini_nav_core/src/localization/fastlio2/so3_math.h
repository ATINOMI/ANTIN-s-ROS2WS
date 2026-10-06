/**
 * @file so3_math.h
 * @brief 三维叉乘矩阵、SO(3) 指数/对数映射与弧度制欧拉角转换。
 * @author Antinomy
 * @date 2026-10-05
 */
#ifndef SO3_MATH_H
#define SO3_MATH_H

#include <math.h>
#include <Eigen/Core>

#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0

/**
 * @brief 构造三维向量的反对称叉乘矩阵。
 * @param v 三维向量。
 * @return 满足 skew_sym_mat(v)·a=v×a 的 3×3 矩阵。
 */
template<typename T>
Eigen::Matrix<T, 3, 3> skew_sym_mat(const Eigen::Matrix<T, 3, 1> &v)
{
    Eigen::Matrix<T, 3, 3> skew_sym_mat;
    skew_sym_mat<<0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0;
    return skew_sym_mat;
}

/**
 * @brief 使用轴角 Rodrigues 公式生成旋转矩阵。
 * @param ang 旋转向量，方向为旋转轴、模长为转角（rad）；此重载接受右值。
 * @return 3×3 旋转矩阵；向量/角速度模长不大于 1e-7 时返回单位阵，分量重载阈值为 1e-5。
 * @note 不进行有限值检查；阈值以下直接置为单位阵，未保留一阶微小旋转。
 */
template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &&ang)
{
    /* Rodrigues 公式 R=I+sin(theta)[u]×+(1-cos(theta))[u]×²；先判断模长，避免零轴归一化。 */
    T ang_norm = ang.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (ang_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang / ang_norm;
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(r_axis);
        /// Roderigous Tranformation
        return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

/**
 * @brief 使用轴角 Rodrigues 公式生成旋转矩阵。
 * @param ang_vel 角速度向量（rad/s）；小量判断针对角速度模长。
 * @param dt 积分时间（s），用于计算转角 |ang_vel|·dt。
 * @return 3×3 旋转矩阵；向量/角速度模长不大于 1e-7 时返回单位阵，分量重载阈值为 1e-5。
 * @note 不进行有限值检查；阈值以下直接置为单位阵，未保留一阶微小旋转。
 */
template<typename T, typename Ts>
Eigen::Matrix<T, 3, 3> Exp(const Eigen::Matrix<T, 3, 1> &ang_vel, const Ts &dt)
{
    T ang_vel_norm = ang_vel.norm();
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();

    if (ang_vel_norm > 0.0000001)
    {
        Eigen::Matrix<T, 3, 1> r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix<T, 3, 3> K;

        K << SKEW_SYM_MATRX(r_axis);

        T r_ang = ang_vel_norm * dt;

        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

/**
 * @brief 使用轴角 Rodrigues 公式生成旋转矩阵。
 * @param v1 旋转向量 x 分量（rad）。
 * @param v2 旋转向量 y 分量（rad）。
 * @param v3 旋转向量 z 分量（rad）。
 * @return 3×3 旋转矩阵；向量/角速度模长不大于 1e-7 时返回单位阵，分量重载阈值为 1e-5。
 * @note 不进行有限值检查；阈值以下直接置为单位阵，未保留一阶微小旋转。
 */
template<typename T>
Eigen::Matrix<T, 3, 3> Exp(const T &v1, const T &v2, const T &v3)
{
    T &&norm = sqrt(v1 * v1 + v2 * v2 + v3 * v3);
    Eigen::Matrix<T, 3, 3> Eye3 = Eigen::Matrix<T, 3, 3>::Identity();
    if (norm > 0.00001)
    {
        T r_ang[3] = {v1 / norm, v2 / norm, v3 / norm};
        Eigen::Matrix<T, 3, 3> K;
        K << SKEW_SYM_MATRX(r_ang);

        /// Roderigous Tranformation
        return Eye3 + std::sin(norm) * K + (1.0 - std::cos(norm)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

/* Logrithm of a Rotation Matrix */
/**
 * @brief 从旋转矩阵提取主值旋转向量。
 * @param R 应为正交且行列式为 1 的旋转矩阵。
 * @return 旋转轴乘转角（rad）的三维向量；|theta|<0.001 时采用反对称部分的一阶近似。
 * @note 一般分支包含 theta/sin(theta)，接近 π 时数值退化；acos 输入未做完整区间裁剪。
 */
template<typename T>
Eigen::Matrix<T,3,1> Log(const Eigen::Matrix<T, 3, 3> &R)
{
    T theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
    Eigen::Matrix<T,3,1> K(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
    return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

/**
 * @brief 按 R=Rz(yaw)·Ry(pitch)·Rx(roll) 提取欧拉角。
 * @param rot 有效三维旋转矩阵。
 * @return 依次为 roll、pitch、yaw 的弧度向量。
 * @note sy<1e-6 时视为俯仰奇异，固定 yaw=0 并将剩余旋转合并到 roll。
 */
template<typename T>
Eigen::Matrix<T, 3, 1> RotMtoEuler(const Eigen::Matrix<T, 3, 3> &rot)
{
    T sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
    /* sy=|cos(pitch)|；奇异处 roll/yaw 无法唯一分离，必须保持固定 yaw 的同一分支约定。 */
    bool singular = sy < 1e-6;
    T x, y, z;
    if(!singular)
    {
        x = atan2(rot(2, 1), rot(2, 2));
        y = atan2(-rot(2, 0), sy);   
        z = atan2(rot(1, 0), rot(0, 0));  
    }
    else
    {    
        x = atan2(-rot(1, 2), rot(1, 1));    
        y = atan2(-rot(2, 0), sy);    
        z = 0;
    }
    Eigen::Matrix<T, 3, 1> ang(x, y, z);
    return ang;
}

#endif
