// This is an advanced implementation of the algorithm described in the
// following paper:
//    C. Hertzberg,  R.  Wagner,  U.  Frese,  and  L.  Schroder.  Integratinggeneric   sensor   fusion   algorithms   with   sound   state   representationsthrough  encapsulation  of  manifolds.
//    CoRR,  vol.  abs/1107.1119,  2011.[Online]. Available: http://arxiv.org/abs/1107.1119

/*
 *  Copyright (c) 2019--2023, The University of Hong Kong
 *  All rights reserved.
 *
 *  Modifier: Dongjiao HE <hdj65822@connect.hku.hk>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
 
/*
 *  Copyright (c) 2008--2011, Universitaet Bremen
 *  All rights reserved.
 *
 *  Author: Christoph Hertzberg <chtz@informatik.uni-bremen.de>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file mtk/types/SOn.hpp
 * @brief Standard Orthogonal Groups i.e.\ rotatation groups.
 */
/**
 * @file SOn.hpp
 * @brief 二维角度 SO2 与三维单位四元数 SO3 的流形接口。SO3 采用右乘扰动，系数流输入/输出顺序为 x、y、z、w。
 * @author Antinomy
 * @date 2026-10-05
 */
#ifndef SON_H_
#define SON_H_

#include <Eigen/Geometry>

#include "vect.hpp"
#include "../src/mtkmath.hpp"


namespace MTK {


/**
 * Two-dimensional orientations represented as scalar.
 * There is no guarantee that the representing scalar is within any interval,
 * but the result of boxminus will always have magnitude @f$\le\pi @f$.
 */
/**
 * @brief 以角度标量表示二维方向；存储角度不强制归一，差分结果归一到 ±π。
 */
template<class _scalar = double, int Options = Eigen::AutoAlign>
struct SO2 : public Eigen::Rotation2D<_scalar> {
	enum {DOF = 1, DIM = 2, TYP = 3};
	
	typedef _scalar scalar;
	typedef Eigen::Rotation2D<scalar> base;
	typedef vect<DIM, scalar, Options> vect_type;
	
	//! Construct from angle
	/**
	 * @brief 构造二维方向。
	 * @param angle 初始角度（rad），默认 0。
	 */
	SO2(const scalar& angle = 0) : base(angle) {	}
	
	//! Construct from Eigen::Rotation2D
	/**
	 * @brief 构造二维方向。
	 * @param src 已有 Eigen 二维旋转。
	 */
	SO2(const base& src) : base(src) {}
	
	/**
	 * Construct from 2D vector.
	 * Resulting orientation will rotate the first unit vector to point to vec.
	 */
	/**
	 * @brief 构造二维方向。
	 * @param vec 二维方向向量，以 atan2(y,x) 确定角度。
	 */
	SO2(const vect_type &vec) : base(atan2(vec[1], vec[0])) {};
	
	
	//! Calculate @c this->inverse() * @c r
	/**
	 * @brief 对输入施加当前旋转的逆。
	 * @param r 待复合旋转。
	 * @return this⁻¹·r 或 this⁻¹·vec；SO3 用共轭实现，依赖单位四元数。
	 */
	SO2 operator%(const base &r) const {
		return base::inverse() * r;
	}

	//! Calculate @c this->inverse() * @c r
	/**
	 * @brief 对输入施加当前旋转的逆。
	 * @param vec 待逆旋转向量。
	 * @return this⁻¹·r 或 this⁻¹·vec；SO3 用共轭实现，依赖单位四元数。
	 */
	template<class Derived>
	vect_type operator%(const Eigen::MatrixBase<Derived> &vec) const {
		return base::inverse() * vec;
	}
	
	//! Calculate @c *this * @c r.inverse()
	/**
	 * @brief 复合当前旋转与输入的逆。
	 * @param r 作为右侧逆旋转的输入。
	 * @return this·r⁻¹；SO3 以共轭充当逆。
	 */
	SO2 operator/(const SO2 &r) const {
		return *this * r.inverse();
	}
	
	//! Gets the angle as scalar.
	/**
	 * @brief 读取 SO2 的当前角度。
	 * @return 存储角度（rad），未执行周期归一化。
	 */
	operator scalar() const {
		return base::angle(); 
	}
	/**
	 * @brief 为非 S2 类型提供组合流形分派占位。
	 * @param res 写入 3×3 零矩阵；本类型没有球面方向叉乘量。
	 */
	void S2_hat(Eigen::Matrix<scalar, 3, 3> &res)
	{
		res = Eigen::Matrix<scalar, 3, 3>::Zero();
	}
	//! @name Manifold requirements
	/**
	 * @brief 拒绝将非 S2 分量作为球面差分处理。
	 * @param res 保持统一接口的输出参数，正常流程不应调用本分支。
	 * @note 打印 wrong idx for S2 后 exit(100)；后面的赋零语句不会执行。
	 */
	void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3> &res)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 2, 3>::Zero();
	}

	/**
	 * @brief 拒绝将非 S2 分量作为球面更新处理。
	 * @param res 接口占位输出，进程退出前不写入有效结果。
	 * @param delta 接口占位的二维误差，当前未使用。
	 * @note 打印错误后 exit(100)，不抛出异常；必须保证组合状态 S2 索引表正确。
	 */
	void S2_Mx(Eigen::Matrix<scalar, 3, 2> &res, MTK::vectview<const scalar, 2> delta)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 3, 2>::Zero();
	}

	/**
	 * @brief 按过程增量更新当前流形值。
	 * @param vec 过程增量，具体加法、对数或旋转含义与当前类型一致。
	 * @param scale 过程增量缩放量。
	 * @note 此文件各类型的 oplus 与其 boxplus 使用相同更新公式。
	 */
	void oplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1) {
		base::angle() += scale * vec[0];
	}
	
	/**
	 * @brief 用局部误差更新当前流形值。
	 * @param vec 局部误差：普通数值按加法，PositiveScalar 按对数，SO2/SO3 按弧度。
	 * @param scale 误差缩放系数。
	 * @note SO2 加到角度；SO3 右乘 exp(scale·vec)。右乘约定对应本体局部误差，不能独立改为左乘。
	 */
	void boxplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1) {
		base::angle() += scale * vec[0];
	}
	/**
	 * @brief 计算当前值相对 other 的局部误差。
	 * @param res 写入差分结果，不分配额外输出存储。
	 * @param other 局部误差参考值。
	 * @note SO2 使用周期归一化角差；SO3 使用 log(other.conjugate()·this)，要求四元数有效且归一化。
	 */
	void boxminus(MTK::vectview<scalar, DOF> res, const SO2<scalar>& other) const {
		res[0] = MTK::normalize(base::angle() - other.angle(), scalar(MTK::pi));
	}
	
	/**
	 * @brief 从流中读取方向表示。
	 * @param is 输入流。
	 * @param ang SO2 目标，直接读取角度（rad）。
	 * @return 原输入流引用。
	 * @note SO3 输入应非零，函数未单独处理流失败或全零输入。
	 */
	friend std::istream& operator>>(std::istream &is, SO2<scalar>& ang){
		return is >> ang.angle();
	}

};


/**
 * Three-dimensional orientations represented as Quaternion.
 * It is assumed that the internal Quaternion always stays normalized,
 * should this not be the case, call inherited member function @c normalize().
 */
/**
 * @brief 以单位四元数表示三维方向，局部旋转自由度为 3。
 * @note 运算使用共轭充当逆，依赖单位模长；只有指定构造/流输入执行归一化，不能假设所有赋值都归一。
 */
template<class _scalar = double, int Options = Eigen::AutoAlign>
struct SO3 : public Eigen::Quaternion<_scalar, Options> {
	enum {DOF = 3, DIM = 3, TYP = 2}; 
	typedef _scalar scalar;
	typedef Eigen::Quaternion<scalar, Options> base;
	typedef Eigen::Quaternion<scalar> Quaternion;
	typedef vect<DIM, scalar, Options> vect_type;
	
	//! Calculate @c this->inverse() * @c r
	/**
	 * @brief 对输入施加当前旋转的逆。
	 * @param r 待复合旋转。
	 * @return this⁻¹·r 或 this⁻¹·vec；SO3 用共轭实现，依赖单位四元数。
	 */
	template<class OtherDerived> EIGEN_STRONG_INLINE 
	Quaternion operator%(const Eigen::QuaternionBase<OtherDerived> &r) const {
		return base::conjugate() * r;
	}
	
	//! Calculate @c this->inverse() * @c r
	/**
	 * @brief 对输入施加当前旋转的逆。
	 * @param vec 待逆旋转向量。
	 * @return this⁻¹·r 或 this⁻¹·vec；SO3 用共轭实现，依赖单位四元数。
	 */
	template<class Derived>
	vect_type operator%(const Eigen::MatrixBase<Derived> &vec) const {
		return base::conjugate() * vec;
	}
	
	//! Calculate @c this * @c r.conjugate()
	/**
	 * @brief 复合当前旋转与输入的逆。
	 * @param r 作为右侧逆旋转的输入。
	 * @return this·r⁻¹；SO3 以共轭充当逆。
	 */
	template<class OtherDerived> EIGEN_STRONG_INLINE 
	Quaternion operator/(const Eigen::QuaternionBase<OtherDerived> &r) const {
		return *this * r.conjugate();
	}
	
	/**
	 * Construct from real part and three imaginary parts.
	 * Quaternion is normalized after construction.
	 */
	/**
	 * @brief 构造三维旋转四元数。
	 * @param w 实部。
	 * @param x 虚部 x。
	 * @param y 虚部 y。
	 * @param z 虚部 z。
	 * @note 四分量重载调用 normalize；其他重载要求源数据已经表示合法旋转。
	 */
	SO3(const scalar& w, const scalar& x, const scalar& y, const scalar& z) : base(w, x, y, z) {
		base::normalize();
	}
	
	/**
	 * Construct from Eigen::Quaternion.
	 * @note Non-normalized input may result result in spurious behavior.
	 */
	/**
	 * @brief 构造三维旋转四元数。
	 * @param src 单位四元数，默认 Identity；该重载不归一化。
	 * @note 四分量重载调用 normalize；其他重载要求源数据已经表示合法旋转。
	 */
	SO3(const base& src = base::Identity()) : base(src) {}
	
	/**
	 * Construct from rotation matrix.
	 * @note Invalid rotation matrices may lead to spurious behavior.
	 */
	/**
	 * @brief 构造三维旋转四元数。
	 * @param matrix 有效 3×3 旋转矩阵。
	 * @note 四分量重载调用 normalize；其他重载要求源数据已经表示合法旋转。
	 */
	template<class Derived>
	SO3(const Eigen::MatrixBase<Derived>& matrix) : base(matrix) {}
	
	/**
	 * Construct from arbitrary rotation type.
	 * @note Invalid rotation matrices may lead to spurious behavior.
	 */
	/**
	 * @brief 构造三维旋转四元数。
	 * @param rotation 有效三维旋转表达式。
	 * @note 四分量重载调用 normalize；其他重载要求源数据已经表示合法旋转。
	 */
	template<class Derived>
	SO3(const Eigen::RotationBase<Derived, 3>& rotation) : base(rotation.derived()) {}
	
	//! @name Manifold requirements
	
	/**
	 * @brief 用局部误差更新当前流形值。
	 * @param vec 局部误差：普通数值按加法，PositiveScalar 按对数，SO2/SO3 按弧度。
	 * @param scale 误差缩放系数。
	 * @note SO2 加到角度；SO3 右乘 exp(scale·vec)。右乘约定对应本体局部误差，不能独立改为左乘。
	 */
	void boxplus(MTK::vectview<const scalar, DOF> vec, scalar scale=1) {
		SO3 delta = exp(vec, scale);
		/* 右乘增量：局部误差在当前姿态的本体坐标表达，滤波雅可比和协方差变换依赖同一约定。 */
		*this = *this * delta;
	}
	/**
	 * @brief 计算当前值相对 other 的局部误差。
	 * @param res 写入差分结果，不分配额外输出存储。
	 * @param other 局部误差参考值。
	 * @note SO2 使用周期归一化角差；SO3 使用 log(other.conjugate()·this)，要求四元数有效且归一化。
	 */
	void boxminus(MTK::vectview<scalar, DOF> res, const SO3<scalar>& other) const {
		res = SO3::log(other.conjugate() * *this);
	}
	//}

	/**
	 * @brief 按过程增量更新当前流形值。
	 * @param vec 过程增量，具体加法、对数或旋转含义与当前类型一致。
	 * @param scale 过程增量缩放量。
	 * @note 此文件各类型的 oplus 与其 boxplus 使用相同更新公式。
	 */
	void oplus(MTK::vectview<const scalar, DOF> vec, scalar scale=1) {
		SO3 delta = exp(vec, scale);
		/* 右乘增量：局部误差在当前姿态的本体坐标表达，滤波雅可比和协方差变换依赖同一约定。 */
		*this = *this * delta;
	}

	/**
	 * @brief 为非 S2 类型提供组合流形分派占位。
	 * @param res 写入 3×3 零矩阵；本类型没有球面方向叉乘量。
	 */
	void S2_hat(Eigen::Matrix<scalar, 3, 3> &res)
	{
		res = Eigen::Matrix<scalar, 3, 3>::Zero();
	}
	/**
	 * @brief 拒绝将非 S2 分量作为球面差分处理。
	 * @param res 保持统一接口的输出参数，正常流程不应调用本分支。
	 * @note 打印 wrong idx for S2 后 exit(100)；后面的赋零语句不会执行。
	 */
	void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3> &res)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 2, 3>::Zero();
	}

	/**
	 * @brief 拒绝将非 S2 分量作为球面更新处理。
	 * @param res 接口占位输出，进程退出前不写入有效结果。
	 * @param delta 接口占位的二维误差，当前未使用。
	 * @note 打印错误后 exit(100)，不抛出异常；必须保证组合状态 S2 索引表正确。
	 */
	void S2_Mx(Eigen::Matrix<scalar, 3, 2> &res, MTK::vectview<const scalar, 2> delta)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 3, 2>::Zero();
	}

	/**
	 * @brief 按 x、y、z、w 顺序输出四元数系数。
	 * @param os 输出流。
	 * @param q 待输出单位四元数。
	 * @return 原输出流引用。
	 */
	friend std::ostream& operator<<(std::ostream &os, const SO3<scalar, Options>& q){
		return os << q.coeffs().transpose() << " ";
	}

	/**
	 * @brief 从流中读取方向表示。
	 * @param is 输入流。
	 * @param q SO3 目标，读取 x、y、z、w 后归一化。
	 * @return 原输入流引用。
	 * @note SO3 输入应非零，函数未单独处理流失败或全零输入。
	 */
	friend std::istream& operator>>(std::istream &is, SO3<scalar, Options>& q){
		vect<4,scalar> coeffs;
		is >> coeffs;
		q.coeffs() = coeffs.normalized();
		return is;
	}
	
	//! @name Helper functions
	//{
	/**
	 * Calculate the exponential map. In matrix terms this would correspond 
	 * to the Rodrigues formula.
	 */
	// FIXME vectview<> can't be constructed from every MatrixBase<>, use const Vector3x& as workaround
//	static SO3 exp(MTK::vectview<const scalar, 3> dvec, scalar scale = 1){
	/**
	 * @brief 将旋转向量映射为单位四元数。
	 * @param dvec 三维旋转向量（rad）。
	 * @param scale 旋转角缩放系数，计算时正确使用 scale/2 半角。
	 * @return 表示 Exp(scale·dvec) 的 SO3 四元数。
	 */
	static SO3 exp(const Eigen::Matrix<scalar, 3, 1>& dvec, scalar scale = 1){
		SO3 res;
		res.w() = MTK::exp<scalar, 3>(res.vec(), dvec, scalar(scale/2));
		return res;
	}
	/**
	 * Calculate the inverse of @c exp.
	 * Only guarantees that <code>exp(log(x)) == x </code>
	 */
	/**
	 * @brief 提取四元数的局部旋转向量。
	 * @param orient 有效单位四元数；q 与 -q 表示同一方向。
	 * @return 主值三维旋转向量（rad）；在 π 边界不保证轴符号唯一。
	 */
	static typename base::Vector3 log(const SO3 &orient){
		typename base::Vector3 res;
		MTK::log<scalar, 3>(res, orient.w(), orient.vec(), scalar(2), true);
		return res;
	}
};

namespace internal {
/**
 * @brief 为 SO2 选择关闭对齐的存储选项。
 */
template<class Scalar, int Options>
struct UnalignedType<SO2<Scalar, Options > >{
	typedef SO2<Scalar, Options | Eigen::DontAlign> type;
};

/**
 * @brief 为 SO3 选择关闭对齐的存储选项。
 */
template<class Scalar, int Options>
struct UnalignedType<SO3<Scalar, Options > >{
	typedef SO3<Scalar, Options | Eigen::DontAlign> type;
};

}  // namespace internal


}  // namespace MTK

#endif /*SON_H_*/

