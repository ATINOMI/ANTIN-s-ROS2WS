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
 * @file mtk/types/vect.hpp
 * @brief Basic vectors interpreted as manifolds.
 * 
 * This file also implements a simple wrapper for matrices, for arbitrary scalars
 * and for positive scalars.
 */
/**
 * @file vect.hpp
 * @brief 向量、矩阵、标量、正标量和复数的流形适配。普通数值采用加法误差，正标量采用对数误差。
 * @author Antinomy
 * @date 2026-10-05
 */
#ifndef VECT_H_
#define VECT_H_

#include <iosfwd>
#include <iostream>
#include <vector>

#include "../src/vectview.hpp"

namespace MTK {

static const Eigen::IOFormat IO_no_spaces(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", ",", "", "", "[", "]"); 


/**
 * A simple vector class.
 * Implementation is basically a wrapper around Eigen::Matrix with manifold 
 * requirements added.
 */
/**
 * @brief 将 Eigen 向量适配为 DOF=DIM=D 的欧氏流形。
 */
template<int D = 3, class _scalar = double, int _Options=Eigen::AutoAlign>
struct vect : public Eigen::Matrix<_scalar, D, 1, _Options> {
	typedef Eigen::Matrix<_scalar, D, 1, _Options> base;
	enum {DOF = D, DIM = D, TYP = 0};
	typedef _scalar scalar;
	
	//using base::operator=;
	
	/** Standard constructor. Sets all values to zero. */
	/**
	 * @brief 构造欧氏向量；默认元素为零。
	 * @param src 源向量或连续数据指针。
	 */
	vect(const base &src = base::Zero()) : base(src) {}

	/** Constructor copying the value of the expression \a other */
	/**
	 * @brief 构造欧氏向量；默认元素为零。
	 * @param other 尺寸兼容的 Eigen 表达式。
	 */
	template<typename OtherDerived>
	EIGEN_STRONG_INLINE vect(const Eigen::DenseBase<OtherDerived>& other) : base(other) {}
	
	/** Construct from memory. */
	/**
	 * @brief 构造欧氏向量；默认元素为零。
	 * @param src 源向量或连续数据指针。
	 * @param size 从指针映射的元素数，应匹配 D。
	 */
	vect(const scalar* src, int size = DOF) : base(base::Map(src, size)) { }

	/**
	 * @brief 用局部误差更新当前流形值。
	 * @param vec 局部误差：普通数值按加法，PositiveScalar 按对数，SO2/SO3 按弧度。
	 * @param scale 误差缩放系数。
	 * @note vect/matrix/Scalar/Complex 做加法；PositiveScalar 乘 exp(scale·vec[0])。
	 */
	void boxplus(MTK::vectview<const scalar, D> vec, scalar scale=1) {
		*this += scale * vec;
	}
	/**
	 * @brief 计算当前值相对 other 的局部误差。
	 * @param res 写入差分结果，不分配额外输出存储。
	 * @param other 局部误差参考值。
	 * @note 普通值使用 this-other；PositiveScalar 使用 log(this/other)，要求两者为正。
	 */
	void boxminus(MTK::vectview<scalar, D> res, const vect<D, scalar>& other) const {
		res = *this - other;
	}

	/**
	 * @brief 按过程增量更新当前流形值。
	 * @param vec 过程增量，具体加法、对数或旋转含义与当前类型一致。
	 * @param scale 过程增量缩放量。
	 * @note 此文件各类型的 oplus 与其 boxplus 使用相同更新公式。
	 */
	void oplus(MTK::vectview<const scalar, D> vec, scalar scale=1) {
		*this += scale * vec;
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
	 * @brief 按底层顺序输出欧氏向量或矩阵元素。
	 * @param os 输出流。
	 * @param v 待输出向量。
	 * @return 原输出流引用；每个元素后跟空格。
	 */
	friend std::ostream& operator<<(std::ostream &os, const vect<D, scalar, _Options>& v){
		// Eigen sometimes messes with the streams flags, so output manually:
		for(int i=0; i<DOF; ++i)
			os << v(i) << " ";
		return os;
	}
	/**
	 * @brief 从流中读取数值并更新对象。
	 * @param is 输入流。
	 * @param v 目标向量；动态长度要求括号，定长可无括号。
	 * @return 原输入流引用；向量可跳过逗号，括号不配对会置 badbit。
	 */
	friend std::istream& operator>>(std::istream &is, vect<D, scalar, _Options>& v){
		char term=0;
		is >> std::ws; // skip whitespace
		switch(is.peek()) {
		case '(': term=')'; is.ignore(1); break;
		case '[': term=']'; is.ignore(1); break;
		case '{': term='}'; is.ignore(1); break;
		default: break;
		}
		/* 动态向量依赖配对括号识别长度；定长向量读取当前长度，不改变尺寸。括号错误反映在流状态。 */
		if(D==Eigen::Dynamic) { 
			assert(term !=0 && "Dynamic vectors must be embraced");
			std::vector<scalar> temp;
			while(is.good() && is.peek() != term) {
				scalar x;
				is >> x;
				temp.push_back(x);
				if(is.peek()==',') is.ignore(1);
			}
			v = vect::Map(temp.data(), temp.size());
		} else
			for(int i=0; i<v.size(); ++i){
				is >> v[i];
				if(is.peek()==',') { // ignore commas between values
					is.ignore(1);
				}
			}
		if(term!=0) {
			char x;
			is >> x;
			if(x!=term) {
				is.setstate(is.badbit);
//				assert(x==term && "start and end bracket do not match!");
			}
		}
		return is;
	}
	
	/**
	 * @brief 借用向量尾部 dim 个元素。
	 * @return 可写或只读子视图，取决于当前对象 const 性。
	 * @note 编译期要求 0<dim<=DOF；子视图共享原向量存储。
	 */
	template<int dim>
	vectview<scalar, dim> tail(){
		BOOST_STATIC_ASSERT(0< dim && dim <= DOF);
		return base::template tail<dim>();
	}
	/**
	 * @brief 借用向量尾部 dim 个元素。
	 * @return 可写或只读子视图，取决于当前对象 const 性。
	 * @note 编译期要求 0<dim<=DOF；子视图共享原向量存储。
	 */
	template<int dim>
	vectview<const scalar, dim> tail() const{
		BOOST_STATIC_ASSERT(0< dim && dim <= DOF);
		return base::template tail<dim>();
	}
	/**
	 * @brief 借用向量头部 dim 个元素。
	 * @return 可写或只读子视图，取决于当前对象 const 性。
	 * @note 编译期要求 0<dim<=DOF；子视图共享原向量存储。
	 */
	template<int dim>
	vectview<scalar, dim> head(){
		BOOST_STATIC_ASSERT(0< dim && dim <= DOF);
		return base::template head<dim>();
	}
	/**
	 * @brief 借用向量头部 dim 个元素。
	 * @return 可写或只读子视图，取决于当前对象 const 性。
	 * @note 编译期要求 0<dim<=DOF；子视图共享原向量存储。
	 */
	template<int dim>
	vectview<const scalar, dim> head() const{
		BOOST_STATIC_ASSERT(0< dim && dim <= DOF);
		return base::template head<dim>();
	}
};


/**
 * A simple matrix class.
 * Implementation is basically a wrapper around Eigen::Matrix with manifold 
 * requirements added, i.e., matrix is viewed as a plain vector for that.
 */
/**
 * @brief 将矩阵按底层存储顺序视为 M·N 维欧氏误差；此类型 DIM=0，不能直接视为支持同维过程积分。
 */
template<int M, int N, class _scalar = double, int _Options = Eigen::Matrix<_scalar, M, N>::Options>
struct matrix : public Eigen::Matrix<_scalar, M, N, _Options> {
	typedef Eigen::Matrix<_scalar, M, N, _Options> base; 
	enum {DOF = M * N, TYP = 4, DIM=0};
	typedef _scalar scalar;
	
	using base::operator=; 
	
	/** Standard constructor. Sets all values to zero. */
	/**
	 * @brief 构造欧氏矩阵；无参构造将所有元素置零。
	 */
	matrix() {
		base::setZero();
	}
	
	/** Constructor copying the value of the expression \a other */
	/**
	 * @brief 构造欧氏矩阵；无参构造将所有元素置零。
	 * @param other 同尺寸 Eigen 矩阵表达式。
	 */
	template<typename OtherDerived>
	EIGEN_STRONG_INLINE matrix(const Eigen::MatrixBase<OtherDerived>& other) : base(other) {}
	
	/** Construct from memory. */
	/**
	 * @brief 构造欧氏矩阵；无参构造将所有元素置零。
	 * @param src 连续矩阵数据指针，存储顺序应与模板一致。
	 */
	matrix(const scalar* src) : base(src) { } 
	
	/**
	 * @brief 用局部误差更新当前流形值。
	 * @param vec 局部误差：普通数值按加法，PositiveScalar 按对数，SO2/SO3 按弧度。
	 * @param scale 误差缩放系数。
	 * @note vect/matrix/Scalar/Complex 做加法；PositiveScalar 乘 exp(scale·vec[0])。
	 */
	void boxplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1) {
		*this += scale * base::Map(vec.data());
	}
	/**
	 * @brief 计算当前值相对 other 的局部误差。
	 * @param res 写入差分结果，不分配额外输出存储。
	 * @param other 局部误差参考值。
	 * @note 普通值使用 this-other；PositiveScalar 使用 log(this/other)，要求两者为正。
	 */
	void boxminus(MTK::vectview<scalar, DOF> res, const matrix& other) const {
		base::Map(res.data()) = *this - other;
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
	 * @brief 按过程增量更新当前流形值。
	 * @param vec 过程增量，具体加法、对数或旋转含义与当前类型一致。
	 * @param scale 过程增量缩放量。
	 * @note 此文件各类型的 oplus 与其 boxplus 使用相同更新公式。
	 */
	void oplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1) {
		*this += scale * base::Map(vec.data());
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
	 * @brief 按底层顺序输出欧氏向量或矩阵元素。
	 * @param os 输出流。
	 * @param mat 待输出矩阵。
	 * @return 原输出流引用；每个元素后跟空格。
	 */
	friend std::ostream& operator<<(std::ostream &os, const matrix<M, N, scalar, _Options>& mat){
		for(int i=0; i<DOF; ++i){
			os << mat.data()[i] << " ";
		}
		return os;
	}
	/**
	 * @brief 从流中读取数值并更新对象。
	 * @param is 输入流。
	 * @param mat 按底层存储顺序写入的矩阵。
	 * @return 原输入流引用；向量可跳过逗号，括号不配对会置 badbit。
	 */
	friend std::istream& operator>>(std::istream &is, matrix<M, N, scalar, _Options>& mat){
		for(int i=0; i<DOF; ++i){
			is >> mat.data()[i];
		}
		return is;
	}
};// @todo What if M / N = Eigen::Dynamic?



/**
 * A simple scalar type.
 */
/**
 * @brief 一维可写普通标量流形，误差用加法。
 */
template<class _scalar = double>
struct Scalar {
	enum {DOF = 1, TYP = 5, DIM=0};
	typedef _scalar scalar;
	
	scalar value;
	
	/**
	 * @brief 构造普通标量流形。
	 * @param value 初值，默认为零。
	 */
	Scalar(const scalar& value = scalar(0)) : value(value) {}
	/**
	 * @brief 取得内部标量的只读引用。
	 * @return value 的引用，生命周期与当前对象一致。
	 */
	operator const scalar&() const { return value; }
	/**
	 * @brief 取得普通标量的可写引用。
	 * @return 可直接修改 value 的引用。
	 */
	operator scalar&() { return value; }
	/**
	 * @brief 更新标量并返回当前对象。
	 * @param val 新值；PositiveScalar 通过 assert 要求严格大于零。
	 * @return 当前对象引用。
	 */
	Scalar& operator=(const scalar& val) { value = val; return *this; }

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
	 * @brief 按过程增量更新当前流形值。
	 * @param vec 过程增量，具体加法、对数或旋转含义与当前类型一致。
	 * @param scale 过程增量缩放量。
	 * @note 此文件各类型的 oplus 与其 boxplus 使用相同更新公式。
	 */
	void oplus(MTK::vectview<const scalar, DOF> vec, scalar scale=1) {
		value += scale * vec[0];
	}

	/**
	 * @brief 用局部误差更新当前流形值。
	 * @param vec 局部误差：普通数值按加法，PositiveScalar 按对数，SO2/SO3 按弧度。
	 * @param scale 误差缩放系数。
	 * @note vect/matrix/Scalar/Complex 做加法；PositiveScalar 乘 exp(scale·vec[0])。
	 */
	void boxplus(MTK::vectview<const scalar, DOF> vec, scalar scale=1) {
		value += scale * vec[0];
	}
	/**
	 * @brief 计算当前值相对 other 的局部误差。
	 * @param res 写入差分结果，不分配额外输出存储。
	 * @param other 局部误差参考值。
	 * @note 普通值使用 this-other；PositiveScalar 使用 log(this/other)，要求两者为正。
	 */
	void boxminus(MTK::vectview<scalar, DOF> res, const Scalar& other) const {
		res[0] = *this - other;
	}
};

/**
 * Positive scalars.
 * Boxplus is implemented using multiplication by @f$x\boxplus\delta = x\cdot\exp(\delta) @f$.
 */
/**
 * @brief 正数的一维对数误差流形，乘 exp(delta) 更新以维持正性。
 */
template<class _scalar = double>
struct PositiveScalar {
	enum {DOF = 1, TYP = 6, DIM=0};
	typedef _scalar scalar;
	
	scalar value;
	
	/**
	 * @brief 构造严格正的标量流形。
	 * @param value 初值，默认为 1；value>0 由 assert 检查。
	 */
	PositiveScalar(const scalar& value = scalar(1)) : value(value) {
		assert(value > scalar(0));
	}
	/**
	 * @brief 取得内部标量的只读引用。
	 * @return value 的引用，生命周期与当前对象一致。
	 */
	operator const scalar&() const { return value; }
	/**
	 * @brief 更新标量并返回当前对象。
	 * @param val 新值；PositiveScalar 通过 assert 要求严格大于零。
	 * @return 当前对象引用。
	 */
	PositiveScalar& operator=(const scalar& val) { assert(val>0); value = val; return *this; }

	/**
	 * @brief 用局部误差更新当前流形值。
	 * @param vec 局部误差：普通数值按加法，PositiveScalar 按对数，SO2/SO3 按弧度。
	 * @param scale 误差缩放系数。
	 * @note vect/matrix/Scalar/Complex 做加法；PositiveScalar 乘 exp(scale·vec[0])。
	 */
	void boxplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1) {
		value *= std::exp(scale * vec[0]);
	}
	/**
	 * @brief 计算当前值相对 other 的局部误差。
	 * @param res 写入差分结果，不分配额外输出存储。
	 * @param other 局部误差参考值。
	 * @note 普通值使用 this-other；PositiveScalar 使用 log(this/other)，要求两者为正。
	 */
	void boxminus(MTK::vectview<scalar, DOF> res, const PositiveScalar& other) const {
		res[0] = std::log(*this / other);
	}

	/**
	 * @brief 按过程增量更新当前流形值。
	 * @param vec 过程增量，具体加法、对数或旋转含义与当前类型一致。
	 * @param scale 过程增量缩放量。
	 * @note 此文件各类型的 oplus 与其 boxplus 使用相同更新公式。
	 */
	void oplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1) {
		value *= std::exp(scale * vec[0]);
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
	 * @brief 从流中读取数值并更新对象。
	 * @param is 输入流。
	 * @param s 目标正标量，读取后 assert 检查 s.value>0。
	 * @return 原输入流引用；向量可跳过逗号，括号不配对会置 badbit。
	 */
	friend std::istream& operator>>(std::istream &is, PositiveScalar<scalar>& s){
		is >> s.value;
		assert(s.value > 0);
		return is;
	}
};

/**
 * @brief 以实部、虚部为两维加法误差的复数流形。
 */
template<class _scalar = double>
struct Complex : public std::complex<_scalar>{
	enum {DOF = 2, TYP = 7, DIM=0};
	typedef _scalar scalar;
	
	typedef std::complex<scalar> Base;
	
	/**
	 * @brief 按实部和虚部构造复数流形。
	 * @param value 源复数值。
	 */
	Complex(const Base& value) : Base(value) {}
	/**
	 * @brief 按实部和虚部构造复数流形。
	 * @param re 实部，默认 0。
	 * @param im 虚部，默认 0。
	 */
	Complex(const scalar& re = 0.0, const scalar& im = 0.0) : Base(re, im) {}
	/**
	 * @brief 按实部和虚部构造复数流形。
	 * @param in 前两元素分别为实部和虚部的视图或表达式。
	 */
	Complex(const MTK::vectview<const scalar, 2> &in) : Base(in[0], in[1]) {}
	/**
	 * @brief 按实部和虚部构造复数流形。
	 * @param in 前两元素分别为实部和虚部的视图或表达式。
	 */
	template<class Derived>
	Complex(const Eigen::DenseBase<Derived> &in) : Base(in[0], in[1]) {}
	
	/**
	 * @brief 用局部误差更新当前流形值。
	 * @param vec 局部误差：普通数值按加法，PositiveScalar 按对数，SO2/SO3 按弧度。
	 * @param scale 误差缩放系数。
	 * @note vect/matrix/Scalar/Complex 做加法；PositiveScalar 乘 exp(scale·vec[0])。
	 */
	void boxplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1) {
		Base::real() += scale * vec[0];
		Base::imag() += scale * vec[1];
	};
	/**
	 * @brief 计算当前值相对 other 的局部误差。
	 * @param res 写入差分结果，不分配额外输出存储。
	 * @param other 局部误差参考值。
	 * @note 普通值使用 this-other；PositiveScalar 使用 log(this/other)，要求两者为正。
	 */
	void boxminus(MTK::vectview<scalar, DOF> res, const Complex& other) const {
		Complex diff = *this - other;
		res << diff.real(), diff.imag();
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
	 * @brief 按过程增量更新当前流形值。
	 * @param vec 过程增量，具体加法、对数或旋转含义与当前类型一致。
	 * @param scale 过程增量缩放量。
	 * @note 此文件各类型的 oplus 与其 boxplus 使用相同更新公式。
	 */
	void oplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1) {
		Base::real() += scale * vec[0];
		Base::imag() += scale * vec[1];
	};

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
	 * @brief 计算复数的模平方。
	 * @return real²+imag²。
	 */
	scalar squaredNorm() const {
		return std::pow(Base::real(),2) + std::pow(Base::imag(),2);
	}
	
	/**
	 * @brief 按索引访问复数的实部或虚部。
	 * @param i 0 为实部，1 为虚部；用 assert 检查范围。
	 * @return 对应分量的引用，const 重载为只读引用。
	 */
	const scalar& operator()(int i) const {
		assert(0<=i && i<2 && "Index out of range");
		return i==0 ? Base::real() : Base::imag();
	}
	/**
	 * @brief 按索引访问复数的实部或虚部。
	 * @param i 0 为实部，1 为虚部；用 assert 检查范围。
	 * @return 对应分量的引用，const 重载为只读引用。
	 */
	scalar& operator()(int i){
		assert(0<=i && i<2 && "Index out of range");
		return i==0 ? Base::real() : Base::imag();
	}
};


namespace internal {

/**
 * @brief 选择关闭 Eigen 对齐的向量存储类型。
 */
template<int dim, class Scalar, int Options>
struct UnalignedType<vect<dim, Scalar, Options > >{
	typedef vect<dim, Scalar, Options | Eigen::DontAlign> type;
};

}  // namespace internal


}  // namespace MTK




#endif /*VECT_H_*/
