// This is a NEW implementation of the algorithm described in the
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
 * @file mtk/types/S2.hpp
 * @brief Unit vectors on the sphere, or directions in 3D.
 */
/**
 * @file S2.hpp
 * @brief 固定模长三维方向的 S2 流形，使用二维局部误差和三维旋转过程增量。对跖点处的方向差不唯一。
 * @author Antinomy
 * @date 2026-10-05
 */
#ifndef S2_H_
#define S2_H_


#include "vect.hpp"

#include "SOn.hpp"
#include "../src/mtkmath.hpp"




namespace MTK {

/**
 * Manifold representation of @f$ S^2 @f$. 
 * Used for unit vectors on the sphere or directions in 3D.
 * 
 * @todo add conversions from/to polar angles?
 */
/**
 * @brief 表示半径 den/num 的球面方向，DOF=2、DIM=3。
 * @note S2_typ 选择局部坐标基参考轴（1=x、2=y、3=z）；默认/输入向量均按固定半径构造，外部直接修改公开 vec 可破坏模长约束。
 */
template<class _scalar = double, int den = 1, int num = 1, int S2_typ = 3>
struct S2 {
	
	typedef _scalar scalar;
	typedef vect<3, scalar> vect_type; 
	typedef SO3<scalar> SO3_type;
	typedef typename vect_type::base vec3; 
	scalar length = scalar(den)/scalar(num);
	enum {DOF=2, TYP = 1, DIM = 3};
	
//private:
	/**
	 * Unit vector on the sphere, or vector pointing in a direction
	 */
	vect_type vec; 
	
public:
	/**
	 * @brief 构造固定半径球面方向。
	 * @note 默认沿 S2_typ 对应正轴；输入方向应非零，num 应非零且半径应为正。
	 */
	S2() {
		if(S2_typ == 3) vec=length * vec3(0, 0, std::sqrt(1));
		if(S2_typ == 2) vec=length * vec3(0, std::sqrt(1), 0);
		if(S2_typ == 1) vec=length * vec3(std::sqrt(1), 0, 0);
	} 
	/**
	 * @brief 构造固定半径球面方向。
	 * @param x 初始方向 x 分量。
	 * @param y 初始方向 y 分量。
	 * @param z 初始方向 z 分量。
	 * @note 默认沿 S2_typ 对应正轴；输入方向应非零，num 应非零且半径应为正。
	 */
	S2(const scalar &x, const scalar &y, const scalar &z) : vec(vec3(x, y, z)) { 
		vec.normalize();
		vec = vec * length;
	}
	
	/**
	 * @brief 构造固定半径球面方向。
	 * @param _vec 待归一化的三维方向。
	 * @note 默认沿 S2_typ 对应正轴；输入方向应非零，num 应非零且半径应为正。
	 */
	S2(const vect_type &_vec) : vec(_vec) {
		vec.normalize();
		vec = vec * length;
	}

	/**
	 * @brief 用三维旋转增量更新方向。
	 * @param delta 三维轴角增量（rad）。
	 * @param scale 旋转增量的缩放系数。
	 * @note 通过旋转作用在 vec 上，维持模长；与二维 boxplus 的输入空间不同。
	 */
	void oplus(MTK::vectview<const scalar, 3> delta, scalar scale = 1)
	{
		SO3_type res;
		res.w() = MTK::exp<scalar, 3>(res.vec(), delta, scalar(scale/2));
		vec = res.toRotationMatrix() * vec;
	}
	
	/**
	 * @brief 将二维局部误差通过 Bx 提升为三维旋转后更新方向。
	 * @param delta 二维切空间角度增量（rad）。
	 * @param scale 增量缩放系数。
	 * @note Bx 必须在当前方向计算；使用其他方向的基会改变误差坐标含义。
	 */
	void boxplus(MTK::vectview<const scalar, 2> delta, scalar scale=1) {
		Eigen::Matrix<scalar, 3, 2> Bx;
		S2_Bx(Bx);
		vect_type Bu = Bx*delta;SO3_type res;
		res.w() = MTK::exp<scalar, 3>(res.vec(), Bu, scalar(scale/2));
		vec = res.toRotationMatrix() * vec;
	} 
	
	/**
	 * @brief 计算当前方向相对 other 的二维局部误差。
	 * @param res 输出二维方向差（rad）。
	 * @param other 作为局部坐标参考的方向，半径须一致。
	 * @note 同向返回零；对跖点的轴不唯一，当前固定返回 [3.1415926,0]。
	 */
	void boxminus(MTK::vectview<scalar, 2> res, const S2<scalar, den, num, S2_typ>& other) const {
		/* atan2(|vec×other|,vec·other) 避免 acos 对模长的额外归一化；对跖点轴不唯一，必须单独处理。 */
		scalar v_sin = (MTK::hat(vec)*other.vec).norm();
		scalar v_cos = vec.transpose() * other.vec;
		scalar theta = std::atan2(v_sin, v_cos);
		if(v_sin < MTK::tolerance<scalar>())
		{
			if(std::fabs(theta) > MTK::tolerance<scalar>() )
			{
				res[0] = 3.1415926;
				res[1] = 0;
			}
			else{
				res[0] = 0;
				res[1] = 0;
			}
		}
		else
		{
			S2<scalar, den, num, S2_typ> other_copy = other;
			Eigen::Matrix<scalar, 3, 2>Bx;
			other_copy.S2_Bx(Bx);
			res = theta/v_sin * Bx.transpose() * MTK::hat(other.vec)*vec;
		}
	}
	
	/**
	 * @brief 取得当前方向的叉乘矩阵。
	 * @param res 写入 3×3 反对称矩阵。
	 */
	void S2_hat(Eigen::Matrix<scalar, 3, 3> &res)
	{
		Eigen::Matrix<scalar, 3, 3> skew_vec;
		skew_vec << scalar(0), -vec[2], vec[1],
								vec[2], scalar(0), -vec[0],
								-vec[1], vec[0], scalar(0);
		res = skew_vec;
	}


	/**
	 * @brief 构造由二维误差到三维旋转轴的局部基。
	 * @param res 输出 3×2 基矩阵。
	 * @note 公式依赖 S2_typ 与 length；参考轴负极附近使用固定矩阵分支，修改此约定会同时影响 boxplus/boxminus 和滤波雅可比。
	 */
	void S2_Bx(Eigen::Matrix<scalar, 3, 2> &res)
	{
		if(S2_typ == 3)
		{
		if(vec[2] + length > tolerance<scalar>())
		{
			
			res << length - vec[0]*vec[0]/(length+vec[2]), -vec[0]*vec[1]/(length+vec[2]),
					 -vec[0]*vec[1]/(length+vec[2]), length-vec[1]*vec[1]/(length+vec[2]),
					 -vec[0], -vec[1];
			res /= length;
		}
		else
		{
			res = Eigen::Matrix<scalar, 3, 2>::Zero();
			res(1, 1) = -1;
			res(2, 0) = 1;
		}
		}
		else if(S2_typ == 2)
		{
		if(vec[1] + length > tolerance<scalar>())
		{
			
			res << length - vec[0]*vec[0]/(length+vec[1]), -vec[0]*vec[2]/(length+vec[1]),
					 -vec[0], -vec[2],
					 -vec[0]*vec[2]/(length+vec[1]), length-vec[2]*vec[2]/(length+vec[1]);
			res /= length;
		}
		else
		{
			res = Eigen::Matrix<scalar, 3, 2>::Zero();
			res(1, 1) = -1;
			res(2, 0) = 1;
		}
		}
		else
		{
		if(vec[0] + length > tolerance<scalar>())
		{
			
			res << -vec[1], -vec[2],
					 length - vec[1]*vec[1]/(length+vec[0]), -vec[2]*vec[1]/(length+vec[0]),
					 -vec[2]*vec[1]/(length+vec[0]), length-vec[2]*vec[2]/(length+vec[0]);
			res /= length;
		}
		else
		{
			res = Eigen::Matrix<scalar, 3, 2>::Zero();
			res(1, 1) = -1;
			res(2, 0) = 1;
		}
		}
	}

	/**
	 * @brief 计算球面差分的 2×3 微分矩阵。
	 * @param res 输出球面差分微分。
	 * @param subtrahend 参与差分的另一个方向。
	 * @note 相同方向使用连续极限；对跖点无唯一微分，当前打印错误并 exit(100)，不抛出 C++ 异常。
	 */
	void S2_Nx(Eigen::Matrix<scalar, 2, 3> &res, S2<scalar, den, num, S2_typ>& subtrahend)
	{
		if((vec+subtrahend.vec).norm() > tolerance<scalar>())
		{
			Eigen::Matrix<scalar, 3, 2> Bx;
			S2_Bx(Bx);
			if((vec-subtrahend.vec).norm() > tolerance<scalar>())
			{
				/* atan2(|vec×other|,vec·other) 避免 acos 对模长的额外归一化；对跖点轴不唯一，必须单独处理。 */
				scalar v_sin = (MTK::hat(vec)*subtrahend.vec).norm();
				scalar v_cos = vec.transpose() * subtrahend.vec;
				
				res = Bx.transpose() * (std::atan2(v_sin, v_cos)/v_sin*MTK::hat(vec)+MTK::hat(vec)*subtrahend.vec*((-v_cos/v_sin/v_sin/length/length/length/length+std::atan2(v_sin, v_cos)/v_sin/v_sin/v_sin)*subtrahend.vec.transpose()*MTK::hat(vec)*MTK::hat(vec)-vec.transpose()/length/length/length/length));
			}
			else
			{
				res = 1/length/length*Bx.transpose()*MTK::hat(vec);
			}
		}
		else
		{
			std::cerr << "No N(x, y) for x=-y" << std::endl;
			std::exit(100);
		}
	}

	/**
	 * @brief 计算同向点处的球面差分微分。
	 * @param res 输出 Bxᵀ·hat(vec)/length²（2×3）。
	 */
	void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3> &res)
	{
		Eigen::Matrix<scalar, 3, 2> Bx;
		S2_Bx(Bx);
		res = 1/length/length*Bx.transpose()*MTK::hat(vec);
	}

	/**
	 * @brief 计算二维 boxplus 对方向向量的微分。
	 * @param res 输出 3×2 雅可比。
	 * @param delta 二维局部扰动（rad）。
	 * @note 零增量使用 -hat(vec)·Bx；非零分支的 scalar(1/2) 先进行整数除法得到 0，当前不能视为有效半角系数。
	 */
	void S2_Mx(Eigen::Matrix<scalar, 3, 2> &res, MTK::vectview<const scalar, 2> delta)
	{
		Eigen::Matrix<scalar, 3, 2> Bx;
		S2_Bx(Bx);
		if(delta.norm() < tolerance<scalar>())
		{
			res = -MTK::hat(vec)*Bx;
		}
		else{
			vect_type Bu = Bx*delta;
			SO3_type exp_delta;
			/* 当前 scalar(1/2) 的 1/2 是整数除法，结果为 0；这里记录该行为，未改动计算。 */
			exp_delta.w() = MTK::exp<scalar, 3>(exp_delta.vec(), Bu, scalar(1/2));
			res = -exp_delta.toRotationMatrix()*MTK::hat(vec)*MTK::A_matrix(Bu).transpose()*Bx;
		}
	}

	/**
	 * @brief 借用方向向量的只读引用。
	 * @return 当前 vec 的引用，生命周期由 S2 对象决定。
	 */
	operator const vect_type&() const{
		return vec;
	}
	
	/**
	 * @brief 取得方向向量的只读引用。
	 * @return 当前 vec；调用不复制也不归一化。
	 */
	const vect_type& get_vect() const {
		return vec;
	}
	
	/**
	 * @brief 将三维旋转作用于球面方向。
	 * @param rot 有效单位四元数旋转。
	 * @param dir 待旋转球面方向。
	 * @return 保留同一半径约定的新方向。
	 */
	friend S2<scalar, den, num, S2_typ> operator*(const SO3<scalar>& rot, const S2<scalar, den, num, S2_typ>& dir)
	{
		S2<scalar, den, num, S2_typ> ret;
		ret.vec = rot * dir.vec;
		return ret;
	}
	
	/**
	 * @brief 读取方向的指定坐标分量。
	 * @param idx 坐标索引，须为 0、1 或 2。
	 * @return 对应 x、y 或 z 分量，单位与 length 一致。
	 */
	scalar operator[](int idx) const {return vec[idx]; }
	
	/**
	 * @brief 输出方向向量的三个分量。
	 * @param os 输出流。
	 * @param vec 待输出方向。
	 * @return 原输出流引用。
	 */
	friend std::ostream& operator<<(std::ostream &os, const S2<scalar, den, num, S2_typ>& vec){
		return os << vec.vec.transpose() << " ";
	}
	/**
	 * @brief 读取三个分量并归一到固定半径。
	 * @param is 输入流。
	 * @param vec 写入的方向对象。
	 * @return 原输入流引用。
	 * @note 输入须能形成非零方向；未另行校验流失败或零向量。
	 */
	friend std::istream& operator>>(std::istream &is, S2<scalar, den, num, S2_typ>& vec){
		for(int i=0; i<3; ++i)
			is >> vec.vec[i];
		vec.vec.normalize();
		vec.vec = vec.vec * vec.length;
		return is;
	
	}
};


}  // namespace MTK


#endif /*S2_H_*/
