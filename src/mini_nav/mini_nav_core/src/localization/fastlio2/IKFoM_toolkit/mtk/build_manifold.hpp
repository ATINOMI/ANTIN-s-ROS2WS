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
 * @file mtk/build_manifold.hpp
 * @brief Macro to automatically construct compound manifolds.
 * 
 */
/**
 * @file build_manifold.hpp
 * @brief 用预处理宏生成组合流形及分量扰动、差分和索引表接口。必须分别累加 DOF 与 DIM，S2 的两者不同。
 * @author Antinomy
 * @date 2026-10-05
 */
#ifndef MTK_AUTOCONSTRUCT_HPP_
#define MTK_AUTOCONSTRUCT_HPP_

#include <vector>

#include <boost/preprocessor/seq.hpp>
#include <boost/preprocessor/cat.hpp>
#include <Eigen/Core>

#include "src/SubManifold.hpp"
#include "startIdx.hpp"

#ifndef PARSED_BY_DOXYGEN
//////// internals //////

#define MTK_APPLY_MACRO_ON_TUPLE(r, macro, tuple) macro tuple

#define MTK_TRANSFORM_COMMA(macro, entries) BOOST_PP_SEQ_ENUM(BOOST_PP_SEQ_TRANSFORM_S(1, MTK_APPLY_MACRO_ON_TUPLE, macro, entries))

#define MTK_TRANSFORM(macro, entries) BOOST_PP_SEQ_FOR_EACH_R(1, MTK_APPLY_MACRO_ON_TUPLE, macro, entries)

#define MTK_CONSTRUCTOR_ARG(  type, id) const type& id = type()
#define MTK_CONSTRUCTOR_COPY( type, id) id(id)
#define MTK_BOXPLUS(          type, id) id.boxplus(MTK::subvector(__vec, &self::id), __scale);
#define MTK_OPLUS(               type, id) id.oplus(MTK::subvector_(__vec, &self::id), __scale);
#define MTK_BOXMINUS(         type, id) id.boxminus(MTK::subvector(__res, &self::id), __oth.id);
#define MTK_S2_hat(          type, id) if(id.IDX == idx){id.S2_hat(res);}
#define MTK_S2_Nx_yy(            type, id) if(id.IDX == idx){id.S2_Nx_yy(res);}
#define MTK_S2_Mx(          type, id) if(id.IDX == idx){id.S2_Mx(res, dx);}
#define MTK_OSTREAM(          type, id) << __var.id << " "
#define MTK_ISTREAM(          type, id) >> __var.id
#define MTK_S2_state(         type, id) if(id.TYP == 1){S2_state.push_back(std::make_pair(id.IDX, id.DIM));}
#define MTK_SO3_state(        type, id) if(id.TYP == 2){(SO3_state).push_back(std::make_pair(id.IDX, id.DIM));}
#define MTK_vect_state(        type, id) if(id.TYP == 0){(vect_state).push_back(std::make_pair(std::make_pair(id.IDX, id.DIM), type::DOF));}

/**
 * @brief 递归生成子流形成员与总维数。
 * @param seq 双括号类型/成员序列。
 * @param S2state 传递的球面索引表标识。
 * @param SO3state 传递的旋转索引表标识。
 * @note 每个成员分别累计 DOF 和 DIM，末项生成总维数枚举及 scalar 类型。
 */
#define MTK_SUBVARLIST(seq, S2state, SO3state) \
BOOST_PP_FOR_1( \
		( \
				BOOST_PP_SEQ_SIZE(seq), \
				BOOST_PP_SEQ_HEAD(seq), \
				BOOST_PP_SEQ_TAIL(seq) (~), \
				0,\
				0,\
				S2state,\
				SO3state ),\
		MTK_ENTRIES_TEST, MTK_ENTRIES_NEXT, MTK_ENTRIES_OUTPUT)

#define MTK_PUT_TYPE(type, id, dof, dim, S2state, SO3state) \
	MTK::SubManifold<type, dof, dim> id; 
#define MTK_PUT_TYPE_AND_ENUM(type, id, dof, dim, S2state, SO3state) \
	MTK_PUT_TYPE(type, id, dof, dim, S2state, SO3state) \
	enum {DOF = type::DOF + dof}; \
	enum {DIM = type::DIM+dim}; \
	typedef type::scalar scalar; 

#define MTK_ENTRIES_OUTPUT(r, state) MTK_ENTRIES_OUTPUT_I state
#define MTK_ENTRIES_OUTPUT_I(s, head, seq, dof, dim, S2state, SO3state) \
	MTK_APPLY_MACRO_ON_TUPLE(~, \
		BOOST_PP_IF(BOOST_PP_DEC(s), MTK_PUT_TYPE, MTK_PUT_TYPE_AND_ENUM), \
		( BOOST_PP_TUPLE_REM_2 head, dof, dim, S2state, SO3state)) 

#define MTK_ENTRIES_TEST(r, state) MTK_TUPLE_ELEM_4_0 state

//! this used to be BOOST_PP_TUPLE_ELEM_4_0:
#define MTK_TUPLE_ELEM_4_0(a,b,c,d,e,f, g) a

#define MTK_ENTRIES_NEXT(r, state) MTK_ENTRIES_NEXT_I state
#define MTK_ENTRIES_NEXT_I(len, head, seq, dof, dim, S2state, SO3state) ( \
		BOOST_PP_DEC(len), \
		BOOST_PP_SEQ_HEAD(seq), \
		BOOST_PP_SEQ_TAIL(seq), \
		dof + BOOST_PP_TUPLE_ELEM_2_0 head::DOF,\
		dim + BOOST_PP_TUPLE_ELEM_2_0 head::DIM,\
		S2state,\
		SO3state)

#endif /* not PARSED_BY_DOXYGEN */


/**
 * Construct a manifold.
 * @param name is the class-name of the manifold, 
 * @param entries is the list of sub manifolds 
 * 
 * Entries must be given in a list like this:
 * @code
 * typedef MTK::trafo<MTK::SO3<double> > Pose;
 * typedef MTK::vect<double, 3> Vec3;
 * MTK_BUILD_MANIFOLD(imu_state,
 *    ((Pose, pose))
 *    ((Vec3, vel))
 *    ((Vec3, acc_bias))
 * )
 * @endcode
 * Whitespace is optional, but the double parentheses are necessary.
 * Construction is done entirely in preprocessor.
 * After construction @a name is also a manifold. Its members can be 
 * accessed by names given in @a entries.
 * 
 * @note Variable types are not allowed to have commas, thus types like
 *       @c vect<double, 3> need to be typedef'ed ahead.
 */
/**
 * @brief 生成具备扰动、差分、流操作和分类索引接口的组合状态。
 * @param name 生成的状态结构体名称。
 * @param entries 由 ((type,member)) 组成的预处理序列，带逗号的模板类型须先定义别名。
 * @note boxplus/boxminus 按 DOF 分块，oplus 按 DIM 分块；分类 build_* 方法只追加，不幂等。
 */
#define MTK_BUILD_MANIFOLD(name, entries) \
struct name { \
	typedef name self; \
	std::vector<std::pair<int, int> > S2_state;\
	std::vector<std::pair<int, int> > SO3_state;\
	std::vector<std::pair<std::pair<int, int>, int> > vect_state;\
	MTK_SUBVARLIST(entries, S2_state, SO3_state) \
	/** @brief 构造组合流形，各输入成员按 entries 顺序复制，缺省值由成员类型提供。 @note 构造参数名和类型由 entries 生成；不自动构建分类索引表。 */ \
	name ( \
		MTK_TRANSFORM_COMMA(MTK_CONSTRUCTOR_ARG, entries) \
		) : \
		MTK_TRANSFORM_COMMA(MTK_CONSTRUCTOR_COPY, entries) {}\
	/** @brief 查询组合状态局部自由度。 @return 所有成员 DOF 之和。 */ \
	int getDOF() const { return DOF; } \
	/** @brief 按 DOF 片段分别更新各子流形。 @param __vec 组合状态局部误差向量。 @param __scale 扰动缩放量。 */ \
	void boxplus(const MTK::vectview<const scalar, DOF> & __vec, scalar __scale = 1 ) { \
		MTK_TRANSFORM(MTK_BOXPLUS, entries)\
	} \
	/** @brief 按 DIM 片段分别积分各子流形。 @param __vec 组合状态过程增量向量。 @param __scale 过程增量缩放量。 */ \
	void oplus(const MTK::vectview<const scalar, DIM> & __vec, scalar __scale = 1 ) { \
		MTK_TRANSFORM(MTK_OPLUS, entries)\
	} \
	/** @brief 按成员局部坐标计算组合差分。 @param __res DOF 维差分输出。 @param __oth 作为差分参考的组合状态。 */ \
	void boxminus(MTK::vectview<scalar,DOF> __res, const name& __oth) const { \
		MTK_TRANSFORM(MTK_BOXMINUS, entries)\
	} \
	/** @brief 依 entries 顺序输出成员。 @param __os 输出流。 @param __var 待输出状态。 @return 原输出流引用。 */ \
	friend std::ostream& operator<<(std::ostream& __os, const name& __var){ \
		return __os MTK_TRANSFORM(MTK_OSTREAM, entries); \
	} \
	/** @brief 追加 S2 成员的误差索引和过程偏移。 @note 不清空原列表，重复调用会重复登记。 */ \
	void build_S2_state(){\
		MTK_TRANSFORM(MTK_S2_state, entries)\
	}\
	/** @brief 追加欧氏向量成员的索引、过程偏移和自由度。 @note 只登记 TYP=0；不清空已有列表。 */ \
	void build_vect_state(){\
		MTK_TRANSFORM(MTK_vect_state, entries)\
	}\
	/** @brief 追加 SO3 成员的误差索引和过程偏移。 @note 不清空原列表，重复调用会重复登记。 */ \
	void build_SO3_state(){\
		MTK_TRANSFORM(MTK_SO3_state, entries)\
	}\
	/** @brief 按误差索引分派球面叉乘矩阵。 @param res 3×3 输出矩阵。 @param idx 目标成员的 DOF 起始索引。 @note 索引未匹配时不写 res。 */ \
	void S2_hat(Eigen::Matrix<scalar, 3, 3> &res, int idx) {\
		MTK_TRANSFORM(MTK_S2_hat, entries)\
	}\
	/** @brief 按误差索引分派同向球面差分微分。 @param res 2×3 输出矩阵。 @param idx 目标 S2 成员的 DOF 起始索引。 @note 匹配非 S2 成员会进入其错误分支。 */ \
	void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3> &res, int idx) {\
		MTK_TRANSFORM(MTK_S2_Nx_yy, entries)\
	}\
	/** @brief 按误差索引分派球面更新微分。 @param res 3×2 输出矩阵。 @param dx 二维局部扰动。 @param idx 目标 S2 成员的 DOF 起始索引。 */ \
	void S2_Mx(Eigen::Matrix<scalar, 3, 2> &res, Eigen::Matrix<scalar, 2, 1> dx, int idx) {\
		MTK_TRANSFORM(MTK_S2_Mx, entries)\
	}\
	/** @brief 依 entries 顺序读取成员。 @param __is 输入流。 @param __var 待写入状态。 @return 原输入流引用。 */ \
	friend std::istream& operator>>(std::istream& __is, name& __var){ \
		return __is MTK_TRANSFORM(MTK_ISTREAM, entries); \
	} \
};



#endif /*MTK_AUTOCONSTRUCT_HPP_*/
