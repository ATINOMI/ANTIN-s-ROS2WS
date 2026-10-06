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
 * @file mtk/startIdx.hpp 
 * @brief Tools to access sub-elements of compound manifolds.
 */
/**
 * @file startIdx.hpp
 * @brief 通过成员指针和编译期偏移访问组合状态的向量片段与协方差块。返回视图共享原存储，不延长其生命周期。
 * @author Antinomy
 * @date 2026-10-05
 */
#ifndef GET_START_INDEX_H_
#define GET_START_INDEX_H_

#include <Eigen/Core>

#include "src/SubManifold.hpp"
#include "src/vectview.hpp"

namespace MTK {


/** 
 * \defgroup SubManifolds Accessing Submanifolds
 * For compound manifolds constructed using MTK_BUILD_MANIFOLD, member pointers
 * can be used to get sub-vectors or matrix-blocks of a corresponding big matrix.
 * E.g. for a type @a pose consisting of @a orient and @a trans the member pointers
 * @c &pose::orient and @c &pose::trans give all required information and are still
 * valid if the base type gets extended or the actual types of @a orient and @a trans
 * change (e.g. from 2D to 3D).
 * 
 * @todo Maybe require manifolds to typedef MatrixType and VectorType, etc.
 */
//@{

/**
 * Determine the index of a sub-variable within a compound variable.
 */
/**
 * @brief 查询子流形在局部误差向量中的起始偏移。
 * @return 编译期 idx。
 * @note 未命名成员指针参数用于推导子流形类型及编译期偏移，不读取对象值。
 */
template<class Base, class T, int idx, int dim> 
int getStartIdx( MTK::SubManifold<T, idx, dim> Base::*)
{
	return idx;
}

/**
 * @brief 查询子流形在过程表示向量中的起始偏移。
 * @return 编译期 dim。
 * @note 未命名成员指针参数用于推导子流形类型及编译期偏移，不读取对象值。
 */
template<class Base, class T, int idx, int dim> 
int getStartIdx_( MTK::SubManifold<T, idx, dim> Base::*)
{
	return dim;
}

/**
 * Determine the degrees of freedom of a sub-variable within a compound variable.
 */
/**
 * @brief 查询子流形的局部自由度。
 * @return T::DOF。
 * @note 未命名成员指针参数用于推导子流形类型及编译期偏移，不读取对象值。
 */
template<class Base, class T, int idx, int dim> 
int getDof( MTK::SubManifold<T, idx, dim> Base::*)
{
	return T::DOF;
}
/**
 * @brief 查询子流形的过程表示维数。
 * @return T::DIM。
 * @note 未命名成员指针参数用于推导子流形类型及编译期偏移，不读取对象值。
 */
template<class Base, class T, int idx, int dim> 
int getDim( MTK::SubManifold<T, idx, dim> Base::*)
{
	return T::DIM;
}

/**
 * set the diagonal elements of a covariance matrix corresponding to a sub-variable
 */
/**
 * @brief 设置子流形对应的矩阵对角片段，保留其他元素。
 * @param cov 待修改矩阵；普通版本按 DOF 布局，下划线版本按 DIM 布局。
 * @param val 写入每个选定对角元素的数值，函数不检查非负性。
 * @note 未命名成员指针参数用于推导子流形类型及编译期偏移，不读取对象值。
 */
template<class Base, class T, int idx, int dim> 
void setDiagonal(Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF> &cov, 
		MTK::SubManifold<T, idx, dim> Base::*, const typename Base::scalar &val)
{
	cov.diagonal().template segment<T::DOF>(idx).setConstant(val);
}

/**
 * @brief 设置子流形对应的矩阵对角片段，保留其他元素。
 * @param cov 待修改矩阵；普通版本按 DOF 布局，下划线版本按 DIM 布局。
 * @param val 写入每个选定对角元素的数值，函数不检查非负性。
 * @note 未命名成员指针参数用于推导子流形类型及编译期偏移，不读取对象值。
 */
template<class Base, class T, int idx, int dim> 
void setDiagonal_(Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM> &cov, 
		MTK::SubManifold<T, idx, dim> Base::*, const typename Base::scalar &val)
{
	cov.diagonal().template segment<T::DIM>(dim).setConstant(val);
}

/**
 * Get the subblock of corresponding to two members, i.e.
 * \code
 *  Eigen::Matrix<double, Pose::DOF, Pose::DOF> m;
 *  MTK::subblock(m, &Pose::orient, &Pose::trans) = some_expression;
 *  MTK::subblock(m, &Pose::trans, &Pose::orient) = some_expression.trans();
 * \endcode
 * lets you modify mixed covariance entries in a bigger covariance matrix.
 */
/**
 * @brief 借用矩阵中一个子流形或两个子流形交叉的块。
 * @param cov 原矩阵；普通版本按 DOF，带下划线版本按 DIM 排列。
 * @return 共享 cov 存储的可写 Eigen::Block；单成员指针版本取对应对角块。
 * @note 返回块不能比 cov 活得更久。
 * @note 未命名成员指针参数用于推导子流形类型及编译期偏移，不读取对象值。
 */
template<class Base, class T1, int idx1, int dim1, class T2, int idx2, int dim2>
typename MTK::internal::CovBlock<Base, T1, T2>::Type
subblock(Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF> &cov, 
		MTK::SubManifold<T1, idx1, dim1> Base::*, MTK::SubManifold<T2, idx2, dim2> Base::*)
{
	return cov.template block<T1::DOF, T2::DOF>(idx1, idx2);
}

/**
 * @brief 借用矩阵中一个子流形或两个子流形交叉的块。
 * @param cov 原矩阵；普通版本按 DOF，带下划线版本按 DIM 排列。
 * @return 共享 cov 存储的可写 Eigen::Block；单成员指针版本取对应对角块。
 * @note 返回块不能比 cov 活得更久。
 * @note 未命名成员指针参数用于推导子流形类型及编译期偏移，不读取对象值。
 */
template<class Base, class T1, int idx1,  int dim1, class T2, int idx2, int dim2>
typename MTK::internal::CovBlock_<Base, T1, T2>::Type
subblock_(Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM> &cov, 
		MTK::SubManifold<T1, idx1, dim1> Base::*, MTK::SubManifold<T2, idx2, dim2> Base::*)
{
	return cov.template block<T1::DIM, T2::DIM>(dim1, dim2);
}

/**
 * @brief 借用矩阵中一个子流形或两个子流形交叉的块。
 * @param cov 原矩阵；普通版本按 DOF，带下划线版本按 DIM 排列。
 * @return 共享 cov 存储的可写 Eigen::Block；单成员指针版本取对应对角块。
 * @note 返回块不能比 cov 活得更久。
 * @note 未命名成员指针参数用于推导子流形类型及编译期偏移，不读取对象值。
 */
template<typename Base1, typename Base2, typename T1, typename T2, int idx1, int idx2, int dim1, int dim2>
typename MTK::internal::CrossCovBlock<Base1, Base2, T1, T2>::Type
subblock(Eigen::Matrix<typename Base1::scalar, Base1::DOF, Base2::DOF> &cov, MTK::SubManifold<T1, idx1, dim1> Base1::*, MTK::SubManifold<T2, idx2, dim2> Base2::*)
{
	return cov.template block<T1::DOF, T2::DOF>(idx1, idx2);
}

/**
 * @brief 借用矩阵中一个子流形或两个子流形交叉的块。
 * @param cov 原矩阵；普通版本按 DOF，带下划线版本按 DIM 排列。
 * @return 共享 cov 存储的可写 Eigen::Block；单成员指针版本取对应对角块。
 * @note 返回块不能比 cov 活得更久。
 * @note 未命名成员指针参数用于推导子流形类型及编译期偏移，不读取对象值。
 */
template<typename Base1, typename Base2, typename T1, typename T2, int idx1, int idx2, int dim1, int dim2>
typename MTK::internal::CrossCovBlock_<Base1, Base2, T1, T2>::Type
subblock_(Eigen::Matrix<typename Base1::scalar, Base1::DIM, Base2::DIM> &cov, MTK::SubManifold<T1, idx1, dim1> Base1::*, MTK::SubManifold<T2, idx2, dim2> Base2::*)
{
	return cov.template block<T1::DIM, T2::DIM>(dim1, dim2);
}
/**
 * Get the subblock of corresponding to a member, i.e.
 * \code
 *  Eigen::Matrix<double, Pose::DOF, Pose::DOF> m;
 *  MTK::subblock(m, &Pose::orient) = some_expression;
 * \endcode
 * lets you modify covariance entries in a bigger covariance matrix.
 */
/**
 * @brief 借用矩阵中一个子流形或两个子流形交叉的块。
 * @param cov 原矩阵；普通版本按 DOF，带下划线版本按 DIM 排列。
 * @return 共享 cov 存储的可写 Eigen::Block；单成员指针版本取对应对角块。
 * @note 返回块不能比 cov 活得更久。
 * @note 未命名成员指针参数用于推导子流形类型及编译期偏移，不读取对象值。
 */
template<class Base, class T, int idx, int dim>
typename MTK::internal::CovBlock_<Base, T, T>::Type
subblock_(Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM> &cov, 
		MTK::SubManifold<T, idx, dim> Base::*)
{
	return cov.template block<T::DIM, T::DIM>(dim, dim);
}

/**
 * @brief 借用矩阵中一个子流形或两个子流形交叉的块。
 * @param cov 原矩阵；普通版本按 DOF，带下划线版本按 DIM 排列。
 * @return 共享 cov 存储的可写 Eigen::Block；单成员指针版本取对应对角块。
 * @note 返回块不能比 cov 活得更久。
 * @note 未命名成员指针参数用于推导子流形类型及编译期偏移，不读取对象值。
 */
template<class Base, class T, int idx, int dim>
typename MTK::internal::CovBlock<Base, T, T>::Type
subblock(Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF> &cov, 
		MTK::SubManifold<T, idx, dim> Base::*)
{
	return cov.template block<T::DOF, T::DOF>(idx, idx);
}

/**
 * @brief 根据状态 DOF 定义局部误差协方差矩阵类型。
 */
template<typename Base>
class get_cov { 
public:
    typedef Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF> type;
    typedef const Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF> const_type;
};

/**
 * @brief 根据状态 DIM 定义过程表示矩阵类型。
 */
template<typename Base>
class get_cov_ { 
public:
    typedef Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM> type;
    typedef const Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM> const_type;
};

/**
 * @brief 根据两个状态 DOF 定义交叉协方差矩阵类型。
 */
template<typename Base1, typename Base2>
class get_cross_cov {
public:
    typedef Eigen::Matrix<typename Base1::scalar, Base1::DOF, Base2::DOF> type;
    typedef const type const_type;
};

/**
 * @brief 根据两个状态 DIM 定义交叉矩阵类型。
 */
template<typename Base1, typename Base2>
class get_cross_cov_ {
public:
    typedef Eigen::Matrix<typename Base1::scalar, Base1::DIM, Base2::DIM> type;
    typedef const type const_type;
};


/**
 * @brief 取得成员对应的向量片段视图。
 * @param vec 原向量或视图；普通版本采用 DOF/idx，下划线版本采用 DIM/dim。
 * @return 共享原存储的子视图；只读重载不允许通过视图修改元素。
 * @note 底层向量必须连续且在使用视图期间有效。
 * @note 未命名成员指针参数用于推导子流形类型及编译期偏移，不读取对象值。
 */
template<class Base, class T, int idx, int dim>
vectview<typename Base::scalar, T::DIM>
subvector_impl_(vectview<typename Base::scalar, Base::DIM> vec, SubManifold<T, idx, dim> Base::*)
{
	return vec.template segment<T::DIM>(dim);
}

/**
 * @brief 取得成员对应的向量片段视图。
 * @param vec 原向量或视图；普通版本采用 DOF/idx，下划线版本采用 DIM/dim。
 * @return 共享原存储的子视图；只读重载不允许通过视图修改元素。
 * @note 底层向量必须连续且在使用视图期间有效。
 * @note 未命名成员指针参数用于推导子流形类型及编译期偏移，不读取对象值。
 */
template<class Base, class T, int idx, int dim>
vectview<typename Base::scalar, T::DOF>
subvector_impl(vectview<typename Base::scalar, Base::DOF> vec, SubManifold<T, idx, dim> Base::*)
{
	return vec.template segment<T::DOF>(idx);
}

/**
 * Get the subvector corresponding to a sub-manifold from a bigger vector.
 */
 /**
  * @brief 取得成员对应的向量片段视图。
  * @param vec 原向量或视图；普通版本采用 DOF/idx，下划线版本采用 DIM/dim。
  * @param ptr 子流形成员指针，推导片段偏移及长度。
  * @return 共享原存储的子视图；只读重载不允许通过视图修改元素。
  * @note 底层向量必须连续且在使用视图期间有效。
  */
 template<class Scalar, int BaseDIM, class Base, class T, int idx, int dim>
vectview<Scalar, T::DIM>
subvector_(vectview<Scalar, BaseDIM> vec, SubManifold<T, idx, dim> Base::* ptr)
{
	return subvector_impl_(vec, ptr);
}

/**
 * @brief 取得成员对应的向量片段视图。
 * @param vec 原向量或视图；普通版本采用 DOF/idx，下划线版本采用 DIM/dim。
 * @param ptr 子流形成员指针，推导片段偏移及长度。
 * @return 共享原存储的子视图；只读重载不允许通过视图修改元素。
 * @note 底层向量必须连续且在使用视图期间有效。
 */
template<class Scalar, int BaseDOF, class Base, class T, int idx, int dim>
vectview<Scalar, T::DOF>
subvector(vectview<Scalar, BaseDOF> vec, SubManifold<T, idx, dim> Base::* ptr)
{
	return subvector_impl(vec, ptr);
}

/**
 * @todo This should be covered already by subvector(vectview<typename Base::scalar,Base::DOF> vec,SubManifold<T,idx> Base::*)
 */
/**
 * @brief 取得成员对应的向量片段视图。
 * @param vec 原向量或视图；普通版本采用 DOF/idx，下划线版本采用 DIM/dim。
 * @param ptr 子流形成员指针，推导片段偏移及长度。
 * @return 共享原存储的子视图；只读重载不允许通过视图修改元素。
 * @note 底层向量必须连续且在使用视图期间有效。
 */
template<class Scalar, int BaseDOF, class Base, class T, int idx, int dim>
vectview<Scalar, T::DOF>
subvector(Eigen::Matrix<Scalar, BaseDOF, 1>& vec, SubManifold<T, idx, dim> Base::* ptr)
{
	return subvector_impl(vectview<Scalar, BaseDOF>(vec), ptr);
}
 
/**
 * @brief 取得成员对应的向量片段视图。
 * @param vec 原向量或视图；普通版本采用 DOF/idx，下划线版本采用 DIM/dim。
 * @param ptr 子流形成员指针，推导片段偏移及长度。
 * @return 共享原存储的子视图；只读重载不允许通过视图修改元素。
 * @note 底层向量必须连续且在使用视图期间有效。
 */
template<class Scalar, int BaseDIM, class Base, class T, int idx, int dim>
vectview<Scalar, T::DIM>
subvector_(Eigen::Matrix<Scalar, BaseDIM, 1>& vec, SubManifold<T, idx, dim> Base::* ptr)
{
	return subvector_impl_(vectview<Scalar, BaseDIM>(vec), ptr);
}

/**
 * @brief 取得成员对应的向量片段视图。
 * @param vec 原向量或视图；普通版本采用 DOF/idx，下划线版本采用 DIM/dim。
 * @param ptr 子流形成员指针，推导片段偏移及长度。
 * @return 共享原存储的子视图；只读重载不允许通过视图修改元素。
 * @note 底层向量必须连续且在使用视图期间有效。
 */
template<class Scalar, int BaseDIM, class Base, class T, int idx, int dim>
vectview<const Scalar, T::DIM>
subvector_(const Eigen::Matrix<Scalar, BaseDIM, 1>& vec, SubManifold<T, idx, dim> Base::* ptr)
{
	return subvector_impl_(vectview<const Scalar, BaseDIM>(vec), ptr);
}

/**
 * @brief 取得成员对应的向量片段视图。
 * @param vec 原向量或视图；普通版本采用 DOF/idx，下划线版本采用 DIM/dim。
 * @param ptr 子流形成员指针，推导片段偏移及长度。
 * @return 共享原存储的子视图；只读重载不允许通过视图修改元素。
 * @note 底层向量必须连续且在使用视图期间有效。
 */
template<class Scalar, int BaseDOF, class Base, class T, int idx, int dim>
vectview<const Scalar, T::DOF>
subvector(const Eigen::Matrix<Scalar, BaseDOF, 1>& vec, SubManifold<T, idx, dim> Base::* ptr)
{
	return subvector_impl(vectview<const Scalar, BaseDOF>(vec), ptr);
}


/**
 * const version of subvector(vectview<typename Base::scalar,Base::DOF> vec,SubManifold<T,idx> Base::*)
 */
/**
 * @brief 取得成员对应的向量片段视图。
 * @param cvec 原只读视图，按当前版本的 DOF 或 DIM 布局。
 * @return 共享原存储的子视图；只读重载不允许通过视图修改元素。
 * @note 底层向量必须连续且在使用视图期间有效。
 * @note 未命名成员指针参数用于推导子流形类型及编译期偏移，不读取对象值。
 */
template<class Base, class T, int idx, int dim>
vectview<const typename Base::scalar, T::DOF>
subvector_impl(const vectview<const typename Base::scalar, Base::DOF> cvec, SubManifold<T, idx, dim> Base::*)
{
	return cvec.template segment<T::DOF>(idx);
}

/**
 * @brief 取得成员对应的向量片段视图。
 * @param cvec 原只读视图，按当前版本的 DOF 或 DIM 布局。
 * @return 共享原存储的子视图；只读重载不允许通过视图修改元素。
 * @note 底层向量必须连续且在使用视图期间有效。
 * @note 未命名成员指针参数用于推导子流形类型及编译期偏移，不读取对象值。
 */
template<class Base, class T, int idx, int dim>
vectview<const typename Base::scalar, T::DIM>
subvector_impl_(const vectview<const typename Base::scalar, Base::DIM> cvec, SubManifold<T, idx, dim> Base::*)
{
	return cvec.template segment<T::DIM>(dim);
}

/**
 * @brief 取得成员对应的向量片段视图。
 * @param cvec 原只读视图，按当前版本的 DOF 或 DIM 布局。
 * @param ptr 子流形成员指针，推导片段偏移及长度。
 * @return 共享原存储的子视图；只读重载不允许通过视图修改元素。
 * @note 底层向量必须连续且在使用视图期间有效。
 */
template<class Scalar, int BaseDOF, class Base, class T, int idx, int dim>
vectview<const Scalar, T::DOF>
subvector(const vectview<const Scalar, BaseDOF> cvec, SubManifold<T, idx, dim> Base::* ptr)
{
	return subvector_impl(cvec, ptr);
}


} // namespace MTK

#endif // GET_START_INDEX_H_
