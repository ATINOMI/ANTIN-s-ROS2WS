
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
 * @file mtk/src/vectview.hpp
 * @brief Wrapper class around a pointer used as interface for plain vectors.
 */

/**
 * @file vectview.hpp
 * @brief 提供 Eigen 向量存储的可写/只读视图及协方差子块类型。视图不持有数据，底层内存必须连续且持续有效。
 * @author Antinomy
 * @date 2026-10-05
 */
#ifndef VECTVIEW_HPP_
#define VECTVIEW_HPP_

#include <Eigen/Core>

namespace MTK {

/**
 * A view to a vector.
 * Essentially, @c vectview is only a pointer to @c scalar but can be used directly in @c Eigen expressions.
 * The dimension of the vector is given as template parameter and type-checked when used in expressions.
 * Data has to be modifiable.
 * 
 * @tparam scalar Scalar type of the vector.
 * @tparam dim    Dimension of the vector.
 * 
 * @todo @c vectview can be replaced by simple inheritance of @c Eigen::Map, as soon as they get const-correct
 */
namespace internal {
	/**
	 * @brief 组合状态的 DOF×DOF 协方差子块类型。
	 */
	template<class Base, class T1, class T2>
	struct CovBlock {
		typedef typename Eigen::Block<Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF>, T1::DOF, T2::DOF> Type;
		typedef typename Eigen::Block<const Eigen::Matrix<typename Base::scalar, Base::DOF, Base::DOF>, T1::DOF, T2::DOF> ConstType;
	};

	/**
	 * @brief 组合状态的 DIM×DIM 矩阵子块类型。
	 */
	template<class Base, class T1, class T2>
	struct CovBlock_ {
		typedef typename Eigen::Block<Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM>, T1::DIM, T2::DIM> Type;
		typedef typename Eigen::Block<const Eigen::Matrix<typename Base::scalar, Base::DIM, Base::DIM>, T1::DIM, T2::DIM> ConstType;
	};

	/**
	 * @brief 两个状态间按 DOF 定义的交叉协方差子块类型。
	 */
	template<typename Base1, typename Base2, typename T1, typename T2>
	struct CrossCovBlock {
		typedef typename Eigen::Block<Eigen::Matrix<typename Base1::scalar, Base1::DOF, Base2::DOF>, T1::DOF, T2::DOF> Type;
		typedef typename Eigen::Block<const Eigen::Matrix<typename Base1::scalar, Base1::DOF, Base2::DOF>, T1::DOF, T2::DOF> ConstType;
	};

	/**
	 * @brief 两个状态间按 DIM 定义的交叉矩阵子块类型。
	 */
	template<typename Base1, typename Base2, typename T1, typename T2>
	struct CrossCovBlock_ {
		typedef typename Eigen::Block<Eigen::Matrix<typename Base1::scalar, Base1::DIM, Base2::DIM>, T1::DIM, T2::DIM> Type;
		typedef typename Eigen::Block<const Eigen::Matrix<typename Base1::scalar, Base1::DIM, Base2::DIM>, T1::DIM, T2::DIM> ConstType;
	};

	/**
	 * @brief 为标量及维数选择 Eigen 可写和只读 Map 类型。
	 */
	template<class scalar, int dim>
	struct VectviewBase {
		typedef Eigen::Matrix<scalar, dim, 1> matrix_type;
		typedef typename matrix_type::MapType Type;
		typedef typename matrix_type::ConstMapType ConstType;
	};

	/**
	 * @brief 默认保持类型，具体数值类型可特化以关闭对齐。
	 */
	template<class T>
	struct UnalignedType {
		typedef T type;
	};
}

/**
 * @brief 借用连续标量存储构造可写向量视图。
 * @note 不分配内存；矩阵块必须连续，原对象销毁或重新分配后视图失效。
 */
template<class scalar, int dim>
class vectview : public internal::VectviewBase<scalar, dim>::Type {
	typedef internal::VectviewBase<scalar, dim> VectviewBase;
public:
	//! plain matrix type
	typedef typename VectviewBase::matrix_type matrix_type;
	//! base type
	typedef typename VectviewBase::Type base;
	//! construct from pointer
	/**
	 * @brief 将已有连续内存映射为向量视图。
	 * @param data 底层首元素指针，须覆盖所需维数且持续有效。
	 * @param dim_ 运行时元素数，固定维数时必须与 dim 一致。
	 * @note 构造视图不延长原存储生命周期，不能保留指向临时表达式的视图。
	 */
	explicit
	vectview(scalar* data, int dim_=dim) : base(data, dim_) {}
	//! construct from plain matrix
	/**
	 * @brief 将已有连续内存映射为向量视图。
	 * @param m 提供连续存储的矩阵或向量，不发生数据复制。
	 * @note 构造视图不延长原存储生命周期，不能保留指向临时表达式的视图。
	 */
	vectview(matrix_type& m) : base(m.data(), m.size()) {}
	//! construct from another @c vectview
	/**
	 * @brief 将已有连续内存映射为向量视图。
	 * @param v 共享底层存储的源视图。
	 * @note 构造视图不延长原存储生命周期，不能保留指向临时表达式的视图。
	 */
	vectview(const vectview &v) : base(v) {}
	//! construct from Eigen::Block:
	/**
	 * @brief 将已有连续内存映射为向量视图。
	 * @param block 连续向量块；不支持用此接口表达任意跨步存储。
	 * @note 构造视图不延长原存储生命周期，不能保留指向临时表达式的视图。
	 */
	template<class Base>
	vectview(Eigen::VectorBlock<Base, dim> block) : base(&block.coeffRef(0), block.size()) {}
	/**
	 * @brief 将已有连续内存映射为向量视图。
	 * @param block 连续向量块；不支持用此接口表达任意跨步存储。
	 * @note 构造视图不延长原存储生命周期，不能保留指向临时表达式的视图。
	 */
	template<class Base, bool PacketAccess>
	vectview(Eigen::Block<Base, dim, 1, PacketAccess> block) : base(&block.coeffRef(0), block.size()) {}

	//! inherit assignment operator
	using base::operator=;
	//! data pointer
	/**
	 * @brief 取得底层可写首元素指针。
	 * @return 原存储指针，生命周期和范围与当前视图相同。
	 */
	scalar* data() {return const_cast<scalar*>(base::data());}
};

/**
 * @c const version of @c vectview.
 * Compared to @c Eigen::Map this implementation is const correct, i.e.,
 * data will not be modifiable using this view.
 * 
 * @tparam scalar Scalar type of the vector.
 * @tparam dim    Dimension of the vector.
 * 
 * @sa vectview
 */
/**
 * @brief 借用连续标量存储构造只读向量视图。
 * @note const 限制通过视图写入，不保证原存储不会被其他引用改变。
 */
template<class scalar, int dim>
class vectview<const scalar, dim> : public internal::VectviewBase<scalar, dim>::ConstType {
	typedef internal::VectviewBase<scalar, dim> VectviewBase;
public:
	//! plain matrix type
	typedef typename VectviewBase::matrix_type matrix_type;
	//! base type
	typedef typename VectviewBase::ConstType base;
	//! construct from const pointer
	/**
	 * @brief 将已有连续内存映射为向量视图。
	 * @param data 底层首元素指针，须覆盖所需维数且持续有效。
	 * @param dim_ 运行时元素数，固定维数时必须与 dim 一致。
	 * @note 构造视图不延长原存储生命周期，不能保留指向临时表达式的视图。
	 */
	explicit
	vectview(const scalar* data, int dim_ = dim) : base(data, dim_) {}
	//! construct from column vector
	/**
	 * @brief 将已有连续内存映射为向量视图。
	 * @param m 提供连续存储的矩阵或向量，不发生数据复制。
	 * @note 构造视图不延长原存储生命周期，不能保留指向临时表达式的视图。
	 */
	template<int options>
	vectview(const Eigen::Matrix<scalar, dim, 1, options>& m) : base(m.data()) {}
	//! construct from row vector
	/**
	 * @brief 将已有连续内存映射为向量视图。
	 * @param m 提供连续存储的矩阵或向量，不发生数据复制。
	 * @note 构造视图不延长原存储生命周期，不能保留指向临时表达式的视图。
	 */
	template<int options, int phony>
	vectview(const Eigen::Matrix<scalar, 1, dim, options, phony>& m) : base(m.data()) {}
	//! construct from another @c vectview
	/**
	 * @brief 将已有连续内存映射为向量视图。
	 * @param x 共享存储的源视图或 Eigen Map。
	 * @note 构造视图不延长原存储生命周期，不能保留指向临时表达式的视图。
	 */
	vectview(vectview<scalar, dim> x) : base(x.data()) {}
	//! construct from base
	/**
	 * @brief 将已有连续内存映射为向量视图。
	 * @param x 共享存储的源视图或 Eigen Map。
	 * @note 构造视图不延长原存储生命周期，不能保留指向临时表达式的视图。
	 */
	vectview(const base &x) : base(x) {}
	/**
	 * Construct from Block
	 * @todo adapt this, when Block gets const-correct
	 */
	/**
	 * @brief 将已有连续内存映射为向量视图。
	 * @param block 连续向量块；不支持用此接口表达任意跨步存储。
	 * @note 构造视图不延长原存储生命周期，不能保留指向临时表达式的视图。
	 */
	template<class Base>
	vectview(Eigen::VectorBlock<Base, dim> block) : base(&block.coeffRef(0)) {}
	/**
	 * @brief 将已有连续内存映射为向量视图。
	 * @param block 连续向量块；不支持用此接口表达任意跨步存储。
	 * @note 构造视图不延长原存储生命周期，不能保留指向临时表达式的视图。
	 */
	template<class Base, bool PacketAccess>
	vectview(Eigen::Block<Base, dim, 1, PacketAccess> block) : base(&block.coeffRef(0)) {}

};


} // namespace MTK

#endif /* VECTVIEW_HPP_ */
