/*
 *  Copyright (c) 2010--2011, Universitaet Bremen and DFKI GmbH
 *  All rights reserved.
 *
 *  Author: Rene Wagner <rene.wagner@dfki.de>
 *          Christoph Hertzberg <chtz@informatik.uni-bremen.de>
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
 *   * Neither the name of the Universitaet Bremen nor the DFKI GmbH 
 *     nor the names of its contributors may be used to endorse or 
 *     promote products derived from this software without specific 
 *     prior written permission.
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
 * @file wrapped_cv_mat.hpp
 * @brief 以共享存储的 CvMat 头适配 Eigen 矩阵，保留旧 OpenCV C API 接口。拷贝构造必须重绑矩阵头，避免引用源对象。
 * @author Antinomy
 * @date 2026-10-05
 */
#ifndef WRAPPED_CV_MAT_HPP_
#define WRAPPED_CV_MAT_HPP_

#include <Eigen/Core>
#include <opencv/cv.h>

namespace MTK {

/**
 * @brief 声明 C++ 标量到 OpenCV 矩阵深度的映射；仅提供 float/double 特化。
 */
template<class f_type>
struct cv_f_type;

/**
 * @brief 将 double 映射为 CV_64F。
 */
template<>
struct cv_f_type<double>
{
	enum {value = CV_64F};
};

/**
 * @brief 将 float 映射为 CV_32F。
 */
template<>
struct cv_f_type<float>
{
	enum {value = CV_32F};
};

/**
 * cv_mat wraps a CvMat around an Eigen Matrix
 */
/**
 * @brief 将 Eigen 自有存储包装为共享数据的 CvMat 头；一般矩阵按行存储，单列向量按列存储。
 */
template<int rows, int cols, class f_type = double>
class cv_mat : public matrix<rows, cols, f_type, cols==1 ? Eigen::ColMajor : Eigen::RowMajor>
{
	typedef matrix<rows, cols, f_type, cols==1 ? Eigen::ColMajor : Eigen::RowMajor> base_type;
	enum {type_ = cv_f_type<f_type>::value};
	CvMat cv_mat_;

public:
	/**
	 * @brief 构造矩阵并将 CvMat 头绑定到本对象存储。
	 */
	cv_mat()
	{
		cv_mat_ = cvMat(rows, cols, type_, base_type::data());
	}

	/**
	 * @brief 构造矩阵并将 CvMat 头绑定到本对象存储。
	 * @param oth 复制源，复制数值后重新绑定当前对象数据。
	 */
	cv_mat(const cv_mat& oth) : base_type(oth)
	{
		cv_mat_ = cvMat(rows, cols, type_, base_type::data());
	}

	/**
	 * @brief 构造矩阵并将 CvMat 头绑定到本对象存储。
	 * @param value 初始化矩阵的 Eigen 表达式，尺寸须与模板一致。
	 */
	template<class Derived>
	cv_mat(const Eigen::MatrixBase<Derived> &value) : base_type(value)
	{
		cv_mat_ = cvMat(rows, cols, type_, base_type::data());
	}

	/**
	 * @brief 赋值矩阵元素并保持当前 CvMat 头引用本对象存储。
	 * @param value 相同尺寸的源矩阵或 Eigen 表达式。
	 * @return 当前对象引用。
	 */
	template<class Derived>
	cv_mat& operator=(const Eigen::MatrixBase<Derived> &value)
	{
		base_type::operator=(value);
		return *this;
	}

	/**
	 * @brief 赋值矩阵元素并保持当前 CvMat 头引用本对象存储。
	 * @param value 相同尺寸的源矩阵或 Eigen 表达式。
	 * @return 当前对象引用。
	 */
	cv_mat& operator=(const cv_mat& value)
	{
		base_type::operator=(value);
		return *this;
	}
	
	// FIXME: Maybe overloading operator& is not a good idea ...
	/**
	 * @brief 取得共享当前矩阵数据的 OpenCV 矩阵头。
	 * @return 内部 CvMat 的指针；只读重载返回 const 指针。
	 * @note 重载地址运算符，因此 &对象 返回 CvMat 头而不是 C++ 对象地址；对象销毁后失效。
	 */
	CvMat* operator&()
	{
		return &cv_mat_;
	}
	/**
	 * @brief 取得共享当前矩阵数据的 OpenCV 矩阵头。
	 * @return 内部 CvMat 的指针；只读重载返回 const 指针。
	 * @note 重载地址运算符，因此 &对象 返回 CvMat 头而不是 C++ 对象地址；对象销毁后失效。
	 */
	const CvMat* operator&() const
	{
		return &cv_mat_;
	}
};

} // namespace MTK

#endif /* WRAPPED_CV_MAT_HPP_ */
