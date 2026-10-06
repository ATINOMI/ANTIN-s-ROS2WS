/*
 *  Copyright (c) 2019--2023, The University of Hong Kong
 *  All rights reserved.
 *
 *  Author: Dongjiao HE <hdj65822@connect.hku.hk>
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
 * @file util.hpp
 * @brief 迭代误差状态滤波的类型检查和恒等复制工具。
 * @author Antinomy
 * @date 2026-10-05
 */
#ifndef __MEKFOM_UTIL_HPP__
#define __MEKFOM_UTIL_HPP__

#include <Eigen/Core>
#include "../mtk/src/mtkmath.hpp"
namespace esekfom {

/**
 * @brief 判断两个模板类型是否相同，主模板返回 false。
 */
template <typename T1, typename T2>
class is_same {
public:
    /**
     * @brief 返回当前类型特化的判断结果。
     * @return 匹配的特化为 true，主模板为 false。
     */
    operator bool() {
        return false;
    }
};
/**
 * @brief 相同类型的特化，转换为 bool 时返回 true。
 */
template<typename T1>
class is_same<T1, T1> {
public:
    /**
     * @brief 返回当前类型特化的判断结果。
     * @return 匹配的特化为 true，主模板为 false。
     */
    operator bool() {
        return true;
    }
};

/**
 * @brief 判断模板参数是否为 double，主模板返回 false。
 */
template <typename T>
class is_double {
public:
    /**
     * @brief 返回当前类型特化的判断结果。
     * @return 匹配的特化为 true，主模板为 false。
     */
    operator bool() {
        return false;
    }
};

/**
 * @brief double 类型特化，转换为 bool 时返回 true。
 */
template<>
class is_double<double> {
public:
    /**
     * @brief 返回当前类型特化的判断结果。
     * @return 匹配的特化为 true，主模板为 false。
     */
    operator bool() {
        return true;
    }
};

/**
 * @brief 按值复制并返回输入。
 * @param x 待复制的值。
 * @return 与 x 等值的副本。
 */
template<typename T>
static T
id(const T &x)
{
	return x;
}

} // namespace esekfom
	
#endif // __MEKFOM_UTIL_HPP__
