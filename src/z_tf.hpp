/**
 * @file z_tf.hpp
 * @author X. Y.
 * @brief Z 传递函数
 * @version 0.1
 * @date 2023-06-26
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "discrete_tf.hpp"
#include <vector>
#include <stdexcept>

/**
 * @brief Z传递函数
 *
 * @tparam T 数据类型，例如 float 或 double
 */
template <typename T>
class ZTf : public DiscreteTf<T>
{
private:
    std::vector<T> input_coefficient_, output_coefficient_; // 输入系数 i1, i2, ... 和输出系数 o1, o2, ...
    T input_c0;                                             // 输入系数 i0

    std::vector<T> inputs_, outputs_; // 输入和输出队列
    size_t dim_ = 0, dim_m1_ = 0;     // dim_：系统维数（等于分母维数）；dim_m1_：系统维数 - 1（提前算好，加快运算速度）
    size_t index_ = 0;                // 上一次输入和输出在队列中的位置

public:
    /**
     * @brief 创建空的 Z 传函
     * @note 由于没有分子和分母，之后必须调用 Init() 指定分子和分母才能调用 Step()
     */
    ZTf(){};

    /**
     * @brief 创建一个 Z 传函
     *
     * @param num 分子
     * @param den 分母
     * @note 分子阶数不能大于分母，否则是非因果系统
     */
    ZTf(const std::vector<T> &num, const std::vector<T> &den)
    {
        Init(num, den);
    }

    /**
     * @brief 初始化 Z 传函或重新指定 Z 传函的表达式
     *
     * @param num 分子
     * @param den 分母
     * @note 分子阶数不能大于分母，否则是非因果系统
     */
    void Init(const std::vector<T> &num, const std::vector<T> &den)
    {
        int size_diff = den.size() - num.size(); // 分母维数与分子维数之差

        if (size_diff < 0) {
            // 分子阶数不能大于分母，否则是非因果系统
            throw std::logic_error("The size of numerator is larger than the size of denominator.");
        }

        dim_    = den.size();
        dim_m1_ = dim_ - 1;

        input_coefficient_.resize(dim_m1_);
        output_coefficient_.resize(dim_m1_);

        if (den.at(0) == 0) {
            throw std::runtime_error("den.at(0) cannot be 0!");
        }

        // 输入系数 i0
        if (size_diff == 0) {
            input_c0 = num.at(0) / den.at(0);
        } else {
            // 如果分子阶数小于分母，i0 一定为 0
            input_c0 = 0;
        }

        // 其他输入系数 i1, i2, ...
        // 如果分子阶数小于分母，就往前面补一些 0
        for (int i = 0; i < size_diff; i++) {
            input_coefficient_.at(i) = 0;
        }

        // 剩下的输入系数
        for (size_t i = size_diff; i < dim_m1_; i++) {
            input_coefficient_.at(i) = num.at(i + 1 - size_diff) / den.at(0);
        }

        // 输出系数
        for (size_t i = 0; i < dim_m1_; i++) {
            output_coefficient_.at(i) = -den.at(i + 1) / den.at(0);
        }

        inputs_.resize(dim_);
        outputs_.resize(dim_);
        ResetState();
    }

    /**
     * @brief 走一个周期
     *
     * @param input 输入
     * @return 输出
     */
    T Step(T input) override
    {
        if (dim_ == 0) {
            throw std::runtime_error("dim_ == 0! Did you forget to call Init()?");
        }

        T output = input_c0 * input;
        for (size_t i = 0; i < dim_m1_; i++) {
            output += input_coefficient_[i] * inputs_[index_];
            output += output_coefficient_[i] * outputs_[index_];
            if (index_ < dim_m1_) {
                index_++;
            } else {
                index_ = 0;
            }
        }

        inputs_[index_]  = input;
        outputs_[index_] = output;

        return output;
    }

    /**
     * @brief 重置内部状态
     *
     */
    void ResetState() override
    {
        std::fill(inputs_.begin(), inputs_.end(), 0);
        std::fill(outputs_.begin(), outputs_.end(), 0);
        index_ = 0;
    }
};