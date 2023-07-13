/**
 * @file z_tf.hpp
 * @author X. Y.
 * @brief Z 传递函数
 * @version 0.3
 * @date 2023-07-13
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "discrete_controller_base.hpp"
#include <vector>
#include <cassert>
#include "ring_list.hpp"

namespace control_system
{

/**
 * @brief Z传递函数
 *
 * @tparam T 数据类型，例如 float 或 double
 */
template <typename T>
class ZTf : public DiscreteControllerBase<T>
{
private:
    std::vector<T> input_c_, output_c_; // 输入系数 i0, i1, ... 和输出系数 o0, o1, ...

    typedef struct
    {
        T input;
        T output;
    } data_t;

    RingList<data_t> data_list_;

    size_t order_ = 0; // 系统阶数（等于分母阶数）
    size_t order_m1;   // order_ - 1（提前算好，加快运算速度）
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
        assert(den.at(0) != 0);

        int size_diff = den.size() - num.size(); // 分母维数与分子维数之差
        assert(size_diff >= 0);                  // 分子阶数不能大于分母，否则是非因果系统

        order_   = den.size();
        order_m1 = order_ - 1;

        input_c_.resize(order_);
        output_c_.resize(order_);
        data_list_.resize(order_m1);

        // 如果分子阶数小于分母，就往前面补一些 0
        for (int i = 0; i < size_diff; i++) {
            input_c_.at(i) = 0;
        }

        // 剩下的输入系数
        for (size_t i = size_diff; i < order_; i++) {
            input_c_.at(i) = num.at(i - size_diff) / den.at(0);
        }

        // 输出系数
        for (size_t i = 0; i < order_; i++) {
            output_c_.at(i) = -den.at(i) / den.at(0);
        }

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
        assert(order_ != 0);

        T output = input_c_[0] * input;

        data_t *data = &(data_list_.get());

        for (size_t i = 1; i < order_m1; i++) {
            output += input_c_[i] * data->input;
            output += output_c_[i] * data->output;
            data = &(data_list_.spin());
        }

        output += input_c_[order_m1] * data->input;
        output += output_c_[order_m1] * data->output;

        // 插入新数据，并把新数据放在链表开头
        data->input  = input;
        data->output = output;

        return output;
    }

    /**
     * @brief 重置内部状态
     *
     */
    void ResetState() override
    {
        data_list_.fill({0, 0});
    }
};

} // namespace control_system
