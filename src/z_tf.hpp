#pragma once

#include "discrete_tf.hpp"
#include <vector>

template <typename T>
class ZTf : public DiscreteTf<T>
{
private:
    std::vector<T> input_coefficient_, output_coefficient_; // 输入系数 i1, i2, ... 和输出系数 o1, o2, ...
    T input_c0;                                             // 输入系数 i0

    std::vector<T> inputs_, outputs_; // 输入和输出队列
    size_t dim_, dim_m1_;             // dim_：系统维数（等于分母维数）；dim_m1_：系统维数 - 1（提前算好，加快运算速度）
    size_t index_ = 0;                // 上一次输入和输出在队列中的位置

public:
    ZTf(){};

    ZTf(std::vector<T> num, std::vector<T> den)
    {
        Init(num, den);
    }

    bool Init(std::vector<T> num, std::vector<T> den)
    {
        if (num.size() > den.size()) {
            // 分子阶数不能大于分母，否则是非因果系统
            return false;
        }

        dim_    = den.size();
        dim_m1_ = dim_ - 1;

        if (num.size() < dim_) {
            // 使分子分母维数相同
            num.insert(num.begin(), dim_ - num.size(), 0);
        }

        input_c0 = num.at(0) / den.at(0);
        input_coefficient_.resize(dim_ - 1);
        for (size_t i = 0; i < dim_ - 1; i++) {
            input_coefficient_.at(i) = num.at(i + 1) / den.at(0);
        }

        output_coefficient_.resize(dim_ - 1);
        for (size_t i = 0; i < dim_ - 1; i++) {
            output_coefficient_.at(i) = -den.at(i + 1) / den.at(0);
        }

        inputs_.resize(dim_);
        outputs_.resize(dim_);
        ResetState();

        return true;
    }

    T Step(T input) override
    {
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

    void ResetState() override
    {
        std::fill(inputs_.begin(), inputs_.end(), 0);
        std::fill(outputs_.begin(), outputs_.end(), 0);
        index_ = 0;
    }
};