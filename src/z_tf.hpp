#pragma once

#include "discrete_tf.hpp"
#include <vector>

template <typename T>
class ZTf : public DiscreteTf<T>
{
private:
    std::vector<T> input_coefficient_, output_coefficient_;
    std::vector<T> input_, output_;
    size_t index_ = 0;
    size_t dim_;

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

        dim_ = den.size();

        if (num.size() < dim_) {
            // 使分子分母维数相同
            num.insert(num.begin(), dim_ - num.size(), 0);
        }

        input_coefficient_.resize(dim_);
        for (size_t i = 0; i < dim_; i++) {
            input_coefficient_.at(i) = num.at(i) / den.at(0);
        }

        output_coefficient_.resize(dim_ - 1);
        for (size_t i = 0; i < dim_ - 1; i++) {
            output_coefficient_.at(i) = -den.at(i + 1) / den.at(0);
        }

        input_.clear();
        input_.resize(dim_);
        output_.clear();
        output_.resize(dim_);
        index_ = 0;

        return true;
    }

    T Step(T input) override
    {
        input_[index_] = input;

        auto temp_index = index_;
        T output        = 0;
        auto dim_m1     = dim_ - 1;

        for (size_t i = 0; i < dim_m1; i++) {
            output += input_coefficient_[i] * input_[temp_index];
            output += output_coefficient_[i] * output_[temp_index];
            if (temp_index == 0) {
                temp_index = dim_m1;
            } else {
                temp_index--;
            }
        }

        output += input_coefficient_[dim_m1] * input_[temp_index];

        index_++;
        if (index_ >= dim_) {
            index_ = 0;
        }

        output_[index_] = output;

        return output;
    }

    void ResetState() override
    {
        std::fill(input_.begin(), input_.end(), 0);
        std::fill(output_.begin(), output_.end(), 0);
        index_ = 0;
    }

    /* T Step1(T input)
    {
        for (size_t i = dim_ - 1; i > 0; i--)
        {
            input_.at(i) = input_.at(i - 1);
        }

        input_.at(0) = input;

        T output = 0;

        for (size_t i = 0; i < dim_; i++)
        {
            output += input_coefficient_.at(i) * input_.at(i);
        }

        for (size_t i = 0; i < dim_ - 1; i++)
        {
            output += output_coefficient_.at(i) * output_.at(i);
        }

        for (size_t i = dim_ - 1; i > 0; i--)
        {
            output_.at(i) = output_.at(i - 1);
        }

        output_.at(0) = output;

        return output;
    }

    T Step2(T input)
    {
        std::move_backward(input_.begin(), input_.end() - 1, input_.end());

        input_.at(0) = input;

        T output = 0;

        for (size_t i = 0; i < dim_; i++)
        {
            output += input_coefficient_.at(i) * input_.at(i);
        }

        for (size_t i = 0; i < dim_ - 1; i++)
        {
            output += output_coefficient_.at(i) * output_.at(i);
        }

        std::move_backward(output_.begin(), output_.end() - 1, output_.end());

        output_.at(0) = output;

        return output;
    } */
};
