#pragma once

#include <deque>
#include <vector>

template <typename T>
class DiscreteTf
{
private:
    std::vector<T> input_coefficient_, output_coefficient_;
    std::deque<T> input_, output_;
    size_t dim_;

public:
    DiscreteTf(){};

    DiscreteTf(std::vector<T> num, std::vector<T> den)
    {
        Init(num, den);
    }

    bool Init(std::vector<T> num, std::vector<T> den)
    {
        if (num.size() > den.size())
        {
            // 分子阶数不能大于分母，否则是非因果系统
            return false;
        }

        dim_ = den.size();

        if (num.size() < dim_)
        {
            // 使分子分母维数相同
            num.insert(num.begin(), dim_ - num.size(), 0);
        }

        input_coefficient_.resize(dim_);
        for (size_t i = 0; i < dim_; i++)
        {
            input_coefficient_.at(i) = num.at(i) / den.at(0);
        }

        output_coefficient_.resize(dim_ - 1);
        for (size_t i = 0; i < dim_ - 1; i++)
        {
            output_coefficient_.at(i) = den.at(i + 1) / den.at(0);
        }

        input_.clear();
        input_.resize(dim_);
        output_.clear();
        output_.resize(dim_);

        return true;
    }

    T Step(T input)
    {
        input_.pop_back();
        input_.push_front(input);
        T output = 0;

        for (size_t i = 0; i < dim_; i++)
        {
            output += input_coefficient_.at(i) * input_.at(i);
        }

        for (size_t i = 0; i < dim_ - 1; i++)
        {
            output -= output_coefficient_.at(i) * output_.at(i);
        }

        output_.pop_back();
        output_.push_front(output);
        return output;
    }

    void ResetState()
    {
        for (auto &var : input_)
        {
            var = 0;
        }

        for (auto &var : output_)
        {
            var = 0;
        }
    }
};
