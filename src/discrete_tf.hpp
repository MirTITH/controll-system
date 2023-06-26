#pragma once

/**
 * @brief 离散传递函数
 *
 * @tparam T 数据类型，例如 float 或 double
 */
template <typename T>
class DiscreteTf
{
private:
    /* data */
public:
    /**
     * @brief 走一个周期
     *
     * @param input 输入
     * @return  输出
     */
    virtual T Step(T input) = 0;

    /**
     * @brief 重置传函内部状态
     *
     */
    virtual void ResetState() = 0;
};
