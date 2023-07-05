/**
 * @file pid_controller.hpp
 * @author X. Y.
 * @brief PID 控制器
 * @version 0.2
 * @date 2023-07-05
 *
 * @copyright Copyright (c) 2023
 *
 * 这些控制器按照 Simulink 中的 Discrete PID Controller 模块设计，其中积分器方法和滤波器方法均为梯形（也就是双线性变换）
 * 你可以直接把 Discrete PID Controller 中的参数填入这些控制器中
 *
 * PID 控制器采用的公式为：
 *   Kp + Ki/s + Kd * Kn / (1 + Kn/s)
 *
 *   其中,
 *   s = 2/Ts * (z-1)/(z+1)
 *   Kp 为比例系数
 *   Ki 为积分系数
 *   Kd 为微分系数
 *   Kn 为滤波器系数
 *   Ts 为采样周期
 *
 * 参数说明：
 *   Kn 可以减弱微分器对高频噪声的敏感性。Kn 越小减弱效果越好，但微分器的响应更慢
 *   Kn 的值超过 2/Ts 时，微分器的阶跃响应会震荡
 *
 *   Ts 为采样周期，单位为秒。如 1000 Hz 的采样率，对应的 Ts 为 0.001.
 *   请使得调用控制器 Step() 的周期为 Ts
 *
 * 使用示例:
 *   定义 PID 控制器:
 *     pid::PID<float> pid_controller{1.23, 0.54, 0, 1000, 0.01};
 *
 *   使用 PID 控制器:
 *     while(1) {
 *         output_data = controller.Step(input_data);
 *         sleep_ms(10); // 因为 Ts = 0.01, 等待 10 ms
 *     }
 *
 */

#pragma once

#include "discrete_tf.hpp"
#include "z_tf.hpp"
#include "saturation.hpp"
#include <array>

namespace control_system
{

namespace pid
{

template <typename T>
class P : public DiscreteTf<T>
{
private:
    T Kp = 0;

public:
    P(T Kp)
    {
        ResetState();
        SetParam(Kp);
    }

    /**
     * @brief 走一个采样周期
     *
     * @param input 输入
     * @return T 输出
     */
    T Step(T input) override
    {
        return Kp * input;
    }

    /**
     * @brief 设置参数
     *
     * @param Kp 比例项
     */
    void SetParam(T Kp)
    {
        this->Kp = Kp;
    }

    T GetKp() const
    {
        return Kp;
    }

    void ResetState(){};
};

template <typename T>
class I : public DiscreteTf<T>
{
private:
    T Ki, Ts;
    T input_coefficient_;
    T last_input_;
    Saturation<T, T> saturation; // 限幅器

    void UpdateCoefficient()
    {
        input_coefficient_ = Ki * Ts / 2;
    }

public:
    T last_output_;

    /**
     * @brief 积分器
     *
     * @param Ki 积分器系数
     * @param Ts 采样周期
     *
     */
    I(T Ki, T Ts)
    {
        ResetState();
        SetParam(Ki, Ts);
        saturation.SetEnable(false); // 默认不使用限幅器
    }

    /**
     * @brief 走一个采样周期
     *
     * @param input 输入
     * @return T 输出
     */
    T Step(T input) override
    {
        last_output_ = saturation(input_coefficient_ * (input + last_input_) + last_output_);
        last_input_  = input;
        return last_output_;
    }

    void SetParam(T Ki, T Ts)
    {
        this->Ki = Ki;
        this->Ts = Ts;
        UpdateCoefficient();
    }

    void SetParam(T Ki)
    {
        this->Ki = Ki;
        UpdateCoefficient();
    }

    T GetKi() const
    {
        return Ki;
    }

    T GetTs() const
    {
        return Ts;
    }

    void SetOutputMinMax(T min, T max)
    {
        saturation.SetEnable(true); // 使能限幅器
        saturation.SetMinMax(min, max);
    }

    auto GetOutputMin() const
    {
        return saturation.GetMin();
    }

    auto GetOutputMax() const
    {
        return saturation.GetMax();
    }

    /**
     * @brief 重置控制器状态
     *
     */
    void ResetState()
    {
        last_input_  = 0;
        last_output_ = 0;
    }
};

template <typename T>
class D : public DiscreteTf<T>
{
private:
    T Kd, Kn, Ts;
    T input_coefficient_;
    T output_coefficient_;
    T last_input_;
    T last_output_;

    void UpdateCoefficient()
    {
        auto den            = 2 + Kn * Ts;
        input_coefficient_  = (2 * Kd * Kn) / den;
        output_coefficient_ = (2 - Kn * Ts) / den;
    }

public:
    D(T Kd, T Kn, T Ts)
    {
        ResetState();
        SetParam(Kd, Kn, Ts);
    }

    /**
     * @brief 走一个采样周期
     *
     * @param input 输入
     * @return T 输出
     */
    T Step(T input) override
    {
        last_output_ = input_coefficient_ * (input - last_input_) + output_coefficient_ * last_output_;
        last_input_  = input;
        return last_output_;
    }

    void SetParam(T Kd, T Kn, T Ts)
    {
        this->Kd = Kd;
        this->Kn = Kn;
        this->Ts = Ts;
        UpdateCoefficient();
    }

    void SetParam(T Kd, T Kn)
    {
        this->Kd = Kd;
        this->Kn = Kn;
        UpdateCoefficient();
    }

    T GetKd() const
    {
        return Kd;
    }

    T GetKn() const
    {
        return Kn;
    }

    T GetTs() const
    {
        return Ts;
    }

    /**
     * @brief 重置控制器状态
     *
     */
    void ResetState()
    {
        last_input_  = 0;
        last_output_ = 0;
    }
};

template <typename T>
class PID : public DiscreteTf<T>
{
public:
    P<T> p_controller;
    I<T> i_controller;
    D<T> d_controller;

    PID(T Kp, T Ki, T Kd, T Kn, T Ts)
        : p_controller{Kp}, i_controller{Ki, Ts}, d_controller{Kd, Kn, Ts} {};

    /**
     * @brief 走一个采样周期
     *
     * @param input 输入
     * @return T 输出
     */
    T Step(T input) override
    {
        return p_controller.Step(input) + i_controller.Step(input) + d_controller.Step(input);
    }

    void SetParam(T Kp, T Ki, T Kd, T Kn, T Ts)
    {
        p_controller.SetParam(Kp);
        i_controller.SetParam(Ki, Ts);
        d_controller.SetParam(Kd, Kn, Ts);
    }

    void SetParam(T Kp, T Ki, T Kd, T Kn)
    {
        p_controller.SetParam(Kp);
        i_controller.SetParam(Ki);
        d_controller.SetParam(Kd, Kn);
    }

    /**
     * @brief 重置控制器状态
     *
     */
    void ResetState()
    {
        p_controller.ResetState();
        i_controller.ResetState();
        d_controller.ResetState();
    }
};

template <typename T>
class PI : public DiscreteTf<T>
{
public:
    P<T> p_controller;
    I<T> i_controller;

    PI(T Kp, T Ki, T Ts)
        : p_controller{Kp}, i_controller{Ki, Ts} {};

    /**
     * @brief 走一个采样周期
     *
     * @param input 输入
     * @return T 输出
     */
    T Step(T input) override
    {
        return p_controller.Step(input) + i_controller.Step(input);
    }

    /**
     * @brief 重置控制器状态
     *
     */
    void ResetState()
    {
        p_controller.ResetState();
        i_controller.ResetState();
    }
};

template <typename T>
class PD : public DiscreteTf<T>
{
public:
    P<T> p_controller;
    D<T> d_controller;

    PD(T Kp, T Kd, T Kn, T Ts)
        : p_controller{Kp}, d_controller{Kd, Kn, Ts} {};

    /**
     * @brief 走一个采样周期
     *
     * @param input 输入
     * @return T 输出
     */
    T Step(T input) override
    {
        return p_controller.Step(input) + d_controller.Step(input);
    }

    /**
     * @brief 重置控制器状态
     *
     */
    void ResetState()
    {
        p_controller.ResetState();
        d_controller.ResetState();
    }
};

template <typename T>
class PID_AntiWindup : public PD<T>
{
private:
    T Kb, Ki;
    T last_integrate = 0;
    // T origin_output, limited_output;

public:
    using PD<T>::p_controller;
    using PD<T>::d_controller;
    I<T> i_controller;
    Saturation<T, T> output_saturation; // 输出限幅

    PID_AntiWindup(T Kp, T Ki, T Kd, T Kn, T Ts, T Kb, T output_min, T output_max)
        : PD<T>{Kp, Kd, Kn, Ts}, Kb{Kb}, Ki{Ki}, i_controller{1, Ts}, output_saturation{output_min, output_max}
    {
        ResetState();
    }

    /**
     * @brief 走一个采样周期
     *
     * @param input 输入
     * @return T 输出
     */
    T Step(T input) override
    {
        auto p         = p_controller.Step(input);
        auto d         = d_controller.Step(input);
        auto preSat    = last_integrate + p + d;
        auto postSat   = output_saturation(preSat);
        auto b         = postSat - preSat;
        auto int_input = input * Ki + b * Kb;
        last_integrate = i_controller.Step(int_input);
        return postSat;
    }

    /**
     * @brief 重置控制器状态
     *
     */
    void ResetState()
    {
        PD<T>::ResetState();
        i_controller.ResetState();
        // origin_output  = 0;
        // limited_output = 0;
    }
};

} // namespace pid

} // namespace control_system
