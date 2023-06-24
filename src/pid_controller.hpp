#pragma once

#include "discrete_tf.hpp"
#include "z_tf.hpp"
#include <array>
#include <cmath>

template <typename T>
class Pidn : public DiscreteTf<T>
{
protected:
    T Kp, Ki, Kd, Kn, Ts;
    std::array<T, 3> input_coefficient_;
    std::array<T, 2> output_coefficient_;

    std::array<T, 3> inputs_, outputs_;

    void UpdateCoefficient()
    {
        input_coefficient_.at(0)  = Kp + (Ki * Ts) / 2 + (2 * Kd * Kn) / (2 + Kn * Ts);
        input_coefficient_.at(1)  = (-4 * (Kd * Kn + Kp) + Ki * Kn * Ts * Ts) / (2 + Kn * Ts);
        input_coefficient_.at(2)  = (4 * Kd * Kn - (2 * Kp - Ki * Ts) * (-2 + Kn * Ts)) / (4 + 2 * Kn * Ts);
        output_coefficient_.at(0) = 4 / (2 + Kn * Ts);
        output_coefficient_.at(1) = 1 - 4 / (2 + Kn * Ts);
    }

public:
    Pidn(T Kp, T Ki, T Kd, T Kn, T Ts)
        : Kp(Kp), Ki(Ki), Kd(Kd), Kn(Kn), Ts(Ts)
    {
        UpdateCoefficient();
        ResetState();
    }

    void SetParam(T Kp, T Ki, T Kd, T Kn, T Ts)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
        this->Kn = Kn;
        this->Ts = Ts;
        UpdateCoefficient();
    }

    void ResetState() override
    {
        std::fill(inputs_.begin(), inputs_.end(), 0);
        std::fill(outputs_.begin(), outputs_.end(), 0);
    }

    T Step(T input) override
    {
        inputs_[2]  = inputs_[1];
        inputs_[1]  = inputs_[0];
        inputs_[0]  = input;
        outputs_[2] = outputs_[1];
        outputs_[1] = outputs_[0];

        outputs_[0] = input_coefficient_[0] * inputs_[0] + input_coefficient_[1] * inputs_[1] + input_coefficient_[2] * inputs_[2] + output_coefficient_[0] * outputs_[1] + output_coefficient_[1] * outputs_[2];
        return outputs_[0];
    }
};

template <typename T>
class PidnAntiWindup : public Pidn<T>
{
private:
    T max_output, min_output;

    T Saturation(T input)
    {
        if (input > max_output) {
            return max_output;
        }

        if (input < min_output) {
            return min_output;
        }

        return input;
    }

public:
    PidnAntiWindup(T Kp, T Ki, T Kd, T Kn, T Ts, T max_output, T min_output)
        : Pidn<T>(Kp, Ki, Kd, Kn, Ts), max_output(max_output), min_output(min_output){};

    void SetParam(T Kp, T Ki, T Kd, T Kn, T Ts, T max_output, T min_output)
    {
        this->Kp         = Kp;
        this->Ki         = Ki;
        this->Kd         = Kd;
        this->Kn         = Kn;
        this->Ts         = Ts;
        this->max_output = max_output;
        this->min_output = min_output;
        Pidn<T>::UpdateCoefficient();
    }

    T Step(T input) override
    {
        Pidn<T>::Step(input);
        this->outputs_[0] = Saturation(this->outputs_[0]);
        return this->outputs_[0];
    }
};

template <typename T>
class Pdn : public DiscreteTf<T>
{
protected:
    T Kp, Kd, Kn, Ts;
    std::array<T, 2> input_coefficient_;
    T output_coefficient_;

    std::array<T, 2> inputs_;
    T output_;

    void UpdateCoefficient()
    {
        input_coefficient_.at(0) = (2 * Kd * Kn + 2 * Kp + Kn * Kp * Ts) / (2 + Kn * Ts);
        input_coefficient_.at(1) = (-2 * Kd * Kn + Kp * (-2 + Kn * Ts)) / (2 + Kn * Ts);
        output_coefficient_      = (2 - Kn * Ts) / (2 + Kn * Ts);
    }

public:
    Pdn(T Kp, T Kd, T Kn, T Ts)
        : Kp(Kp), Kd(Kd), Kn(Kn), Ts(Ts)
    {
        UpdateCoefficient();
        ResetState();
    }

    void SetParam(T Kp, T Ki, T Kd, T Kn, T Ts)
    {
        this->Kp = Kp;
        this->Kd = Kd;
        this->Kn = Kn;
        this->Ts = Ts;
        UpdateCoefficient();
    }

    void ResetState() override
    {
        std::fill(inputs_.begin(), inputs_.end(), 0);
        output_ = 0;
    }

    T Step(T input) override
    {
        inputs_[1] = inputs_[0];
        inputs_[0] = input;

        output_ = input_coefficient_[0] * inputs_[0] + input_coefficient_[1] * inputs_[1] + output_coefficient_ * output_;
        return output_;
    }
};
