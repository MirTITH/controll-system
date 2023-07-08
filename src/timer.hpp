#pragma once
#include <chrono>

class Timer
{
private:
    decltype(std::chrono::steady_clock().now()) start_point_;

public:
    Timer()
    {
        Start();
    }

    void Start()
    {
        start_point_ = std::chrono::steady_clock().now();
    }

    double GetSecond()
    {
        return static_cast<std::chrono::duration<double>>(std::chrono::steady_clock().now() - start_point_).count();
    }
};
