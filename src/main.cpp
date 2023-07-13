#include <stdio.h>
#include <stdint.h>
#include "control_system/pid_controller.hpp"
#include "control_system/saturation.hpp"
#include "control_system/z_tf.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include "timer.hpp"

using namespace control_system;
using namespace std;

template <typename T>
void StepTest(DiscreteControllerBase<T> &controller, uint32_t loop_time = 10000000)
{
    Timer timer;
    timer.Start();
    for (size_t i = 0; i < loop_time; i++) {
        controller.Step(1);
    }
    auto duration = timer.GetSecond();
    auto speed    = loop_time / duration / 1000.0;
    printf("Step(1) for %u times: duration: %g s, speed: %g kps\n", loop_time, duration, speed);

    printf("Step(1) from %u to %u times:\n", loop_time + 1, loop_time + 11);
    for (size_t i = 0; i < 10; i++) {
        printf("%g\t", controller.Step(1));
    }

    printf("\nStep(1) from tick 1 to 10:\n");
    controller.ResetState();
    for (size_t i = 0; i < 10; i++) {
        printf("%g\t", controller.Step(1));
    }

    printf("\nThen Step(-1) from tick 11 to 20:\n");
    for (size_t i = 0; i < 10; i++) {
        printf("%g\t", controller.Step(-1));
    }
    printf("\n");
}

int main(int, char **)
{
    // 定义一个离散传递函数
    // 其中内部运算数据类型为 float
    // 分子为 66 z^2 - 124 z + 58
    // 分母为 z^2 - 0.333 z - 0.667
    ZTf<double> ztf({1, 2}, {1, 2, 3, 4, 5});

    printf("==== Old ztf: ====\n");
    StepTest(ztf);

    return 0;
}
