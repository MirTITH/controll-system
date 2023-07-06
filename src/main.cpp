#include <stdio.h>
#include <stdint.h>
#include "control_system/pid_controller.hpp"
#include "control_system/saturation.hpp"
#include "control_system/z_tf.hpp"
#include <iostream>

#define os_printf printf

using namespace control_system;

template <typename T>
void StepTest(DiscreteControllerBase<T> &controller, const uint32_t loop_time = 200000)
{
    // uint32_t start_time = HPT_GetUs();
    // for (size_t i = 0; i < loop_time; i++) {
    //     controller.Step(1);
    // }
    // auto duration = HPT_GetUs() - start_time;
    // auto speed    = (float)loop_time / duration * 1000.0f;
    // os_printf("Step(1) for %lu times: duration: %lu us, speed: %g kps\n", loop_time, duration, speed);

    // os_printf("Step(1) from %lu to %lu times:\n", loop_time + 1, loop_time + 11);
    // for (size_t i = 0; i < 10; i++) {
    //     os_printf("%g\t", controller.Step(1));
    // }

    os_printf("\nStep(1) from tick 1 to 10:\n");
    controller.ResetState();
    for (size_t i = 0; i < 10; i++) {
        os_printf("%g\t", controller.Step(1));
    }

    os_printf("\nThen Step(-1) from tick 11 to 20:\n");
    for (size_t i = 0; i < 10; i++) {
        os_printf("%g\t", controller.Step(-1));
    }
    os_printf("\n");
}

int main(int, char **)
{
    pid::PID<float> pid_controller{2, 100, 0.76, 100, 0.01};
    os_printf("pid_controller:\n");
    StepTest(pid_controller);

    // 设置积分限幅
    pid_controller.i_controller.saturation.SetMinMax(-1, 1);

    // 重新设置参数
    pid_controller.SetParam(1, 2, 0.5, 20, 0.1);

    // 单独设置微分项
    pid_controller.d_controller.SetParam(0.2, 20);

    // 重置内部状态（如积分器的积分量等）
    pid_controller.ResetState();

    os_printf("\npid_controller with integrator saturation:\n");
    StepTest(pid_controller);

    // 定义一个离散传递函数
    // 其中内部运算数据类型为 float
    // 分子为 66 z^2 - 124 z + 58
    // 分母为 z^2 - 0.333 z - 0.667
    ZTf<float> ztf({66, -124, 58}, {1, -0.333, -0.667});

    os_printf("\nDiscrete transfer function:\n");
    StepTest(ztf);

    return 0;
}
