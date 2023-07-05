#include <stdio.h>
#include <stdint.h>
#include "control_system/pid_controller.hpp"
#include "control_system/saturation.hpp"

#define os_printf printf

using namespace control_system;

template <typename T>
void PidStepTest(DiscreteTf<T> &controller, const uint32_t loop_time = 200000)
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

    os_printf("\n\nStep(1) from tick 1 to 10:\n");
    // controller.ResetState();
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
    os_printf("\npid_controller:\n");
    pid::PID_AntiWindup<double> pid_controller{2, 100, 0, 100, 0.01, 50, -5, 5};
    // pid_controller.i_controller.SetOutputMinMax(-5, 5);
    PidStepTest(pid_controller);
    return 0;
}
