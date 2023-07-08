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
    ZTf<double> ztf({66.2117333333333, -124.136000000000},
                    {1, -0.333333333333333, -0.666666666666667});

    printf("==== Old ztf: ====\n");
    StepTest(ztf);

    new_design::ZTf<double> newztf({66.2117333333333, -124.136000000000},
                                   {1, -0.333333333333333, -0.666666666666667});
    printf("==== New ztf: ====\n");
    StepTest(newztf);

    // new_design2::ZTf<double> newztf2({66.2117333333333, -124.136000000000},
    //                                {1, -0.333333333333333, -0.666666666666667});
    // printf("==== New ztf2: ====\n");
    // StepTest(newztf2);

    // forward_list<float> flist{1.0, 2.1, 3.22, 5.43};

    // auto iter = flist.cbegin();
    // while (std::next(iter, 2) != flist.cend()) {
    //     ++iter;
    // }

    // cout << " *iter: " << *iter << endl;
    // flist.splice_after(flist.cbefore_begin(), flist, iter);
    // cout << " *iter: " << *iter << endl;

    // for (auto var : flist) {
    //     cout << var << endl;
    // }

    return 0;
}
