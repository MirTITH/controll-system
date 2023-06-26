#include <iostream>
#include <thread>
#include "discrete_tf.hpp"
#include "z_tf.hpp"
#include <chrono>
#include "pid_controller.hpp"
#include <deque>
#include <queue>
#include "ring_queue.hpp"

using namespace std;

class RunTimeStatus
{
private:
    decltype(chrono::steady_clock::now()) startTime;

public:
    RunTimeStatus()
    {
        startTime = chrono::steady_clock::now();
    }

    void Start()
    {
        startTime = chrono::steady_clock::now();
    }

    void End(const string &info = "Time: ")
    {
        chrono::duration<double, milli> elapsedTime = chrono::steady_clock::now() - startTime;
        cout << info << elapsedTime.count() << " ms" << endl;
    }
};

static double inputs[3], outputs[3];

void FastStepReset()
{
    for (size_t i = 0; i < 3; i++) {
        inputs[i]  = 0;
        outputs[i] = 0;
    }
}

double FastStep(double input)
{
    inputs[2] = inputs[1];
    inputs[1] = inputs[0];
    inputs[0] = input;

    outputs[2] = outputs[1];
    outputs[1] = outputs[0];

    outputs[0] = 66.2117333333333 * inputs[0] - 124.136000000000 * inputs[1] + 58.1856000000000 * inputs[2] + 0.333333333333333 * outputs[1] + 0.666666666666667 * outputs[2];
    return outputs[0];
}

void FastStepTest()
{
    // RunTimeStatus timer;
    // timer.Start();
    // for (size_t i = 0; i < 100000000; i++) {
    //     FastStep(1);
    // }
    // timer.End("FastStep: ");
    // cout << FastStep(1) << endl
    //      << endl;
    // FastStepReset();
    for (size_t i = 0; i < 10; i++) {
        cout << FastStep(1) << endl;
    }
    cout << endl;
}

template <typename T>
void TfSpeedTest(T &tf, const string &info = "Time: ")
{
    RunTimeStatus timer;
    timer.Start();
    for (size_t i = 0; i < 10000000; i++) {
        tf.Step(1);
    }
    timer.End(info);
    cout << tf.Step(1) << endl
         << endl;

    tf.ResetState();
    for (size_t i = 0; i < 10; i++) {
        cout << tf.Step(1) << endl;
    }
    // for (size_t i = 0; i < 50; i++) {
    //     cout << tf.Step(-1) << endl;
    // }
    cout << endl;
}

// template <typename T>
// void TfSpeedTest(ZTf<T> &tf, const string &info = "Time: ")
// {
//     RunTimeStatus timer;
//     timer.Start();
//     for (size_t i = 0; i < 100000000; i++) {
//         tf.Step(1);
//     }
//     timer.End(info);
//     cout << tf.Step(1) << endl
//          << endl;
//     tf.ResetState();
//     for (size_t i = 0; i < 10; i++) {
//         cout << tf.Step(1) << endl;
//     }
//     cout << endl;
// }

int main(int, char const **)
{
    RunTimeStatus run_time_status;
    ZTf<double> ztf({66.2117333333333, -124.136000000000, 58.1856000000000},
                    {1, -0.333333333333333, -0.666666666666667});

    ZTf1<double> ztf1({66.2117333333333, -124.136000000000, 58.1856000000000},
                                   {1, -0.333333333333333, -0.666666666666667});


    // PidnAntiWindup<float> pidna(1, 2, 0, 100, 0.1, 3, -3);
    // Pidn<float> pidn(1, 2, 0, 100, 0.1);
    // Pdn<float> pdn(1, 2, 100, 0.1);

    // // FastStep
    FastStepTest();

    // // PIDN
    // TfSpeedTest(pidn, "pidn: ");


    TfSpeedTest(ztf, "ztf: ");
    TfSpeedTest(ztf1, "ztf1: ");

    // TfSpeedTest(pdn, "pdn: ");
    // TfSpeedTest(pidn, "pidn: ");
    // TfSpeedTest(pidna, "pidna: ");

    return 0;
}
