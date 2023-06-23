#include <iostream>
#include <thread>
#include "control_system.hpp"
#include <chrono>

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

double Step1(double input)
{
    static double inputs[3], outputs[3];
    inputs[2] = inputs[1];
    inputs[1] = inputs[0];
    inputs[0] = input;

    outputs[2] = outputs[1];
    outputs[1] = outputs[0];

    outputs[0] = 66.2117333333333 * inputs[0] - 124.136000000000 * inputs[1] + 58.1856000000000 * inputs[2] + 0.333333333333333 * outputs[1] + 0.666666666666667 * outputs[2];
    return outputs[0];
}

int main(int, char const **)
{
    RunTimeStatus run_time_status;
    DiscreteTf<double> dtf({66.2117333333333, -124.136000000000, 58.1856000000000,87}, {1, -0.333333333333333, -0.666666666666667,-0.5});

    for (size_t i = 0; i < 10; i++)
    {
        cout << Step1(1) << "   " << dtf.Step(1) << endl;
    }

    run_time_status.Start();
    for (size_t i = 0; i < 1000000; i++)
    {
        dtf.Step(1);
        // Step1(1);
    }
    run_time_status.End();

    run_time_status.Start();
    for (size_t i = 0; i < 1000000; i++)
    {
        // dtf.Step(1);
        Step1(1);
    }
    run_time_status.End();

    return 0;
}
