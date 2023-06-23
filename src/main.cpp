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

int main(int, char const **)
{
    RunTimeStatus run_time_status;
    DiscreteTf<double> dtf({66.2117333333333,-124.136000000000,58.1856000000000}, {1,-0.333333333333333,-0.666666666666667});

    run_time_status.Start();
    for (size_t i = 0; i < 1000000; i++)
    {
        dtf.Step(1);
    }
    run_time_status.End();

    return 0;
}
