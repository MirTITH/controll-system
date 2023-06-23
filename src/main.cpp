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
    DiscreteTf<double> dtf({4, 2, 3, 5}, {2, 5, 6, 7, 4, 6, 8});

    run_time_status.Start();
    for (size_t i = 0; i < 100000; i++)
    {
        dtf.Step(1);
    }
    run_time_status.End();

    return 0;
}
