#include <iostream>
#include <fstream>
#include <iomanip>
#include "OfflineScurvePlan.h"

int main()
{
    double q0;
    double q1;
    double v0;
    double v1;
    double vmax;
    double amax;
    double jmax;
    double cycle = 0.004;

    int N;
    STypeMotion S;
    q0 = 50, q1 = 58.3, v0 = 0, v1 = 0, vmax = 200, amax = 10000, jmax = 2000000;
    S.SetSysMotionPara(-vmax, vmax, -amax, amax, -jmax, jmax);
    S.SetCycle(cycle);
    bool plan_ok = S.Plan(q0, q1, v0, v1, N);
    double qi = 0;

    std::ofstream of("/mnt/hgfs/Data/result.txt");

    if (plan_ok)
    {
        for (int i = 0; i < N; i++)
        {
            S.Move(i, qi);
            of << std::setprecision(15) << qi << std::endl;
        }
    }
    else
    {
        std::cout << "Plan Failed" << std::endl;
    }
}