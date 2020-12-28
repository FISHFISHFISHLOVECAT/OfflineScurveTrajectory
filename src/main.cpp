#include <iostream>
#include <fstream>
#include <iomanip>
#include "OfflineScurvePlan.h"

int main()
{

    STypeMotion S;

    //系统参数
    double vmax = 300, amax = 3000, jmax = 30000;
    S.SetSysMotionPara(-vmax, vmax, -amax, amax, -jmax, jmax);
    double cycle = 0.004;
    S.SetCycle(cycle);

    //用户参数
    int N;        //实际插补点数
    double q0 = 10, q1 = 0, v0 = 2, v1 = 1;
    bool plan_ok = S.Plan(q0, q1, v0, v1, N);
    
    std::ofstream of("/mnt/hgfs/Data/result.txt");

    double qi = 0;//插补点实际位移
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