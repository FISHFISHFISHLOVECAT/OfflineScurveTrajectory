#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <vector>
#include <cmath>
#include <chrono>
#include "OfflineScurvePlan.h"

int main()
{
    OfflineScurvePlan S;

    //系统参数
    double vmax = 50, amax = 3000, jmax = 30000;
    S.SetSysMotionPara(vmax, amax, jmax);
    S.SetCycle(0.004);

    //用户参数
    double q0 = 0, q1 = 30, v0 = 0, v1 = 0;
    auto start=std::chrono::steady_clock::now();
    bool plan_ok = S.Plan(q0, q1, v0, v1);
    auto end=std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds=end-start;
    std::cout << "elapsed time: " << elapsed_seconds.count()*1000 << "ms\n";
    
    std::ofstream of("/mnt/hgfs/Data/result.txt");

    double qi = 0;//插补点实际位移
    double qtmp;
    if (plan_ok)
    {
        for (int i = 0; i < S.size(); i++)
        {
            S.Move(i, qi);
            of << std::setprecision(15) << qi << std::endl;
        }
    } 
    else
    {
        std::cout << "Segment1 Plan Failed" << std::endl;
    }


}