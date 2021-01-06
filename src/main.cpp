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
    double vmax = 300, amax = 3000, jmax = 30000;
    S.SetSysMotionPara(-vmax, vmax, -amax, amax, -jmax, jmax);
    double cycle = 0.004;
    S.SetCycle(cycle);

    //用户参数
    int N;        //实际插补点数
    double q0 = 0, q1 = 12.5, v0 = 0, v1 = 90;
    auto start=std::chrono::steady_clock::now();
    bool plan_ok = S.Plan(q0, q1, v0, 90, N);
    auto end=std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds=end-start;
    std::cout << "elapsed time: " << elapsed_seconds.count()*1000 << "ms\n";
    
    std::ofstream of("/mnt/hgfs/Data/result.txt");

    double qi = 0;//插补点实际位移
    double qtmp;
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
        std::cout << "Segment1 Plan Failed" << std::endl;
    }

    plan_ok = S.Plan(qi, q1+10, 90, 90, N);
    if (plan_ok)
    {
        for (int i = 1; i < N; i++)
        {
            S.Move(i, qi);
            of << std::setprecision(15) << qi << std::endl;
        }
    } 
    else
    {
        std::cout << "Segment2 Plan Failed" << std::endl;
    }

    plan_ok = S.Plan(qi, q1+20, 90, 90, N);
    if (plan_ok)
    {
        for (int i = 1; i < N; i++)
        {
            S.Move(i, qi);
            of << std::setprecision(15) << qi << std::endl;
        }
    } 
    else
    {
        std::cout << "Segment3 Plan Failed" << std::endl;
    }

   


    // plan_ok = S.Plan(qi, q1, v0, v1, N);

    // if (plan_ok)
    // {
    //     for (int i = 1; i < N; i++)
    //     {
            
    //         S.Move(i, qi);
    //         of << std::setprecision(15) << qi << std::endl;
    //     }
    // } 
    // else
    // {
    //     std::cout << "Segment3 Plan Failed" << std::endl;
    // }

    // plan_ok = S.Plan(qi, q0+50, v1, v0, N);

    // if (plan_ok)
    // {
    //     for (int i = 1; i < N; i++)
    //     {
            
    //         S.Move(i, qi);
    //         of << std::setprecision(15) << qi << std::endl;
    //     }
    // } 
    // else
    // {
    //     std::cout << "Segment4 Plan Failed" << std::endl;
    // }

    // plan_ok = S.Plan(qi, q1, v0, v1, N);

    // if (plan_ok)
    // {
    //     for (int i = 1; i < N; i++)
    //     {
            
    //         S.Move(i, qi);
    //         of << std::setprecision(15) << qi << std::endl;
    //     }
    // } 
    // else
    // {
    //     std::cout << "Segment5 Plan Failed" << std::endl;
    // }


}