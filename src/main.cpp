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
    q0 = 0, q1 = 10, v0 = 0, v1 = 0, vmax = 300, amax = 3000, jmax = 30000;
    S.SetSysMotionPara(-vmax, vmax, -amax, amax, -jmax, jmax);
    S.SetCycle(cycle);
    bool plan_ok = S.Plan(q0, q1, v0, v1, N);
    double modi_amax,modi_amin,modi_vel,modi_total_t;
    //S.GetModifiedPara(modi_amax,modi_amin,modi_vel,modi_total_t);
    //std::cout<<"modi_amax = "<<modi_amax<<"modi_amin = "<<modi_amin<<"modi_vel = "<<modi_vel<<"modi_total_t = "<<modi_total_t<<std::endl;
    
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