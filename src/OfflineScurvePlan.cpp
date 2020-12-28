#include <iostream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <fstream>
#include <cmath>

//对称S型规划
#include "OfflineScurvePlan.h"

STypeMotion::STypeMotion()
{
    this->SetSysMotionPara(-10, 10, -10, 10, -30, 30);
    this->SetCycle(0.004);
}

void STypeMotion::SetSysMotionPara(double vmin, double vmax, double amin, double amax, double jmin, double jmax)
{
    this->m_vmin = vmin;
    this->m_vmax = vmax;
    this->m_amin = amin;
    this->m_amax = amax;
    this->m_jmin = jmin;
    this->m_jmax = jmax;
}
bool STypeMotion::Plan(double q0, double q1, double v0, double v1, int& N)
{
    this->m_q0 = q0;
    this->m_q1 = q1;
    this->m_v0 = v0;
    this->m_v1 = v1;

    //考虑位移减少情况，参数转换
    Convert2OppositeCase();

    //当前参数满足最小位移要求
    if (ParaMinDisRequirement())
    {
        //假设Tv段存在
        bool TvExist = TvExistTimeParaCal();
        if (TvExist)
        {
            CalRealMotionPara();
            N = m_N;
            return true;
        }
        double gama = 1;
        //Tv段不存在
        double k = 1;
        while (gama > 0)
        {
            if (TvNotExistTimeParaCal(gama))
            {
                CalRealMotionPara();
                N = m_N;
                return true;
            }
            else
            {
                std::cout << "Reducing Acc" << std::endl;
                gama *= (1 - 0.000001 * k);
            }
        }
    }
    //当前参数不满足最小位移要求
    else
    {
        double gama = 1;
        //Tv段不存在
        double k = 1;
        while (gama > 0)
        {
            if (TvNotExistTimeParaCal(gama))
            {
                CalRealMotionPara();
                N = m_N;
                if (m_isError == -1)
                    return false;
                return true;
            }
            else
            {
                std::cout << "Reducing Acc" << std::endl;
                gama *= (1 - 0.000001 * k);
            }
        }
        return false;
    }
    return false;

}

bool STypeMotion::Plan2(double q0, double q1, double v0, double v1, int& N)
{
    this->m_q0 = q0;
    this->m_q1 = q1;
    this->m_v0 = v0;
    this->m_v1 = v1;

    //考虑位移减少情况，参数转换
    Convert2OppositeCase();

    //当前参数满足最小位移要求
    if (ParaMinDisRequirement())
    {
        //假设Tv段存在
        bool TvExist = TvExistTimeParaCal();
        if (TvExist)
        {
            CalRealMotionPara();
            N = m_N;
            return true;
        }
        double gama = 1;
        //Tv段不存在
        double k = 1;
        while (gama > 0)
        {
            if (TvNotExistTimeParaCal(gama))
            {
                CalRealMotionPara();
                N = m_N;
                return true;
            }
            else
            {
                std::cout << "Reducing Acc" << std::endl;
                gama *= (1 - 0.01 * k);
            }
        }
    }
    //当前参数不满足最小位移要求
    else
    {
        double gama = 1;
        //Tv段不存在
        double k = 1;
        while (gama > 0)
        {
            if (TvNotExistTimeParaCal(gama))
            {
                CalRealMotionPara();
                N = m_N;
                if (m_isError == -1)
                    return false;
                return true;
            }
            else
            {
                std::cout << "Reducing Acc" << std::endl;
                gama *= (1 - 0.01 * k);
            }
        }
        return false;
    }
    return false;

}

//Tv段存在时的时间参数计算
bool STypeMotion::TvExistTimeParaCal()
{
    //假设Tv存在，如果Tv不小于0，假设成立；反之，假设成功
    if ((m_vmax - m_v0) * m_jmax < m_amax * m_amax)
    {
        m_Tj1 = sqrt((m_vmax - m_v0) / m_jmax);
        m_Ta = 2 * m_Tj1;
    }
    else
    {
        m_Tj1 = m_amax / m_jmax;
        m_Ta = m_Tj1 + (m_vmax - m_v0) / m_amax;
    }

    if ((m_vmax - m_v1) * m_jmax < m_amax * m_amax)
    {
        m_Tj2 = sqrt((m_vmax - m_v1) / m_jmax);
        m_Td = 2 * m_Tj2;
    }
    else
    {
        m_Tj2 = m_amax / m_jmax;
        m_Td = m_Tj2 + (m_vmax - m_v1) / m_amax;
    }

    m_Tv = (m_q1 - m_q0) / m_vmax - 0.5 * m_Ta * (1 + m_v0 / m_vmax) - 0.5 * m_Td * (1 + m_v1 / m_vmax);

    if (m_Tv >= 0)
        return true;
    return false;
}



//Tv段不存在时的时间参数计算
bool STypeMotion::TvNotExistTimeParaCal(double gama)
{
    m_amax = gama * m_amax;
    m_Tv = 0;
    m_Tj1 = m_Tj2 = m_amax / m_jmax;
    std::cout<<m_amax<<std::endl;
    double delta = pow(m_amax, 4) / pow(m_jmax, 2) + 2 * (m_v0 * m_v0 + m_v1 * m_v1) + \
        m_amax * (4 * (m_q1 - m_q0) - 2 * (m_amax / m_jmax) * (m_v0 + m_v1));

    m_Ta = (m_amax * m_amax / m_jmax - 2 * m_v0 + sqrt(delta)) / (2 * m_amax);
    m_Td = (m_amax * m_amax / m_jmax - 2 * m_v1 + sqrt(delta)) / (2 * m_amax);

    if (m_Ta >= 2 * m_Tj1 && m_Td >= 2 * m_Tj2)
     {
        std::cout<<"匀加速段时间: "<<m_Ta-m_Tj1*2<<std::endl;
        return true;
     }   
    else
    {
        if (m_Ta < 0)
        {
            m_Ta = 0;
            m_Tj1 = 0;
            m_Td = 2 * ((m_q1 - m_q0) / (m_v1 + m_v0));

            if (m_jmax * (m_jmax * pow(m_q1 - m_q0, 2) + \
                pow(m_v1 + m_v0, 2) * (m_v1 - m_v0)) < 0)
            {
                m_isError = -1;
                return true;
            }
            m_Tj2 = (m_jmax * (m_q1 - m_q0) - sqrt(m_jmax * (m_jmax * pow(m_q1 - m_q0, 2) + \
                pow(m_v1 + m_v0, 2) * (m_v1 - m_v0)))) / (m_jmax * (m_v1 + m_v0));
            return true;
        }
        if (m_Td < 0)
        {
            m_Td = 0;
            m_Tj2 = 0;
            m_Ta = 2 * ((m_q1 - m_q0) / (m_v1 + m_v0));
            if (m_jmax * (m_jmax * pow(m_q1 - m_q0, 2) + \
                pow(m_v1 + m_v0, 2) * (m_v1 - m_v0)) < 0)
            {
                m_isError = -1;
                return true;
            }
            m_Tj1 = (m_jmax * (m_q1 - m_q0) - sqrt(m_jmax * (m_jmax * pow(m_q1 - m_q0, 2) + \
                pow(m_v1 + m_v0, 2) * (m_v1 - m_v0)))) / (m_jmax * (m_v1 + m_v0));
            return true;
        }

        return false;
    }
}

//Tv段不存在时的时间参数计算
// bool STypeMotion::TvNotExistTimeParaCal(double gama)
// {
//     m_amax = gama * m_amax;
//     m_Tv = 0;
//     m_Tj1 = m_Tj2 = m_amax / m_jmax;

//     double delta = pow(m_amax, 4) / pow(m_jmax, 2) + 2 * (m_v0 * m_v0 + m_v1 * m_v1) + \
//         m_amax * (4 * (m_q1 - m_q0) - 2 * (m_amax / m_jmax) * (m_v0 + m_v1));

//     m_Ta = (m_amax * m_amax / m_jmax - 2 * m_v0 + sqrt(delta)) / (2 * m_amax);
//     m_Td = (m_amax * m_amax / m_jmax - 2 * m_v1 + sqrt(delta)) / (2 * m_amax);

//     if (m_Ta >= 2 * m_Tj1 && m_Td >= 2 * m_Tj2)
//         return true;
//     else
//     {
//         if (m_Ta < 0)
//         {
//             m_Ta = 0;
//             m_Tj1 = 0;
//             m_Td = 2 * ((m_q1 - m_q0) / (m_v1 + m_v0));

//             if (m_jmax * (m_jmax * pow(m_q1 - m_q0, 2) + \
//                 pow(m_v1 + m_v0, 2) * (m_v1 - m_v0)) < 0)
//             {
//                 m_isError = -1;
//                 return true;
//             }
//             m_Tj2 = (m_jmax * (m_q1 - m_q0) - sqrt(m_jmax * (m_jmax * pow(m_q1 - m_q0, 2) + \
//                 pow(m_v1 + m_v0, 2) * (m_v1 - m_v0)))) / (m_jmax * (m_v1 + m_v0));
//             return true;
//         }
//         if (m_Td < 0)
//         {
//             m_Td = 0;
//             m_Tj2 = 0;
//             m_Ta = 2 * ((m_q1 - m_q0) / (m_v1 + m_v0));
//             if (m_jmax * (m_jmax * pow(m_q1 - m_q0, 2) + \
//                 pow(m_v1 + m_v0, 2) * (m_v1 - m_v0)) < 0)
//             {
//                 m_isError = -1;
//                 return true;
//             }
//             m_Tj1 = (m_jmax * (m_q1 - m_q0) - sqrt(m_jmax * (m_jmax * pow(m_q1 - m_q0, 2) + \
//                 pow(m_v1 + m_v0, 2) * (m_v1 - m_v0)))) / (m_jmax * (m_v1 + m_v0));
//             return true;
//         }

//         return false;
//     }
// }

void STypeMotion::SetCycle(double cycle)
{
    this->m_cycle = cycle;
}


bool STypeMotion::ParaMinDisRequirement()
{
    double Tjstar1 = sqrt(fabs(m_v1 - m_v0) / m_jmax);
    double Tjstar2 = m_amax / m_jmax;

    double Tjstar = std::min(Tjstar1, Tjstar2);

    double minq0q1;
    if (Tjstar == Tjstar1)
    {
        minq0q1 = Tjstar * (m_v0 + m_v1);
    }
    else
    {
        minq0q1 = 0.5 * (m_v0 + m_v1) * (Tjstar + fabs(m_v1 - m_v0) * m_amax);
    }

    if (m_q1 - m_q0 > minq0q1)
    {
        return true;
    }

    return false;

}

void STypeMotion::Convert2OppositeCase()
{
    if (m_q1 > m_q0)
        m_sign = 1;
    else
        m_sign = -1;
    m_q0 = m_sign * m_q0;
    m_q1 = m_sign * m_q1;
    m_v0 = m_sign * m_v0;
    m_v1 = m_sign * m_v1;

    double k1 = (m_sign + 1) / 2;
    double k2 = (m_sign - 1) / 2;

    m_vmax = k1 * m_vmax + k2 * m_vmin;
    m_vmin = k1 * m_vmin + k2 * m_vmax;
    m_amax = k1 * m_amax + k2 * m_amin;
    m_amin = k1 * m_amin + k2 * m_amax;
    m_jmax = k1 * m_jmax + k2 * m_jmin;
    m_jmin = k1 * m_jmin + k2 * m_jmax;

}

void STypeMotion::CalRealMotionPara()
{
    this->m_alima = m_jmax * m_Tj1;
    this->m_alimd = -m_jmax * m_Tj2;

    m_vlima = m_v0 + (m_Ta - m_Tj1) * m_alima;

    m_vlimd = m_v1 - (m_Td - m_Tj2) * m_alimd;

    if (fabs(m_vlima) > fabs(m_vlimd))
        m_vlim = fabs(m_vlima);
    else
        m_vlim = fabs(m_vlimd);

    //总时间的计算
    m_T = m_Ta + m_Td + m_Tv;

    m_N = static_cast<int>(m_T / m_cycle + 1);

}


void STypeMotion::ShowKeyTime()
{
    std::cout << "[" << 0 << "," << m_Tj1 << "," << m_Ta - m_Tj1 << "," << \
        m_Ta << "," << m_Ta + m_Tv << "," << m_T - m_Td + m_Tj2 << "," << \
        m_T << "]" << std::endl;
}

bool STypeMotion::GetQi(double ti, double& qi)
{
    if (ti >= 0 && ti < m_Tj1)//AP1
    {
        qi = m_q0 + m_v0 * ti + m_jmax * pow(ti, 3) / 6;
    }
    else if (ti >= m_Tj1 && ti < m_Ta - m_Tj1)//AP2
    {
        qi = m_q0 + m_v0 * ti + (m_alima / 6) * (3 * ti * ti - \
            3 * m_Tj1 * ti + m_Tj1 * m_Tj1);
    }
    else if (ti >= m_Ta - m_Tj1 && ti < m_Ta)//AP3
    {
        qi = m_q0 + (m_vlim + m_v0) * (m_Ta / 2) - m_vlim * (m_Ta - ti) - \
            m_jmin * pow(m_Ta - ti, 3) / 6;
    }
    else if (ti >= m_Ta && ti < m_Ta + m_Tv)//MP
    {
        qi = m_q0 + (m_vlim + m_v0) * (m_Ta / 2) + m_vlim * (ti - m_Ta);
    }
    else if ((ti >= m_T - m_Td || ti >= m_Ta + m_Tv) && ti < m_T - m_Td + m_Tj2)//DP1
    {
        //“或语句”避免精度引起的超区间误判
        qi = m_q1 - (m_vlim + m_v1) * (m_Td / 2) + m_vlim * (ti - m_T + m_Td) \
            - m_jmax * pow(ti - m_T + m_Td, 3) / 6;
    }
    else if (ti >= m_T - m_Td + m_Tj2 && ti < m_T - m_Tj2)//DP2
    {
        qi = m_q1 - (m_vlim + m_v1) * (m_Td / 2) + m_vlim * (ti - m_T + m_Td) + \
            (m_alimd / 6) * (3 * pow(ti - m_T + m_Td, 2) - 3 * m_Tj2 * (ti - m_T + m_Td) + pow(m_Tj2, 2));
    }
    else if (ti >= m_T - m_Tj2 && ti <= m_T)//DP3
    {
        qi = m_q1 - m_v1 * (m_T - ti) - m_jmax * pow(m_T - ti, 3) / 6;
    }
    else
    {
        return false;
    }
    return true;
}

bool STypeMotion::GetModifiedPara(double &plan_amax,double &plan_amin,double &plan_vel,double &plan_duration)
{
    plan_amax=this->m_alima;
    plan_amin=this->m_alimd;

    plan_vel=m_sign*this->m_vlim;

    plan_duration=this->m_T;
}

bool STypeMotion::Move(int i, double& qi)
{
    double ti = m_cycle * i;

    if (ti > m_T)
        ti = m_T;
    if (!GetQi(ti, qi))
    {
        return false;
    }

    qi *= m_sign;
    return true;
}

