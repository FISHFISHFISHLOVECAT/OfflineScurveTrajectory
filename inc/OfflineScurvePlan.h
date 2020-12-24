class STypeMotion
{
public:
    STypeMotion();
    bool Plan(double q0, double q1, double v0, double v1, int& N);
    bool Move(int i, double& qi);
    void SetSysMotionPara(double vmin, double vmax, double amin, double amax, double jmin, double jmax);
    void SetCycle(double cycle);
    //显示关键时间节点
    void ShowKeyTime();

private:
    //S型规划最低要求
    bool ParaMinDisRequirement();

    //Tv段不存在时的时间参数计算
    bool TvNotExistTimeParaCal(double gama);

    //Tv段存在时的时间参数计算
    bool TvExistTimeParaCal();

    //增加对位移减少情况的处理
    void Convert2OppositeCase();

    //计算实际运行参数
    void CalRealMotionPara();

    //获取ti的输出值Qi
    bool GetQi(double ti, double& qi);

    //系统内置参数
    double m_vmin = 0, m_vmax = 0;
    double m_amin = 0, m_amax = 0;
    double m_jmin = 0, m_jmax = 0;
    double m_cycle;

    //用户参数
    double m_q0;
    double m_q1;
    double m_v0;
    double m_v1;

    //根据用户参数调整的系统参数
    double m_alima = 0;//real acc
    double m_alimd = 0;//real dec
    double m_vlima = 0;
    double m_vlimd = 0;
    double m_vlim = 0;//real_vlim

    //规划的时间参数
    double m_Tj1 = 0;
    double m_Tj2 = 0;
    double m_Tv = 0;
    double m_Ta = 0;
    double m_Td = 0;
    double m_T = 0;

    //是否为q0>q1
    int m_sign = 1;
    int m_N = 0;
    int m_isError = 0;

};