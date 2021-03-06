class OfflineScurvePlan
{
public:
    OfflineScurvePlan();
    bool Plan(double q0, double q1, double v0, double v1);
    bool Move(int i, double& qi);
    bool Move(int i,double &qi,double &vi);
    void SetSysMotionPara(double vmax,double amax, double jmax);

    int size(){return m_N;}

    void SetCycle(double cycle);



private:
    //显示关键时间节点
    void ShowKeyTime();

    bool Plan(double q0, double q1, double v0, double v1, int& N);

    void SetSysMotionPara(double vmin, double vmax, double amin, double amax, double jmin, double jmax);

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
    double m_cycle=0.004;

    //用户参数
    double m_q0=0,m_q1=0;
    double m_v0=0,m_v1=0;

    //根据用户参数调整的系统参数
    double m_alima=0 ,m_alimd = 0;
    double m_vlima = 0,m_vlimd = 0;
    double m_vlim = 0;//real_vlim

    //规划的时间参数
    double m_Tj1 = 0,m_Tj2 = 0;
    double m_Tv = 0;
    double m_Ta = 0,m_Td = 0;
    double m_T = 0;

    //是否为q0>q1
    int m_sign = 1;
    int m_N = 0;
    int m_isError = 0;

};