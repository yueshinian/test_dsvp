#ifndef CMOTORINFO_H
#define CMOTORINFO_H

#define byte unsigned char

class CMotorInfo 
{
public:
     CMotorInfo();

    float m_curpos;
    byte m_state;
    byte m_errorcode;
    byte m_ipval;
    byte m_rfinishflag;

    bool m_calposflag;
    float calcpos;
    float zeroval;
    float minval;   //最大角度值
    float maxval;   //最小角度值
    float maxcval;
    float mincval;
    float willtopos;
    float minrealval;   //最小值
    float maxrealval;
    float perdistance;
    float perpulse;

};

#endif // CMOTORINFO_H
