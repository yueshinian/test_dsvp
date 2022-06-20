#include "cmotorinfo.h"

CMotorInfo::CMotorInfo()
{
    m_curpos = 0;
    m_errorcode = 0;
    m_state = 0;
    m_ipval = 0;
    m_rfinishflag = 0;
    
    m_calposflag = false;
    calcpos = 0;
    zeroval = 0;
    minval = 0;   //最大角度值
    maxval = 0;   //最小角度值
    maxcval = 0;
    mincval = 0;
    willtopos = 0;
    minrealval = 0;   //最小值
    maxrealval = 0;

}
