#ifndef CCOMMANDINFO_H
#define CCOMMANDINFO_H

//#include "define.h"
#define byte unsigned char

class CCommandInfo
{
public:
    CCommandInfo();


public:

    bool m_commandflag;
    byte m_commandtype;
    byte m_commandid;
    byte m_commandbuf[50];
    byte m_commandlen;
    byte sendseq;
    byte sendtimes;
};

#endif // CCOMMANDINFO_H
