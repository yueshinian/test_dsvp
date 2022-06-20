#ifndef CAGVINFO_H
#define CAGVINFO_H

//#include <qhostaddress.h>
#include "cmotorinfo.h"
#include "ccommandinfo.h"
#include <string.h>

class CAgvInfo
{
public:
    CAgvInfo();

    void SendData(); //发送udp数据

public:
    int m_agvid;
    int m_willtox;
    int m_willtoy;
//    int m_curx;
//    int m_cury;

    int m_senderrorcount;
//    int m_row;          //小车当前位置信息
//    int m_col;
//    int startline;
//    int endline;
    bool m_online;

    byte mobotResetFlag;


    float m_voltval;
    float m_voltratio;
    int m_mainstate;//0-没有连接，5-已连接，没有位置信息   200-空闲状态

    unsigned char m_lasterrorid;
    unsigned char m_sendbuf[300];
    unsigned char m_commandbuf[50];
    unsigned short m_advals[10];

    unsigned char m_autostate;
    //CNodeInfo m_walkarray[5];   //送书队列
    int m_walkarraycount;
    int m_curwalkseq;

    bool m_chargeErrorFlag;


    //行走相关
    bool m_walkingflag;     //
    unsigned char m_walkstate;
    unsigned char m_wproccount;

    unsigned char m_wnodecount;
    unsigned char m_curwnodeseq;
    bool m_lockflag;

    float m_agvsjyoutpos;   //小车书架出书位X
    float m_agvsjxoutpos;   //小车书架出书位Y
    float m_agvsjoutspeed;  //小车书架出书速度
    //QTimer *m_ptimer;

    void SwitchWalkState(unsigned char st);
    void UpdateWalkState();

    CCommandInfo coms[256];
    unsigned char comhead;
    unsigned char comtail;
    unsigned char count1;
    unsigned char count2;
    int m_proccount;

    bool m_rbflag;      //还书标志
    int m_rbstate;  //0-没有工作，10-启动检查，20-走位到书架，30-还书循环，直到队到完成
    int m_rbproccount;
    int m_rbbookids[5];
    bool m_reachflag1[5];

    bool m_lbflag;      //借书标志
    int m_lbstate;  //0-没有工作，10-启动检查，20-走位到书架，30-还书循环，直到队到完成
    int m_lbproccount;
    int m_lbbookids[5];
    bool m_reachflag2[5];
    bool m_givebookfininshflag;
    int m_templen;

    bool m_initingflag;
    int m_initstate;
    int m_initproccount;
    bool m_hasinitflag;

    int m_shujiaseq;

    bool m_getbookflag;
    int m_getbookstate;
    int m_getbookproccount;

    bool m_setbookflag;
    int m_setbookstate;
    int m_setbookproccount;

    bool m_setsgbookflag;
    int m_setsgbookstate;
    int m_setsgbookproccount;
    int m_setsgbookerrorcount;
    bool m_getsgbookflag;
    int m_getsgbookstate;
    int m_getsgbookproccount;

    bool m_gotochargeflag;
    bool m_stopchargeflag;

    unsigned char m_agvrgidval;

    //QHostAddress m_raddr; //这个可以直接用字符串代替
    char* m_raddr;

    CMotorInfo *m_pminfos;
    void EnableMotor(int mseq ,unsigned char v);
    void MotorStop(int mseq);
    void ReturnStartPos(int mseq);
    void DisableWalkMotor();
    void RunWithPosition(unsigned char mseq ,float v ,float pos);
    void RunWithSpeed(unsigned char mseq ,float v);
    void SetOutControlVal(unsigned char seq ,unsigned char val);
    void SetMotorLimitVal(int mseq ,float v);

    bool SetRowColInfo(int row ,int col);
    void SendCommand(unsigned char comtype ,unsigned char comid ,unsigned char comlen ,unsigned char *combuf);
    void UpdateState();
    void SwitchState(int st);




    float GetCurPos(unsigned char mseq);

    void SetComConfigInfo(int mseq);
    void GetComConfigInfo(int mseq);



private:
    int m_willtochargevoltcount;
};

#endif // CAGVINFO_H
