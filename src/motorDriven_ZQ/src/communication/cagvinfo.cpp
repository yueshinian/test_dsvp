#include "cagvinfo.h"
#include "extern.h"
#include "math.h"
#include <stdio.h>

CAgvInfo::CAgvInfo()
{
    comhead = 0;
    comtail = 0;

    m_online = false;
    //m_commandflag = false;
    m_sendbuf[0] = 0xeb;
    m_sendbuf[1] = 0x90;
    m_sendbuf[2] = 0x91;

    m_senderrorcount = 0;
    m_pminfos = new CMotorInfo[15];
}

void CAgvInfo::EnableMotor(int mseq, unsigned char v)
{
    m_commandbuf[0] = mseq;
    m_commandbuf[1] = v;
    SendCommand(0x30 ,0x55 ,2 ,m_commandbuf);
}

void CAgvInfo::SetMotorLimitVal(int mseq, float v)
{
    m_commandbuf[0] = mseq;
     memcpy(m_commandbuf+1 ,&v ,4);
    SendCommand(0x30 ,0x35 ,5 ,m_commandbuf);
}

void CAgvInfo::SetComConfigInfo(int mseq)
{
    m_commandbuf[0] = mseq;
    memcpy(m_commandbuf+1 ,&(m_pminfos[mseq].perdistance) ,4);
    memcpy(m_commandbuf+5 ,&(m_pminfos[mseq].perpulse) ,4);

    SendCommand(0x30 ,0x37 ,9 ,m_commandbuf);
}

void CAgvInfo::GetComConfigInfo(int mseq)
{
    m_commandbuf[0] = mseq;

    SendCommand(0x30 ,0x12 ,1 ,m_commandbuf);
}
void CAgvInfo::SendData()
{
    //发送网络数据
    if(comhead != comtail)
    {
         //commandVal = 1;
        //发送命令
        m_sendbuf[3] = coms[comhead].m_commandid;
        m_sendbuf[4] = coms[comhead].sendseq;
        m_sendbuf[5] = coms[comhead].m_commandlen;
        memcpy(m_sendbuf+6 ,m_commandbuf ,coms[comhead].m_commandlen);

        g_protocol->SetCRC16(m_sendbuf ,coms[comhead].m_commandlen + 2 + 4);
        g_pudp ->SendData( m_sendbuf ,coms[comhead].m_commandlen + 2 + 4 + 2 ,m_raddr);
        //printf("command\n");
    }
    else
    {
        //发查询
//        m_sendbuf[3] = 0x10;//85
//        m_sendbuf[4] = 0x00;
//        m_sendbuf[5] = 0x01;
//        m_sendbuf[6] = 0x00;

//        g_protocol->SetCRC16(m_sendbuf ,7);

//        g_pudp ->SendData( m_sendbuf ,9 ,m_raddr);
//        m_senderrorcount++;
//        if(m_senderrorcount > 30)
//        {
//            m_senderrorcount = 0;
//            m_online = false;
//            //SwitchState(0);
//        }
    }
    /*for(int i = 0;m_sendbuf[i]!='\0';i++)
    printf("%x ", m_sendbuf[i]);
    printf("\n");*/

}


void CAgvInfo::MotorStop(int mseq)
{

    m_commandbuf[0] = mseq;
    m_commandbuf[1] = 0x01;
    SendCommand(0x30 ,0x50 ,2 ,m_commandbuf);
}

void CAgvInfo::ReturnStartPos(int mseq)
{
    m_commandbuf[0] = mseq;
    m_commandbuf[1] = 0x01;
    SendCommand(0x30 ,0x5f ,2 ,m_commandbuf);
}


void CAgvInfo::RunWithPosition(unsigned char mseq ,float v ,float pos)
{
    m_commandbuf[0] = mseq;
    memcpy(m_commandbuf+1 ,&v ,4);
    memcpy(m_commandbuf+5 ,&pos ,4);
    SendCommand(0x30 ,0x30 ,9 ,m_commandbuf);

    //    coms[comtail].m_commandtype = 0x30;
    //    coms[comtail].m_commandid = 0x30;
    //    coms[comtail].m_commandlen = 9;
    //    coms[comtail].m_commandbuf[0] = mseq;
    //    memcpy(coms[comtail].m_commandbuf+1 ,&v ,4);
    //    memcpy(coms[comtail].m_commandbuf+5 ,&pos ,4);
    //    coms[comtail].sendtimes = 0;
    //    coms[comtail].sendseq = comtail;
    //    comtail++;
}


void CAgvInfo::RunWithSpeed(unsigned char mseq ,float v)
{
    m_commandbuf[0] = mseq;
    memcpy(m_commandbuf+1 ,&v ,4);
    SendCommand(0x30 ,0x20 ,5 ,m_commandbuf);
}

void CAgvInfo::SendCommand(unsigned char comtype ,unsigned char comid ,unsigned char comlen ,unsigned char *combuf)
{
    coms[comtail].m_commandtype = comtype;
    coms[comtail].m_commandid = comid;
    coms[comtail].m_commandlen = comlen;
    if(comlen > 0)
    {
        memcpy(coms[comtail].m_commandbuf ,combuf ,comlen);
    }
    coms[comtail].sendtimes = 0;
    coms[comtail].sendseq = comtail;
    comtail++;
}

