#include "extern.h"
#include "cprotocol.h"
#include <stdio.h>

CProtocol::CProtocol()
{
    m_recvbuf = new string[5];

}

void CProtocol::DealRecvData(char* buf ,int buffLen,char* addr)
{
    int seq = 0;
    m_recvbuf[0].append(buf,buffLen);
    unsigned char start = 0;
    unsigned char len = 0;
    //printf("recv 1\n");
    while(m_recvbuf[seq].length()-start >= 8)
    {
        //printf("recv 2\n");
        if(m_recvbuf[seq].at(start+2) != (char)0x81)
        {
             start++;
            continue;
        }
        if(m_recvbuf[seq].at(start+0)!= (char)0xeb || m_recvbuf[seq].at(start+1)!= (char)0x90)
        {
            start++;
            continue;
        }
        len = m_recvbuf[seq].at(start+5);
        if(start + len +8 > m_recvbuf[seq].length())
            break;
        //校验数据
        if(!CheckCRC16((unsigned char*)m_recvbuf[seq].data()+start ,len+6))
        {
            start += len+8;
            break;
        }
        string parsebuf = m_recvbuf[seq].substr(start+2 ,len+4);
        ParseRecvData((unsigned char*)parsebuf.data() ,seq);
        start += len+8;
    }
    if(start > 0)
    {
        string tempa;
        for(int i=0;i<m_recvbuf[seq].length()-start;i++)
        {
            tempa.append(1,m_recvbuf[seq].at(start+i));
        }
        m_recvbuf[seq].clear();
        m_recvbuf[seq] = tempa;
    }
}

float fratio = 0.95f;
float FilterData(float v, float oldval)
{
    float ov = fratio * oldval + (1 - fratio) * v;
    //oldval = ov;
    return ov;
}

unsigned short tempu16 = 0;
unsigned char tempu8 = 0;
float returnMotorValue85[13];
float returnMotorValue89[11];

void CProtocol::ParseRecvData(unsigned char* buf,int seq)
{
    if(g_agvinfos[seq].comhead != g_agvinfos[seq].comtail)
    {
        if(g_agvinfos[seq].coms[g_agvinfos[seq].comhead].m_commandid==buf[1])
        {
            g_agvinfos[seq].comhead++;
        }
    }
    //printf("recv 1\n");
    switch(buf[1])
    {
        case 0x10:
            //printf("recv 3\n");
            tempu8 = buf[4]; //当前轴个数，１１个
            for(int i=0;i<tempu8;i++)
            {
                memcpy(&g_agvinfos[seq].m_pminfos[i].m_curpos ,buf+5 + i*10 + 0 ,4);
            }
            break;
        case 0x85: //广播接收当前位置
            //printf("recv 4\n");
            tempu8 = buf[4]; //当前轴个数，１3个
            for(int i=0;i<tempu8;i++)
            {
                memcpy(&returnMotorValue85[i] ,buf+5 + i*10 + 0 ,4);
            }
            //对应88位置
            g_agvinfos[seq].m_pminfos[0].m_curpos = returnMotorValue85[6];
            g_agvinfos[seq].m_pminfos[1].m_curpos = returnMotorValue85[5];
            g_agvinfos[seq].m_pminfos[2].m_curpos = returnMotorValue85[4];
            g_agvinfos[seq].m_pminfos[3].m_curpos = returnMotorValue85[3];
            g_agvinfos[seq].m_pminfos[4].m_curpos = returnMotorValue85[2];
            g_agvinfos[seq].m_pminfos[5].m_curpos = returnMotorValue85[1];
            g_agvinfos[seq].m_pminfos[6].m_curpos = returnMotorValue85[0];
            g_agvinfos[seq].m_pminfos[7].m_curpos = returnMotorValue85[11];
            g_agvinfos[seq].m_pminfos[8].m_curpos = returnMotorValue85[12];
            g_agvinfos[seq].m_pminfos[9].m_curpos = returnMotorValue85[7];
            g_agvinfos[seq].m_pminfos[10].m_curpos = returnMotorValue85[9];

            g_agvinfos[seq].m_online = true;
            break;
        case 0x89: //回初始位置命令
            printf("reset return\n");
            break;
        case 0x88: //下发目标位置后返回当前位置
            //printf("recv 6\n");
            for(int i=0;i<11;i++) //11个轴
            {
                memcpy(&g_agvinfos[seq].m_pminfos[i].m_curpos ,buf+4 + i*4 ,4);
            }
            g_agvinfos[seq].mobotResetFlag = buf[48];
            break;

    }

}


uint16_t CProtocol::CRC16(unsigned char * pszBuf, uint16_t unLength)
{
    uint16_t i,j,CurVal;
    uint16_t CrcReg = 0xFFFF;

    for (i = 0; i < unLength; i++)
    {
        CurVal = pszBuf[i] << 8;

        for (j = 0; j < 8; j++)
        {
            if ((short)(CrcReg ^ CurVal) < 0)
                CrcReg = (CrcReg << 1) ^ 0x1021;
            else
                CrcReg <<= 1;
            CurVal <<= 1;
        }
    }
    return CrcReg;
}

bool CProtocol::CheckCRC16(unsigned char * pszBuf, uint16_t unLength)
{
    uint16_t i,j,CurVal;
    uint16_t CrcReg = 0xFFFF;

    for (i = 0; i < unLength; i++)
    {
        CurVal = pszBuf[i] << 8;

        for (j = 0; j < 8; j++)
        {
            if ((short)(CrcReg ^ CurVal) < 0)
                CrcReg = (CrcReg << 1) ^ 0x1021;
            else
                CrcReg <<= 1;
            CurVal <<= 1;
        }
    }
    if((CrcReg&0xff)==pszBuf[unLength] && ((CrcReg>>8)&0xff)==pszBuf[unLength+1])
        return true;
    return false;
}

void CProtocol::SetCRC16(unsigned char * pszBuf, uint16_t unLength)
{
    uint16_t i,j,CurVal;
    uint16_t CrcReg = 0xFFFF;

    for (i = 0; i < unLength; i++)
    {
        CurVal = pszBuf[i] << 8;

        for (j = 0; j < 8; j++)
        {
            if ((short)(CrcReg ^ CurVal) < 0)
                CrcReg = (CrcReg << 1) ^ 0x1021;
            else
                CrcReg <<= 1;
            CurVal <<= 1;
        }
    }
    pszBuf[unLength] = CrcReg&0xff;
    pszBuf[unLength+1] = (CrcReg>>8)&0xff;
}

