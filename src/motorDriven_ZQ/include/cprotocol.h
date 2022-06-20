#ifndef CPROTOCOL_H
#define CPROTOCOL_H

//#include <QHostAddress>
#include <string>
using namespace std;

typedef  unsigned short int uint16_t;

class CProtocol 
{
public:
   CProtocol();

private:
   string *m_recvbuf;
   //QHostAddress *m_nodeaddr;

public:
   int m_recvcount;
   int m_temprecvcount;

   //数据传输
   float m_comperdistance;
   float m_comperpulse;

public:
    void DealRecvData(char* buf ,int buffLen,char* addr);
    void ParseRecvData(unsigned char *buf ,int seq);
    uint16_t CRC16(unsigned char * pszBuf, uint16_t unLength);
    bool CheckCRC16(unsigned char * pszBuf, uint16_t unLength);
    void SetCRC16(unsigned char * pszBuf, uint16_t unLength);


};

#endif // CPROTOCOL_H
