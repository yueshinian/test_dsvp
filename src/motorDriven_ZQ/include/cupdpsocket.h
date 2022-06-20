#ifndef CUPDPSOCKET_H
#define CUPDPSOCKET_H

//#include <QHostAddress>
//#include <QUdpSocket>
#include <sys/socket.h>
#include <netinet/in.h>

class CUdpSocket
{
public:
    CUdpSocket();
    void OnReceive();
    bool initialUDP();
    void SendData(unsigned char *buf ,unsigned char len,char* raddr);

public:
    int m_listenport;
    int m_remoteport;
    
    int sock;
    int len;
    char sendBuffer[255];//自己发出的消息
    int sendLength;//发出信息的长度
    char recvCharArray[255]; //msg length

};

#endif // CUPDPSOCKET_H
