#include "cupdpsocket.h"
#include "extern.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <iostream>  
#include <stdio.h> 
#include <string.h>  
#include <arpa/inet.h> 

using namespace std; 
//extern CProtocol *g_protocol;

CUdpSocket::CUdpSocket()
{
    m_listenport = 16006;
    m_remoteport = 16006;

    
    sock = -1;
    initialUDP();
}
   struct sockaddr_in addrto,clnadr;
bool CUdpSocket::initialUDP()
{
    setvbuf(stdout, NULL, _IONBF, 0);
    fflush(stdout);
    // 绑定地址
   bzero(&addrto, sizeof(struct sockaddr_in));
   addrto.sin_family = AF_INET;
   addrto.sin_addr.s_addr = inet_addr("192.168.1.200");//htonl(INADDR_ANY);
   addrto.sin_port = htons(m_listenport);

   bzero(&clnadr, sizeof(struct sockaddr_in));
   clnadr.sin_family = AF_INET;
   clnadr.sin_addr.s_addr = inet_addr("192.168.1.91");//htonl(INADDR_ANY);
   clnadr.sin_port = htons(m_remoteport);

   //创建套接字
   if ((sock = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
   {
       cout<<"socket error"<<endl;
       return false;
   }
   const int opt = 1;

   if(bind(sock,(struct sockaddr *)&(addrto), sizeof(struct sockaddr_in)) == -1)
   {
       cout<<"bind error..."<<endl;
       return false;
   }
   cout<<"初始化成功！"<<endl;
}
struct sockaddr_in from;
void CUdpSocket::OnReceive()
{
    memset(recvCharArray,0,sizeof(recvCharArray));
    int ret=recvfrom(sock, recvCharArray, sizeof(recvCharArray), 0/*MSG_DONTWAIT*/, (struct sockaddr*)&from,(socklen_t*)&len);
    bool bufDataEqual = true;
//    while(1)
//    {
//        if(sendLength == ret)
//        {
//            for(int i = 0;i<sendLength;i++)
//            {
//                if(sendBuffer[i]!=recvCharArray[i])
//                {
//                    bufDataEqual = false;
//                    break;
//                }
//            }
//        }
//        if(sendLength!=ret || false == bufDataEqual){
//            break;
//        }
//        memset(recvCharArray,0,sizeof(recvCharArray));
//        ret=recvfrom(sock, recvCharArray, sizeof(recvCharArray), 0, (struct sockaddr*)&from,(socklen_t*)&len);
//    }
    if(0 == ret)
    {
        printf("network is wrong!");
    }
    if(ret>0)
    {
        g_protocol->DealRecvData(recvCharArray,ret,(char*)"192.168.1.91");
    }
}

void CUdpSocket::SendData(unsigned char *buf ,unsigned char len ,char* raddr)
{
    memset(sendBuffer,0,sizeof(sendBuffer));
    memcpy(sendBuffer,(char*)buf,len);
    sendLength = len;

    int ret = sendto(sock,(char*)buf,len,0,(struct sockaddr*)&clnadr,sizeof(sockaddr_in));
    if(ret <0)
    {
       printf("send error!");
    }
}

