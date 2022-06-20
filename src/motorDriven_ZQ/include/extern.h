//#include "cmytextbox.h"
//#include "serialcom.h"
#include "cprotocol.h"
//#include <QHostAddress>
#include "cagvinfo.h"
#include "cupdpsocket.h"
//#include "mainwindow.h"
//#include <ros/ros.h>
//#include <ros/network.h>
//#include <gazebo_msgs/ModelState.h>
//#include <gazebo_msgs/SetModelState.h>


//extern CAxis *g_paxis;
//extern CProc *g_proc;
//extern CMyTextBox *g_recvtext;
CProtocol *g_protocol;
CAgvInfo *g_agvinfos;
CUdpSocket *g_pudp;

int commandVal;
//extern string g_showval;

//extern unsigned long GetTickCount();
