#include "motorDrivenMain.h"

using namespace std;
/******************************数据指令宏定义*************************/
#define   LOOP_TIME         0.05 //循环周期 50ms下发一次
/***************************声明通讯全局变量**************************/
extern CProtocol *g_protocol;
extern CAgvInfo *g_agvinfos;
extern CUdpSocket *g_pudp;
extern int commandVal;
/*****************************函数声明****************************/
void initialDriven(); //通讯初始化
//电机数据下发
void motorFollowVelocity(int axis,float speed);
void relayControlFollow(bool control[8]);
/****************************数据接收多线程************************/
bool threadFinishFlag = false;
bool threadStartFlag = false;
void *receiveSocketThread(void *ptr)
{
    while(!threadFinishFlag)
    {
       g_pudp->OnReceive();
    }
     pthread_exit(NULL);
}
/***************************************************************/
//移动平台逆向运动学
void moveWheelInverseFunction(double lineDirSpeed,double thetaDirV,double thetaDirPos,double interCycleTime,
                              double &leftV,double &rightV);
//解析点动指令
void ResolveJogCommand(double linearSpeed,
                       double turnSpeed,
                       int &linearCmd,
                       int &turnCmd);
int main(int argc, char **argv)
{
    /************************Initiate ROS***********************/
    ros::init(argc, argv, "motorDrivenMain"); //初始化节点
    ros::AsyncSpinner spinner(1); //开启线程
    spinner.start();
    ros::NodeHandle  Node_Handle; //节点句柄
    /***********************************************************/
    MotorControlHandller MOTORCONTROL_HANDLLER;
    RelayControlHandller RELATCONTROL_HANDLLER;
    //订阅键值信息
    ros::Subscriber motorControl =Node_Handle.subscribe("/X1/cmd_vel", 10,&MotorControlHandller::callback,&MOTORCONTROL_HANDLLER );
    ros::Subscriber relayControl =Node_Handle.subscribe("/controlRealy", 10,&RelayControlHandller::callback,&RELATCONTROL_HANDLLER );
    //驱动初始化
    initialDriven();
    //创建接收线程
    pthread_t receiveDataId;
    if(pthread_create(&receiveDataId,NULL,receiveSocketThread,NULL))
    {
        cout <<"create createThread error!"<<endl;
        return 0;
    }
    //接收数据初始化
    geometry_msgs::Twist motorCommand;
    motorCommand.linear.x = 0;
    motorCommand.angular.z = 0;
    motorDriven_ZQ::ControlRelay relayCommand; //继电器控制
    bool relayCtl[8];
    relayCommand.motorEnable.resize(8);
    for(int i = 0;i<8;i++)
    {
       relayCommand.motorEnable[i] = false;
       relayCtl[i] = false;
    }
    /*******************************移动平台变量*****************************/
    //指令直线和转向速度
    double linearCommandSpeed = 0;
    double turnCommandSpeed = 0;
    //直线和转向速度
    double linearSpeed = 0;
    double turnAngleSpeed = 0;
    //双轮速度
    double leftSpeed = 0;
    double rightSpeed = 0;
    //上一周期双轮速度
    double lastLeftSpeed = 0;
    double lastRightSpeed = 0;
    //转向角度位置
    double turnAngle = 0;
    /**********************************************************************/
    //实例化点动对象
    JogMotion jogMotion;
    bool successOrFail = false;
    int errorCode = 0;
    /***************************系统参数设置****************************/
    stSystemPara systemPara;
    memset(&systemPara,0,sizeof(systemPara));
    systemPara.adjustPara.teachSpeedRatio = 1;
    systemPara.adjustPara.commPeriod = CYCLETIME;
    //S曲线加减速参数
    //VSG限制
    systemPara.motionLimitPara.limitVel_VSG = 4.5;//先设定这么大，后期测试
    systemPara.motionLimitPara.limitAcc_VSG = 4.5*5;
    systemPara.motionLimitPara.limitJerk_VSG = 4.5*10;

    int linearCmd = DEFAULT_CMD;//空闲状态
    int turnCmd = DEFAULT_CMD;
    stRobotJointCoord currentJointCoord = {0}; //当前关节坐标
    stJointIncreasement targetIncreaseJoint = {0};//插补关节增量

    /*************************************************************************/
    while(ros::ok())
    {
        //起始时间
        double start = ros::Time::now().toSec();
        //接收指令数据
        ros::spinOnce();
        //控制指令
        if (MOTORCONTROL_HANDLLER.newMsg())
        {
            motorCommand = MOTORCONTROL_HANDLLER.fetchMsg();
            linearCommandSpeed = motorCommand.linear.x/1.1;
            turnCommandSpeed = motorCommand.angular.z*1.25;
        }
        //继电器控制指令
        if (RELATCONTROL_HANDLLER.newMsg())
        {
            relayCommand = RELATCONTROL_HANDLLER.fetchMsg();
            for(int i = 0;i<8;i++)
            {
                relayCtl[i] = relayCommand.motorEnable[i];
                //printf("%d  ",relayCtl[i]);
            }
            //printf("\n");
            relayControlFollow(relayCtl);
            g_agvinfos[0].SendData();
        }
//        ResolveJogCommand(linearCommandSpeed,
//                          turnCommandSpeed,
//                          linearCmd,
//                          turnCmd);
//        //点动
//        successOrFail = jogMotion.JogMotionPlanning(linearCmd,
//                                                    turnCmd,
//                                                    systemPara,//系统参数
//                                                    currentJointCoord,//当前关节坐标
//                                                    targetIncreaseJoint,//目标关节增量
//                                                    errorCode);
//        if(!successOrFail)
//        {
//            //如果出现错误，直接销毁进程，并打印出错误
//            break;
//        }

//        linearSpeed = targetIncreaseJoint.increasementVechicleX/LOOP_TIME;
//        turnAngleSpeed = targetIncreaseJoint.increasementVechicleW/LOOP_TIME;

        linearSpeed = linearCommandSpeed;
        turnAngleSpeed = turnCommandSpeed;
        //移动平台逆运动学计算
        moveWheelInverseFunction(linearSpeed,turnAngleSpeed,turnAngle,LOOP_TIME,leftSpeed,rightSpeed);
        leftSpeed = leftSpeed;
        leftSpeed = leftSpeed;
        //判断速度是否超限
        if(leftSpeed>30)
        {
            ROS_INFO("left axis out of rang: %.2f",leftSpeed);
            leftSpeed = lastLeftSpeed;
        }
        if(rightSpeed>30)
        {
            ROS_INFO("right axis out of rang: %.2f",rightSpeed);
            rightSpeed = lastRightSpeed;
        }
        ROS_INFO("leftspeed: %f  rightSpeed: %f",leftSpeed*8.8,rightSpeed*8.8);
        //速度下发
        motorFollowVelocity(0,leftSpeed*10);
        g_agvinfos[0].SendData();
        ros::Duration ( 0.01 ).sleep();
        motorFollowVelocity(1,rightSpeed*10);
        g_agvinfos[0].SendData();
        //重新赋值
        lastLeftSpeed = leftSpeed;
        lastRightSpeed = rightSpeed;
        turnAngle = turnAngle + turnAngleSpeed*LOOP_TIME;
        // 50ms下发一次，超时间说明有错误
        double sleep_time = LOOP_TIME - ( ros::Time::now().toSec() - start );
        if ( sleep_time > 0 )
        {
            ros::Duration ( sleep_time ).sleep();
            //ROS_INFO("Cost Time: %lf ms", (LOOP_TIME-sleep_time)*1000);
        }
        else
            ROS_WARN ( "control loop over time: %f ms", -sleep_time*1000 );

    }
    motorFollowVelocity(0,0);
    g_agvinfos[0].SendData();
    ros::Duration ( 0.01 ).sleep();
    motorFollowVelocity(1,0);
    g_agvinfos[0].SendData();
    threadFinishFlag = true;
    ros::waitForShutdown();

    return 0;
}
void initialDriven()
{
    g_pudp = new CUdpSocket();
    g_agvinfos = new CAgvInfo[1];
    g_agvinfos[0].m_raddr = (char*)"192.168.1.91";
    g_protocol = new CProtocol();

}
//电机数据下发
void motorFollowVelocity(int axis,float speed)
{
    int agvseq = 0;
    g_agvinfos[agvseq].m_commandbuf[0] = axis;
    memcpy(g_agvinfos[agvseq].m_commandbuf+1 ,&speed ,4);
    g_agvinfos[agvseq].SendCommand(0x30 ,0x20 ,5 ,g_agvinfos[agvseq].m_commandbuf);
}
//电机数据下发
void relayControlFollow(bool control[8])
{
    int agvseq = 0;
    for(int i = 0;i<8;i++)
    {
        g_agvinfos[agvseq].m_commandbuf[i] = control[i];
        printf("%d  ",g_agvinfos[agvseq].m_commandbuf[i]);
    }
    printf("\n");
    g_agvinfos[agvseq].SendCommand(0x30 ,0xb2 ,8 ,g_agvinfos[agvseq].m_commandbuf);
}
//移动平台逆向运动学
void moveWheelInverseFunction(double lineDirSpeed,double thetaDirV,double thetaDirPos,double interCycleTime,
                              double &leftV,double &rightV)
{
    //移动平台杆件赋值
    double wheelRadius =0.2;//车轮半径 20cm
    double wheelDistance =0.85;//车轮间距 85cm
    //x方向速度
    double xDirV = lineDirSpeed*cos(thetaDirPos);
    //y方向速度
    double yDirV = lineDirSpeed*sin(thetaDirPos);
    //右轮转速
    if(fabs(cos(thetaDirPos))<1e-5)
    {
        rightV = (thetaDirV*wheelDistance/wheelRadius+2*yDirV/wheelRadius/sin(thetaDirPos))*0.5;
    }
    else
    {
        rightV = (thetaDirV*wheelDistance/wheelRadius+2*xDirV/wheelRadius/cos(thetaDirPos))*0.5;
    }
    leftV = rightV - thetaDirV*wheelDistance/wheelRadius;
}
void ResolveJogCommand(double linearSpeed,
                       double turnSpeed,
                       int &linearCmd,
                       int &turnCmd)//输出指令
{
    if(linearSpeed>0){
        linearCmd = VEHICLE_JOG_X_POS_CMD;
    }
    else{
        linearCmd = VEHICLE_JOG_X_NEG_CMD;
    }
    if(turnSpeed>0){
        turnCmd = VEHICLE_JOG_W_POS_CMD;
    }
    else{
        turnCmd = VEHICLE_JOG_W_NEG_CMD;
    }
    //空闲状态
    if(fabs(linearSpeed)<ACCURACY)
    {
        linearCmd = DEFAULT_CMD;
    }
    if(fabs(turnSpeed)<ACCURACY)
    {
        turnCmd = DEFAULT_CMD;
    }
}
