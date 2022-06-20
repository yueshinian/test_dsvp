#ifndef JOGMOTION_H
#define JOGMOTION_H

#include "MotionSharedData.h"

//移动平台单轴运动指令
#define     VEHICLE_JOG_X_POS_CMD   210100010
#define     VEHICLE_JOG_X_NEG_CMD   210100020
#define     VEHICLE_JOG_W_POS_CMD   210100030       //移动平台旋转方向正向
#define     VEHICLE_JOG_W_NEG_CMD   210100040
/****************************状态******************************/
#define		VEHICLE_JOG_X_POS_EXEC	210100011
#define		VEHICLE_JOG_X_NEG_EXEC	210100021
#define		VEHICLE_JOG_W_POS_EXEC	210100031
#define		VEHICLE_JOG_W_NEG_EXEC	210100041
#define     LINEAR_JOG_DEC_EXEC     210100111
#define     TURN_JOG_DEC_EXEC       210100121
/***************系统状态宏定义*********************/
//系统状态
#define		IDLE_STATUS		200000000
//默认指令，程序空跑，在没有运动任务时执行该指令
#define		DEFAULT_CMD		0
class JogMotion
{
public:
    JogMotion();
    ~JogMotion();
    void LinearInitialParam();
    void TurnInitialParam();
    bool JogMotionPlanning(int &liearCmd, int &turnCmd,
                          stSystemPara systemPara, //系统参数
                          stRobotJointCoord &currentJointCoord, //当前关节坐标
                          stJointIncreasement &targetIncreaseJoint, //目标关节增量
                          int &errorCode);
    bool ResolveTeachCommand(int mLinearCmd,
                            int mTurnCmd,
                           int *mLinearStatus,
                           int *mTurnStatus,
                           bool mLinearMissionFinished,
                           bool mTurnMissionFinished);
    int DetectSystemStatus(stSystemPara *mSystemPara,
                           int *mLinearStatus,
                           int *mTurnStatus,
                           stRobotJointCoord *mCurrentJointCoord,
                           stJointIncreasement *mIncreasementJoint,
                           bool *mLinearMissionFinished,
                           bool *mTurnMissionFinished);

public:
    /****************************点动状态******************************/
    bool linearJogInitialFlag;//点动初始化标志位
    bool turnJogInitialFlag;
    int linearStatus; //直线状态
    int turnStatus;
    /******************************************************************/
    bool sucessOrFailedFlag;//函数运行成功与否标志
    bool linearJogFinished;//直线点动完成
    bool turnJogFinished;//转动点动完成
};

#endif
