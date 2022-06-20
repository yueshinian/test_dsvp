
/************************************************************************/
/* 对共享内存结构体进行声明												*/
/************************************************************************/
#pragma once
#include <math.h>
#include <string>
using namespace std;

/**********************************宏定义*******************************/
#define     PI	                        3.14159265358979323846264
#define     ACCURACY			1e-6            //相等判断误差
#define     CYCLETIME                   0.05               //插补周期
#define     POSITION_AND_LENGTH_ERROR   1e-4            //位置或者长度的判断误差
#define     POSITION_GAP_ACCURACY       1e-2            //位置或者长度的判断误差
#define     INTERATIVE_ERROR	        1e-6            //迭代误差
#define     INTERATINV_TIMES	        100              //最大迭代次数
/************************************************************************/
//关节空间坐标
struct stRobotJointCoord
{
    //VSG关节
    double vechicleX;//移动平台X关节
    double vechicleW;//移动平台W关节

    double vechicleSpeedX;//移动平台X关节速度
    double vechicleSpeedW;
};
//运动限制参数，包括位置，速度，加速度和加加速度
struct stMotionLimitPara
{
    double speedLimit;
    double accLimit;
    double jerkLimit;
    double commandSpeed;

    //VSG(移动平台、摆臂、夹爪)速度、加速度、加加速度限制
    double limitVel_VSG;
    double limitAcc_VSG;
    double limitJerk_VSG;
};
//S曲线各阶段的运行时间
struct stScurveSectionTime
{
    double t1;
    double t2;
    double t3;
    double t4;
    double t5;
    double t6;
    double t7;
    double wholeTime;
};
struct stJointIncreasement
{
    //VSG关节增量
    double increasementVechicleX;//移动平台X方向增量
    double increasementVechicleW;//移动平台W方向增量
};
//机器人系统参数4：可调节系统参数
struct stAdjustPara
{
    double commPeriod;			 //通讯周期，单位ms
    double motionSpeedRatio;     //自动运行、调试模式下的速度系数
    double teachSpeedRatio;      //点动模式下速度系数
};
//机器人系统参数：２个
struct stSystemPara
{
    //１.运动限制参数
    stMotionLimitPara motionLimitPara;
    //２.可调节系统参数
    stAdjustPara adjustPara;
};

