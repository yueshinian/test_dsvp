#include "JogMotion.h"
#include "RobotTeachMission.h"
#include "string.h"

CRobotTeachMission linearTeachMission;//直线点动对象初始化
CRobotTeachMission turnTeachMission;//转向点动对象初始化
stSystemPara systemPara; //系统参数设置

JogMotion::JogMotion()
{
    linearJogInitialFlag = false;
    turnJogInitialFlag = false;
}
JogMotion::~JogMotion()
{

}
//初始化
void JogMotion::LinearInitialParam()
{
    sucessOrFailedFlag = false;
    linearStatus = IDLE_STATUS;
    linearJogFinished = false;
}
void JogMotion::TurnInitialParam()
{
    sucessOrFailedFlag = false;
    turnStatus = IDLE_STATUS;
    turnJogFinished = false;
}
/************************************************************************
Function:JogMotionPlanning

Description：点动运动规划

Input:

systemPara 系统参数

InOutput:
mLinearCmd 直线指令
mTurnCmd 转向指令

Output:
currentJointCoord 当前关节坐标
targetIncreaseJoint 目标关节增量
errorCode 错误代码

Return：计算成功返回true，计算失败返回false
************************************************************************/
bool JogMotion::JogMotionPlanning(int &linearCmd,
                                  int &turnCmd,
                                  stSystemPara systemPara,//系统参数
                                  stRobotJointCoord &currentJointCoord,//当前关节坐标
                                  stJointIncreasement &targetIncreaseJoint,//目标关节增量
                                  int &errorCode)
{
    //初始化
    if(!linearJogInitialFlag)
    {
        LinearInitialParam();
        linearJogInitialFlag = true;
    }
    if(!turnJogInitialFlag)
    {
        TurnInitialParam();
        turnJogInitialFlag = true;
    }
    //状态判断
    sucessOrFailedFlag = ResolveTeachCommand(linearCmd,
                                             turnCmd,
                                            &linearStatus,
                                            &turnStatus,
                                            linearJogFinished,
                                            turnJogFinished);
    if(!sucessOrFailedFlag)
    {
        errorCode = 1001;//点动状态错误
        return false;
    }
    errorCode = DetectSystemStatus(&systemPara,
                                   &linearStatus,
                                   &turnStatus,
                                   &currentJointCoord,
                                   &targetIncreaseJoint,
                                   &linearJogFinished,
                                   &turnJogFinished);
    if(errorCode!=0)
    {
        errorCode =1002;//点动执行错误
        return false;
    }
    //点动完成，清空数据
    if(linearJogFinished||linearStatus == IDLE_STATUS)
    {
        LinearInitialParam();
        linearJogInitialFlag = false;
        linearCmd = DEFAULT_CMD;
    }
    if(turnJogFinished||turnStatus == IDLE_STATUS)
    {
        TurnInitialParam();
        turnJogInitialFlag = false;
        turnCmd = DEFAULT_CMD;
    }
    currentJointCoord.vechicleSpeedX = targetIncreaseJoint.increasementVechicleX/CYCLETIME;
    currentJointCoord.vechicleSpeedW = targetIncreaseJoint.increasementVechicleW/CYCLETIME;

    return true;
}

/************************************************************************
Function:ResolveTeachCommand

Description：解析示教状态指令

Input:
mLinearCmd 直线指令
mTurnCmd 转向指令
mLinearMissionFinished 直线任务完成标志位

InOutput:
mLinearStatus  直线状态变量
mTurnStatus 转向状态变量

Return：计算成功返回true，计算失败返回false
************************************************************************/
bool JogMotion::ResolveTeachCommand(int mLinearCmd,
                                    int mTurnCmd,
                       int *mLinearStatus,
                       int *mTurnStatus,
                       bool mLinearMissionFinished,
                       bool mTurnMissionFinished)
{
    //当前状态为空闲
    if((*mLinearStatus) == IDLE_STATUS && mLinearCmd == DEFAULT_CMD)
    {
        (*mLinearStatus) = IDLE_STATUS;
        mLinearCmd == DEFAULT_CMD;
        //printf("1\n");
    }
    if((*mTurnStatus) == IDLE_STATUS && mTurnCmd == DEFAULT_CMD)
    {
        (*mTurnStatus) = IDLE_STATUS;
        mTurnCmd == DEFAULT_CMD;
        //printf("2\n");
    }
    /**********加速启动情况***********/
    //移动平台X轴正向
    if ((*mLinearStatus) == IDLE_STATUS
        && mLinearCmd == VEHICLE_JOG_X_POS_CMD)
    {
        (*mLinearStatus) = VEHICLE_JOG_X_POS_EXEC;
        //printf("3\n");

    }
    else if ((*mLinearStatus) == VEHICLE_JOG_X_POS_EXEC
        && mLinearCmd != DEFAULT_CMD)
    {
        (*mLinearStatus) = VEHICLE_JOG_X_POS_EXEC;
        //printf("4\n");
    }
    //移动平台X轴反向
    if ((*mLinearStatus) == IDLE_STATUS
        && mLinearCmd == VEHICLE_JOG_X_NEG_CMD)
    {
        (*mLinearStatus) = VEHICLE_JOG_X_NEG_EXEC;
        //printf("5\n");

    }
    else if ((*mLinearStatus) == VEHICLE_JOG_X_NEG_EXEC
        && mLinearCmd != DEFAULT_CMD)
    {
        (*mLinearStatus) = VEHICLE_JOG_X_NEG_EXEC;
        //printf("6\n");
    }
    //移动平台W轴正向
    if ((*mTurnStatus) == IDLE_STATUS
        && mTurnCmd == VEHICLE_JOG_W_POS_CMD)
    {
        (*mTurnStatus) = VEHICLE_JOG_W_POS_EXEC;
        //printf("7\n");
    }
    else if ((*mTurnStatus) == VEHICLE_JOG_W_POS_EXEC
        && mTurnCmd != DEFAULT_CMD)
    {
        (*mTurnStatus) = VEHICLE_JOG_W_POS_EXEC;
        //printf("8\n");
    }
    //移动平台W轴反向
    if ((*mTurnStatus) == IDLE_STATUS
        && mTurnCmd == VEHICLE_JOG_W_NEG_CMD)
    {
        (*mTurnStatus) = VEHICLE_JOG_W_NEG_EXEC;
        //printf("9\n");

    }
    else if ((*mTurnStatus) == VEHICLE_JOG_W_NEG_EXEC
        && mTurnCmd != DEFAULT_CMD)
    {
        (*mTurnStatus) = VEHICLE_JOG_W_NEG_EXEC;
        //printf("10\n");

    }
    //点动停止
    if(((*mLinearStatus) == VEHICLE_JOG_X_POS_EXEC || (*mLinearStatus) == VEHICLE_JOG_X_NEG_EXEC)&& (mLinearCmd == DEFAULT_CMD))
    {
        (*mLinearStatus) = LINEAR_JOG_DEC_EXEC;
        //printf("9\n");
    }
    else if ((*mLinearStatus) == LINEAR_JOG_DEC_EXEC
        && mLinearMissionFinished == false)
    {
        (*mLinearStatus) = LINEAR_JOG_DEC_EXEC;
        //printf("10\n");
    }

    if(((*mTurnStatus) == VEHICLE_JOG_W_POS_EXEC || (*mTurnStatus) == VEHICLE_JOG_W_NEG_EXEC)&& (mTurnCmd == DEFAULT_CMD))
    {
        (*mTurnStatus) = TURN_JOG_DEC_EXEC;
        //printf("11\n");
    }
    else if ((*mTurnStatus) == TURN_JOG_DEC_EXEC
        && mTurnMissionFinished == false)
    {
        (*mTurnStatus) = TURN_JOG_DEC_EXEC;
        //printf("12\n");
    }

    return true;
}
/************************************************************************
Function:DetectSystemStatus

Description：检测机器人系统状态，执行相应任务

Input：
mSystemPara 系统参数
mLinearStatus  直线状态参数
mTurnStatus 转向状态参数

InOutput：
mCurrentJointCoord  当前关节坐标

Output：
mIncreasementJoint  关节增量
mLinearMissionFinished  直线任务完成标志位
mTurnMissionFinished  转向任务完成标志位

Return：
执行正确返回0，执行错误返回错误代码
************************************************************************/
int JogMotion::DetectSystemStatus(stSystemPara *mSystemPara,
                                  int *mLinearStatus,
                                  int *mTurnStatus,
                                  stRobotJointCoord *mCurrentJointCoord,
                                  stJointIncreasement *mIncreasementJoint,
                                  bool *mLinearMissionFinished,
                                  bool *mTurnMissionFinished)
{
    int errorCode = 0;

    switch (*mLinearStatus)
    {
        case IDLE_STATUS://空闲状态
            //清零关节速度
            mCurrentJointCoord->vechicleSpeedX = 0;
            //清零关节增量
            mIncreasementJoint->increasementVechicleX = 0;
            break;
        case VEHICLE_JOG_X_POS_EXEC:
            errorCode = linearTeachMission.DoJointJog(VEHICLE_X_CODE,
                                                    mCurrentJointCoord->vechicleSpeedX,
                                                    mSystemPara->motionLimitPara.limitVel_VSG*mSystemPara->adjustPara.teachSpeedRatio,
                                                    mSystemPara->motionLimitPara.limitAcc_VSG*mSystemPara->adjustPara.teachSpeedRatio,
                                                    mSystemPara->motionLimitPara.limitJerk_VSG*mSystemPara->adjustPara.teachSpeedRatio,
                                                    CYCLETIME,
                                                    &mIncreasementJoint->increasementVechicleX);

            if (errorCode)
            {
                errorCode = 2440101;
                return errorCode;
            }
            //更新关节坐标
            mCurrentJointCoord->vechicleX = mCurrentJointCoord->vechicleX + mIncreasementJoint->increasementVechicleX;
            break;
        case VEHICLE_JOG_X_NEG_EXEC:

            errorCode = linearTeachMission.DoJointJog(-VEHICLE_X_CODE,
                                                    mCurrentJointCoord->vechicleSpeedX,
                                                    mSystemPara->motionLimitPara.limitVel_VSG*mSystemPara->adjustPara.teachSpeedRatio,
                                                    mSystemPara->motionLimitPara.limitAcc_VSG*mSystemPara->adjustPara.teachSpeedRatio,
                                                    mSystemPara->motionLimitPara.limitJerk_VSG*mSystemPara->adjustPara.teachSpeedRatio,
                                                    CYCLETIME,
                                                    &mIncreasementJoint->increasementVechicleX);

            if (errorCode)
            {
                errorCode = 2440101;
                return errorCode;
            }
            //更新关节坐标
            mCurrentJointCoord->vechicleX = mCurrentJointCoord->vechicleX + mIncreasementJoint->increasementVechicleX;

            break;
        case LINEAR_JOG_DEC_EXEC:
            errorCode = linearTeachMission.DoVSGMotionStop(mSystemPara,
                                                           mCurrentJointCoord,
                                                           mIncreasementJoint,
                                                           mLinearMissionFinished);

            if (errorCode)
            {
                errorCode = 2442700;
                return errorCode;
            }
            mCurrentJointCoord->vechicleX = mCurrentJointCoord->vechicleX + mIncreasementJoint->increasementVechicleX;
            break;
        default:
            linearTeachMission.TeachMissionInitialParam();

            errorCode = 2450001;
            return errorCode;
    }
    switch (*mTurnStatus)
    {
        case IDLE_STATUS://空闲状态
            //清零关节速度
            mCurrentJointCoord->vechicleSpeedW = 0;
            //清零关节增量
            mIncreasementJoint->increasementVechicleW = 0;
            break;
        case VEHICLE_JOG_W_POS_EXEC:
            errorCode = turnTeachMission.DoJointJog(VEHICLE_W_CODE,
                                                    mCurrentJointCoord->vechicleSpeedW,
                                                    mSystemPara->motionLimitPara.limitVel_VSG*mSystemPara->adjustPara.teachSpeedRatio,
                                                    mSystemPara->motionLimitPara.limitAcc_VSG*mSystemPara->adjustPara.teachSpeedRatio,
                                                    mSystemPara->motionLimitPara.limitJerk_VSG*mSystemPara->adjustPara.teachSpeedRatio,
                                                    CYCLETIME,
                                                    &mIncreasementJoint->increasementVechicleW);

            if (errorCode)
            {
                errorCode = 2440101;
                return errorCode;
            }
            //更新关节坐标
            mCurrentJointCoord->vechicleW = mCurrentJointCoord->vechicleW + mIncreasementJoint->increasementVechicleW;

            break;
        case VEHICLE_JOG_W_NEG_EXEC:
            errorCode = turnTeachMission.DoJointJog(-VEHICLE_W_CODE,
                                                    mCurrentJointCoord->vechicleSpeedW,
                                                    mSystemPara->motionLimitPara.limitVel_VSG*mSystemPara->adjustPara.teachSpeedRatio,
                                                    mSystemPara->motionLimitPara.limitAcc_VSG*mSystemPara->adjustPara.teachSpeedRatio,
                                                    mSystemPara->motionLimitPara.limitJerk_VSG*mSystemPara->adjustPara.teachSpeedRatio,
                                                    CYCLETIME,
                                                    &mIncreasementJoint->increasementVechicleW);

            if (errorCode)
            {
                errorCode = 2440101;
                return errorCode;
            }
            //更新关节坐标
            mCurrentJointCoord->vechicleW = mCurrentJointCoord->vechicleW + mIncreasementJoint->increasementVechicleW;
            break;
        case TURN_JOG_DEC_EXEC:
            errorCode = turnTeachMission.DoVSGMotionStop(mSystemPara,
                                                           mCurrentJointCoord,
                                                           mIncreasementJoint,
                                                           mTurnMissionFinished);

            if (errorCode)
            {
                errorCode = 2442700;
                return errorCode;
            }
            mCurrentJointCoord->vechicleW = mCurrentJointCoord->vechicleW + mIncreasementJoint->increasementVechicleW;
            break;
        default:
            turnTeachMission.TeachMissionInitialParam();

            errorCode = 2450001;
            return errorCode;
    }

    return 0;
}
