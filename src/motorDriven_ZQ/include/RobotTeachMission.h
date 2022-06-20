#pragma once

#include "MotionSharedData.h"
#include "AccDecPlanningMethod.h"
//VSG点动编码
#define         VEHICLE_X_CODE       11
#define         VEHICLE_W_CODE       12

class CRobotTeachMission
{
public:
	CRobotTeachMission();
	~CRobotTeachMission();

public:
	void TeachMissionInitialParam();

	bool ComputeEachSectionTime(double mStartSpeed, 
		                        double mCommandSpeed, 
								double mAccLimit, 
								double mJerkLimit, 
								stInterpolationParamScurve* mJogInteParamScurve);
	bool ScatterEachSectionTime(double mCycleTime, 
		                        stInterpolationParamScurve *mJogInteParamScurve);
	bool ComputeEachPeriodSpeed(int &index, 
		                        double mStartSpeed, 
								double mCommandSpeed, 
								double mMaxAcc, 
								double mMaxJerk, 
								stInterpolationParamScurve *mJogInteParamScurve, 
								double mCycleTime, 
								double *mCurrentSpeed);
    int DoJointJog(int mTeachJointNum,
                   double mStartJointSpeed,
                   double mCommandJointSpeed,
                   double mJointAccLimit,
                   double mJointJerkLimit,
                   double mCycleTime,
                   double *mJointIncreasement);
    int DoVSGMotionStop(stSystemPara *mSystemPara,
                      stRobotJointCoord *mCurrentJointCoord,
                      stJointIncreasement *mJointIncreasement,
                      bool *mTeachMissionFinished);
public:

	bool successOrFailed;
	int errorCode;

	//用于按下启动函数中的整体规划完成标志位
	bool teachGlobalPlanningFinished;
	bool teachStopGlobalPlanningFinished;

	//插补方向
	int interpolationDirection;
	//点动关节序号或者笛卡尔轴序号
	int interpolationAxisNum;

	//规划的起点速度
	double startSpeed;

	double accLimit;
	double jerkLimit;

	//单轴点动计数变量
	int jointMotionCount;
	int cartisianMotionCount;

	//初始化成员变量标志位
	bool teachInitialParamFinished;

	double teachAccDecTime;

	//当前周期关节速度暂存变量
	double currentSpeedTemp;

	stInterpolationParamScurve jogInteParamScurve;

	//S曲线模式下的实际最大加速度
	double actualMaxAcc;
};

