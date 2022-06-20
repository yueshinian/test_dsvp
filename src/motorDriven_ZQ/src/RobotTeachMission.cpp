#include "RobotTeachMission.h"
#include <string.h>

CRobotTeachMission::CRobotTeachMission()
{
	teachInitialParamFinished = false;
}

CRobotTeachMission::~CRobotTeachMission()
{
}
/************************************************************************
Function:TeachMissionInitialParam

Description:点动任务初始化程序

Input：无

Output：无

Return：void
************************************************************************/
void CRobotTeachMission::TeachMissionInitialParam()
{
	successOrFailed = true;
	errorCode = 0;

	//用于按下启动函数中的整体规划完成标志位
	teachGlobalPlanningFinished = false;
	teachStopGlobalPlanningFinished = false;

	//插补方向
	interpolationDirection = 0;

	//规划的起点速度,用于减速规划，关节空间和笛卡尔空间都可以表示
	startSpeed = 0;
	accLimit = 0;
	jerkLimit = 0;

	//单轴点动计数变量
	jointMotionCount = 1;

	//初始化成员变量标志位
	teachInitialParamFinished = true;

	teachAccDecTime = 0.0;

	//当前周期关节速度暂存变量
	currentSpeedTemp = 0.0;

    memset(&jogInteParamScurve, 0, sizeof(struct stInterpolationParamScurve));

	actualMaxAcc = 0.0;

}
bool CRobotTeachMission::ComputeEachSectionTime(double mStartSpeed,
	                                            double mCommandSpeed,
												double mAccLimit,
												double mJerkLimit,
												stInterpolationParamScurve *mJogInteParamScurve)
{
	double mActualMaxAccTemp = 0;
	//加速曲线
	if (mStartSpeed < mCommandSpeed)
	{
		mJogInteParamScurve->t2 = (mCommandSpeed - mStartSpeed) / mAccLimit - mAccLimit / mJerkLimit;

		//加速段存在匀加速
		if (mJogInteParamScurve->t2 > 0)
		{
			mActualMaxAccTemp = mAccLimit;
			mJogInteParamScurve->t1 = mActualMaxAccTemp / mJerkLimit;
			mJogInteParamScurve->t3 = mJogInteParamScurve->t1;
		}
		//加速段不存在匀加速
		else
		{
            mActualMaxAccTemp = sqrt(mJerkLimit*(mCommandSpeed - mStartSpeed));
			mJogInteParamScurve->t1 = mActualMaxAccTemp / mJerkLimit;
			mJogInteParamScurve->t2 = 0;
			mJogInteParamScurve->t3 = mJogInteParamScurve->t1;
		}
	}
	//减速曲线
	else
	{
		mJogInteParamScurve->t6 = (mStartSpeed - mCommandSpeed) / mAccLimit - mAccLimit / mJerkLimit;
		if (mJogInteParamScurve->t6 >= 0)
		{
			mActualMaxAccTemp = mAccLimit;
			//加减速段
			mJogInteParamScurve->t5 = mJogInteParamScurve->t7 = mActualMaxAccTemp / mJerkLimit;
		}
		else
		{
            mActualMaxAccTemp = sqrt(mJerkLimit*(mStartSpeed - mCommandSpeed));
			//加减速段
			mJogInteParamScurve->t5 = mActualMaxAccTemp / mJerkLimit;
			mJogInteParamScurve->t6 = 0;
			mJogInteParamScurve->t7 = mJogInteParamScurve->t5;
		}
	}

	return true;
}

/************************************************************************
Function:ScatterEachSectionTime

Description:对各段时间进行离散化处理，即以插补周期为单位进行圆整

Input：规划好的各段时间 *mScurveSectionTime
插补周期 mCycleTime

Output：循环周期数 *mCycleQuantity

Return：计算成功返回true
计算失败返回false
************************************************************************/
bool CRobotTeachMission::ScatterEachSectionTime(double mCycleTime,
												stInterpolationParamScurve *mJogInteParamScurve)
{
	//计算各段圆整后的周期数
	mJogInteParamScurve->cycleQuantity1 = (int)(mJogInteParamScurve->t1 / mCycleTime);
    if (fabs((mJogInteParamScurve->cycleQuantity1*mCycleTime + mCycleTime) - mJogInteParamScurve->t1) < ACCURACY)
	{
		mJogInteParamScurve->cycleQuantity1 = mJogInteParamScurve->cycleQuantity1 + 1;
	}

	mJogInteParamScurve->cycleQuantity2 = (int)(mJogInteParamScurve->t2 / mCycleTime);
    if (fabs((mJogInteParamScurve->cycleQuantity2*mCycleTime + mCycleTime) - mJogInteParamScurve->t2) < ACCURACY)
	{
		mJogInteParamScurve->cycleQuantity2 = mJogInteParamScurve->cycleQuantity2 + 1;
	}

	mJogInteParamScurve->cycleQuantity3 = (int)(mJogInteParamScurve->t3 / mCycleTime);
    if (fabs((mJogInteParamScurve->cycleQuantity3*mCycleTime + mCycleTime) - mJogInteParamScurve->t3) < ACCURACY)
	{
		mJogInteParamScurve->cycleQuantity3 = mJogInteParamScurve->cycleQuantity3 + 1;
	}

	mJogInteParamScurve->cycleQuantity4 = (int)(mJogInteParamScurve->t4 / mCycleTime);
    if (fabs((mJogInteParamScurve->cycleQuantity4*mCycleTime + mCycleTime) - mJogInteParamScurve->t4) < ACCURACY)
	{
		mJogInteParamScurve->cycleQuantity4 = mJogInteParamScurve->cycleQuantity4 + 1;
	}

	mJogInteParamScurve->cycleQuantity5 = (int)(mJogInteParamScurve->t5 / mCycleTime);
    if (fabs((mJogInteParamScurve->cycleQuantity5*mCycleTime + mCycleTime) - mJogInteParamScurve->t5) < ACCURACY)
	{
		mJogInteParamScurve->cycleQuantity5 = mJogInteParamScurve->cycleQuantity5 + 1;
	}

	mJogInteParamScurve->cycleQuantity6 = (int)(mJogInteParamScurve->t6 / mCycleTime);
    if (fabs((mJogInteParamScurve->cycleQuantity6*mCycleTime + mCycleTime) - mJogInteParamScurve->t6) < ACCURACY)
	{
		mJogInteParamScurve->cycleQuantity6 = mJogInteParamScurve->cycleQuantity6 + 1;
	}

	mJogInteParamScurve->cycleQuantity7 = (int)(mJogInteParamScurve->t7 / mCycleTime);
    if (fabs((mJogInteParamScurve->cycleQuantity7*mCycleTime + mCycleTime) - mJogInteParamScurve->t7) < ACCURACY)
	{
		mJogInteParamScurve->cycleQuantity7 = mJogInteParamScurve->cycleQuantity7 + 1;
	}

	mJogInteParamScurve->wholeCycleQuantity = mJogInteParamScurve->cycleQuantity1
		+ mJogInteParamScurve->cycleQuantity2
		+ mJogInteParamScurve->cycleQuantity3
		+ mJogInteParamScurve->cycleQuantity4
		+ mJogInteParamScurve->cycleQuantity5
		+ mJogInteParamScurve->cycleQuantity6
		+ mJogInteParamScurve->cycleQuantity7;

	return true;
}

/************************************************************************
Function: ComputeEachPeriodSpeed

Description: 关节单轴S曲线规划,该函数用来计算每个周期的速度，可用于关节和笛卡尔空间的速度计算

Input：	初始速度 mStartSpeed
终点速度 mCommandSpeed
最大加速度 mMaxAcc
最大加加速度 mMaxJerk
S曲线规划的各段时间周期数
通讯周期 mCycleTime

InOutput： 计数变量 index
当前周期的目标速度 mCurrentSpeed

Return：计算成功返回true
计算失败返回false
************************************************************************/
bool CRobotTeachMission::ComputeEachPeriodSpeed(int &index,
	                                            double mStartSpeed,
												double mCommandSpeed,
												double mMaxAcc,
												double mMaxJerk,
												stInterpolationParamScurve *mJogInteParamScurve,
												double mCycleTime,
												double *mCurrentSpeed)
{
	//当前周期关节速度暂存变量
	double mCurrentSpeedTemp = 0;

	//中间计数变量
	int mCount = 0;
	if (mStartSpeed < mCommandSpeed)
	{
		if (mJogInteParamScurve->cycleQuantity2 > 0)
		{
			//加加速段
			mCount = mJogInteParamScurve->cycleQuantity1;
			if (index < mCount + 1)
			{
				mCurrentSpeedTemp = mStartSpeed + (mMaxJerk*index*mCycleTime*index*mCycleTime) / 2;
			}
			//匀加速段
			if (index > mCount)
			{
				mCount += mJogInteParamScurve->cycleQuantity2;
				if (index < mCount + 1)
				{
					int tem = index - mJogInteParamScurve->cycleQuantity1;
					mCurrentSpeedTemp = mStartSpeed + mMaxJerk*mJogInteParamScurve->cycleQuantity1*mCycleTime*mJogInteParamScurve->cycleQuantity1*mCycleTime / 2 + mMaxAcc*tem*mCycleTime;
				}
			}
			//减加速段
			if (index > mCount)
			{
				mCount += mJogInteParamScurve->cycleQuantity3;
				if (index < mCount + 1)
				{
					int tem = index - mJogInteParamScurve->cycleQuantity1 - mJogInteParamScurve->cycleQuantity2;
					mCurrentSpeedTemp = mStartSpeed + mMaxJerk*mJogInteParamScurve->cycleQuantity1*mCycleTime*mJogInteParamScurve->cycleQuantity1*mCycleTime / 2
						+ mMaxAcc*mJogInteParamScurve->cycleQuantity2*mCycleTime + mMaxAcc*tem*mCycleTime - mMaxJerk*tem*mCycleTime*tem*mCycleTime / 2;
				}
			}
			if (index > mCount)
			{
				mCurrentSpeedTemp = (*mCurrentSpeed);//末速度
			}
		}
		//加速段不存在匀加速
		else
		{
			//加加速段
			mCount = mJogInteParamScurve->cycleQuantity1;
			if (index < mCount + 1)
			{
				mCurrentSpeedTemp = mStartSpeed + (mMaxJerk*index*mCycleTime*index*mCycleTime) / 2;

			}

			//减加速段
			if (index > mCount)
			{
				mCount += mJogInteParamScurve->cycleQuantity3;
				if (index < mCount + 1)
				{
					int tem = index - mJogInteParamScurve->cycleQuantity1;
					mCurrentSpeedTemp = mStartSpeed + mMaxJerk*mJogInteParamScurve->cycleQuantity1*mCycleTime*mJogInteParamScurve->cycleQuantity1*mCycleTime / 2
						+ mMaxAcc*tem*mCycleTime - mMaxJerk*tem*mCycleTime*tem*mCycleTime / 2;
				}
			}
			if (index > mCount)
			{
				mCurrentSpeedTemp = (*mCurrentSpeed);
			}
		}
	}

	//减速曲线
	else
	{
		if (mJogInteParamScurve->cycleQuantity6 > 0)
		{
			mCount = mJogInteParamScurve->cycleQuantity5;
			if (index < mCount + 1)
			{
				mCurrentSpeedTemp = mStartSpeed - mMaxJerk*index*mCycleTime*index*mCycleTime / 2;
			}

			//匀减速段
			if (index > mCount)
			{
				mCount += mJogInteParamScurve->cycleQuantity6;
				if (index < mCount + 1)
				{
					int tem = index - mJogInteParamScurve->cycleQuantity5;
					mCurrentSpeedTemp = mStartSpeed - mMaxJerk*mJogInteParamScurve->cycleQuantity5*mCycleTime*mJogInteParamScurve->cycleQuantity5*mCycleTime / 2 - mMaxAcc*tem*mCycleTime;
				}
			}

			//减减速段
			if (index > mCount)
			{
				mCount += mJogInteParamScurve->cycleQuantity7;
				if (index < mCount + 1)
				{
					int tem = index - mJogInteParamScurve->cycleQuantity5 - mJogInteParamScurve->cycleQuantity6;
					mCurrentSpeedTemp = mStartSpeed - mMaxJerk*mJogInteParamScurve->cycleQuantity5*mCycleTime*mJogInteParamScurve->cycleQuantity5*mCycleTime / 2
						- mMaxAcc*mJogInteParamScurve->cycleQuantity6*mCycleTime - mMaxAcc*tem*mCycleTime + mMaxJerk*tem*mCycleTime*tem*mCycleTime / 2;
				}
			}
			if (index > mCount)
			{
				mCurrentSpeedTemp = mCommandSpeed;
			}
		}
		else
		{
			//加减速段
			mCount = mJogInteParamScurve->cycleQuantity5;
			if (index < mCount + 1)
			{
				mCurrentSpeedTemp = mStartSpeed - mMaxJerk*index*index*mCycleTime*mCycleTime / 2;
			}

			//减减速段
			if (index > mCount)
			{
				mCount += mJogInteParamScurve->cycleQuantity7;
				if (index < mCount + 1)
				{
					int tem = index - mJogInteParamScurve->cycleQuantity5;
					mCurrentSpeedTemp = mStartSpeed - mMaxJerk*mJogInteParamScurve->cycleQuantity5*mCycleTime*mJogInteParamScurve->cycleQuantity5*mCycleTime / 2
						- mMaxAcc*tem*mCycleTime + mMaxJerk*tem*mCycleTime*tem*mCycleTime / 2;
				}
			}
			if (index > mCount)
			{
				mCurrentSpeedTemp = mCommandSpeed;
			}
		}
	}

	//赋值
	(*mCurrentSpeed) = mCurrentSpeedTemp;

	return true;
}
/************************************************************************
Function:DoJointJog

Description:关节轴点动程序

Input：
mJointJogNum  关节轴序号
mStartJointSpeed  起点速度
mCommandJointSpeed  指令速度
mJointAccLimit  加速度限制
mCycleTime  通讯周期

Output：
mJointIncreasement  关节角度增量

Return：
计算成功返回0，计算失败返回errorCode
************************************************************************/
int CRobotTeachMission::DoJointJog(int mTeachJointNum,
	                               double mStartJointSpeed,
								   double mCommandJointSpeed,
								   double mJointAccLimit,
								   double mJointJerkLimit,
								   double mCycleTime,
								   double *mJointIncreasement)
{
	//初始化成员变量
    if (!teachInitialParamFinished)
	{
		TeachMissionInitialParam();
	}
	//整体规划模块
	if (!teachGlobalPlanningFinished)
	{
		//获取起点速度
		startSpeed = mStartJointSpeed;

		//获取插补轴数变量
		interpolationAxisNum = mTeachJointNum > 0 ? mTeachJointNum : (-mTeachJointNum);

		//计算S曲线各段时间
		successOrFailed = ComputeEachSectionTime(startSpeed,
			                                     mCommandJointSpeed,
			                                     mJointAccLimit,
												 mJointJerkLimit,
												 &jogInteParamScurve);
		if (!successOrFailed)
		{
			errorCode = 11;
			return errorCode;
		}

		//圆整各段时间
		successOrFailed = ScatterEachSectionTime(mCycleTime,
												 &jogInteParamScurve);
		if (!successOrFailed)
		{
			errorCode = 12;
			return errorCode;
		}

		//计算圆整之后能达到的实际最大加速度
		actualMaxAcc = jogInteParamScurve.cycleQuantity1*mCycleTime*mJointJerkLimit;

		teachGlobalPlanningFinished = true;
	}

	//周期插补过程
	//计算当前周期的关节相对速度
	successOrFailed = ComputeEachPeriodSpeed(jointMotionCount,
		                                     startSpeed,
											 mCommandJointSpeed,
											 actualMaxAcc,
											 mJointJerkLimit,
											 &jogInteParamScurve,
											 mCycleTime,
											 &currentSpeedTemp);
	if (!successOrFailed)
	{
		errorCode = 13;
		return errorCode;
	}
	jointMotionCount++;

	//计算插补方向
	interpolationDirection = mTeachJointNum > 0 ? 1 : -1;

	//关节增量输出
	(*mJointIncreasement) = interpolationDirection*currentSpeedTemp*mCycleTime;

	return errorCode;
}

/************************************************************************
Function:DoVSGMotionStop

Description:VSG单轴减速停止程序

Input：
mSystemPara  系统变量，本程序用到VSG点动加速度限制和通讯周期
mCurrentJointCoord  当前关节坐标，本程序用到当前关节速度

Output：
mJointIncreasement  关节角度增量
mTeachMissionFinished  点动任务完成标志位

Return：
计算成功返回0，计算失败返回errorCode
************************************************************************/
int CRobotTeachMission::DoVSGMotionStop(stSystemPara *mSystemPara,
                                          stRobotJointCoord *mCurrentJointCoord,
                                          stJointIncreasement *mJointIncreasement,
                                          bool *mTeachMissionFinished)
{
    if (!teachStopGlobalPlanningFinished)
    {
        //不用初始化函数原因：保留插补方向
        jointMotionCount = 1;
        startSpeed = 0.0;
        accLimit = 0.0;
        jerkLimit = 0.0;
        currentSpeedTemp = 0.0;
        memset(&jogInteParamScurve, 0, sizeof(struct stInterpolationParamScurve));

        actualMaxAcc = 0.0;
        switch (interpolationAxisNum)
        {
            ////初始化
            //TeachMissionInitialParam();

            //获取减速运动参数
            case VEHICLE_X_CODE:
                startSpeed = fabs(mCurrentJointCoord->vechicleSpeedX);
                accLimit = mSystemPara->motionLimitPara.limitVel_VSG;
                jerkLimit = mSystemPara->motionLimitPara.limitJerk_VSG;
                break;

            case VEHICLE_W_CODE:
                startSpeed = fabs(mCurrentJointCoord->vechicleSpeedW);
                accLimit = mSystemPara->motionLimitPara.limitVel_VSG;
                jerkLimit = mSystemPara->motionLimitPara.limitJerk_VSG;
                break;

            default:
                errorCode = 25;
                return errorCode;
        }
        //规划减速各段时间
        successOrFailed = ComputeEachSectionTime(startSpeed,
                                                 0,
                                                 accLimit,
                                                 jerkLimit,
                                                 &jogInteParamScurve);
        if (!successOrFailed)
        {
            errorCode = 51;
            return errorCode;
        }

        //圆整各段时间
        successOrFailed = ScatterEachSectionTime(mSystemPara->adjustPara.commPeriod,
                                                 &jogInteParamScurve);
        if (!successOrFailed)
        {
            errorCode = 52;
            return errorCode;
        }
        //计算圆整之后能达到的实际最大加速度
        actualMaxAcc = jogInteParamScurve.cycleQuantity5*mSystemPara->adjustPara.commPeriod*jerkLimit;

        teachStopGlobalPlanningFinished = true;
    }
    //计算当前周期的当前关节速度
    successOrFailed = ComputeEachPeriodSpeed(jointMotionCount,
                                             startSpeed,
                                             0,
                                             actualMaxAcc,
                                             jerkLimit,
                                             &jogInteParamScurve,
                                             mSystemPara->adjustPara.commPeriod,
                                             &currentSpeedTemp);
    if (!successOrFailed)
    {
        errorCode = 23;
        return errorCode;
    }
    jointMotionCount++;
    switch (interpolationAxisNum)
    {
    case VEHICLE_X_CODE:
        mJointIncreasement->increasementVechicleX = interpolationDirection*currentSpeedTemp*mSystemPara->adjustPara.commPeriod;
        break;
    case VEHICLE_W_CODE:
        mJointIncreasement->increasementVechicleW = interpolationDirection*currentSpeedTemp*mSystemPara->adjustPara.commPeriod;
        break;
    default:
        errorCode = 53;
        return errorCode;
        break;
    }
    if (fabs(currentSpeedTemp) < ACCURACY)
    {
        switch (interpolationAxisNum)
        {
            case VEHICLE_X_CODE:
                mJointIncreasement->increasementVechicleX = 0;
            break;
            case VEHICLE_W_CODE:
                mJointIncreasement->increasementVechicleW = 0;
            break;
        }
        *mTeachMissionFinished = true;
        TeachMissionInitialParam();
    }
    return errorCode;
}
