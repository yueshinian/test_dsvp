#pragma once
#include <math.h>
/**********************************修正梯形加减速**************************************/
struct stInterpolationParamModifiedTrapzoid
{
	double actualVelLimit;
	double maxV;
	double wholeTime;
	double AccAndDecTime;
	double AccAndDecDistance;
	double uniTime;
	double uniDistance;
	double timeBase;
};
/*****************************************S曲线加减速*****************************************/
struct stCompensationLengthParam
{
	int compensatedCycleQuantity;
	double accumulateCompensation;
	double currentPeriodCompenLength;
	bool compensationFinished;
};
struct stComputeLengthUnderScurve
{
	double accumulateCompensation;
	int sectionNum;
	int interpolationCycleCnt;
	bool planningFinished;
	double accumulateSectionLength;
	double currentSpeed;
	double interpolatedTime;
	double targetLineLength;
	bool interpolationFinished;

};
struct stInterpolationParamScurve
{
	//各段时间
	double t1;
	double t2;
	double t3;
	double t4;
	double t5;
	double t6;
	double t7;
	double WholeTime;

	//累计位移
	double length1;
	double length2;
	double length3;
	double length4;
	double length5;
	double length6;
	double length7;

	//节点速度
	double vs;
	double v1;
	double v2;
	double v3;
	double v4;
	double v5;
	double v6;
	double ve;
	double vMax;

	int cycleQuantity1;
	int cycleQuantity2;
	int cycleQuantity3;
	int cycleQuantity4;
	int cycleQuantity5;
	int cycleQuantity6;
	int cycleQuantity7;
	int wholeCycleQuantity;

	double maxAccOfAccSectionAfterRound;
	double maxAccOfDecSectionAfterRound;
	int errorCompenWholeCycleQuantity;
	double errorCompensationMaxAcc;
};
bool ScurveAccDecMethodPlanPretreat(double mDisplacement,
	double mStartVelocity,
	double mEndVelocity,
	double mCommandVelocity,
	double mVelocityLimit,
	double mAccLimit,
	double mJerkLimit,
	stInterpolationParamScurve *mInterParamScurve);

bool ComputeRoundError(double mDisplacement,
	double mJerkLimit,
	double mStartVelocity,
	double mEndVelocity,
	double mCycleTime,
	stInterpolationParamScurve *mInterParamScurve);

bool ComputeErrorCompensationLength(double mCycleTime,
	stInterpolationParamScurve *mInterParamScurve,
	int *mCompensatedCycleQuantity,
	double *mAccumulateCompensation,
	double *mCurrentPeriodCompenLength,
	bool *mCompensationFinished);

bool ComputeInterpolateLength(double mDisplacement, 
                              double mCycleTime,
                              double mJerkLimit,
                              double mCurrentPeriodCompenLength,
                              stInterpolationParamScurve *mInterParamScurve,
                              double *mAccumulateCompensation,
                              int *mSectionNum,
                              int *mInterpolationCycleCnt,
                              bool *mPlanningFinished,
                              double *mInterpolatedTime,
                              double *mSectionLengthAccumulate,
                              double *mTargetLineLength,
                              double *mCurrentSpeed,
                              bool *mInterpolationFinished);
/**********************************************************************************/

