#pragma once

#include "MotionSharedData.h"

inline double Max(double a,double b) {if(a>=b) return a;return b;}
inline double Min(double a,double b) {if(a<=b) return a;return b;}
//S曲线各节点的速度
struct stScurveNodeSpeed
{
	double vs;
	double v1;
	double v2;
	double v3;
	double v4;
	double v5;
	double v6;
	double ve;
};
//S曲线各节点比例函数值
struct stScureDiffPropFunc
{
	double diffPropStart;
	double diffProp1;
	double diffProp2;
	double diffProp4;
	double diffProp5;
	double diffProp6;
	double diffPropEnd;
};
//S曲线各节点关系映射函数值
struct stScurePropFunc
{
	double scureProp1;
	double scureProp2;
	double scureProp3;
	double scureProp5;
	double scureProp6;
	double scureProp7;
};
//变周期映射S曲线变量
struct stScureVarCycleParam
{
	double diffPropMax; //等周期S曲线一次求导最大值
	double diff2PropMax; //二次求导最大值
	double diff3PropMax; //三次求导最大值

	double wholeTime;  //S曲线规划时间
	double integerTime; //规划时间取整
	double integerBottomTime; //触底圆整时间

	double intervalTime; //积分时间
	double halfTime; //凹一半时间

	stScurveNodeSpeed scurveNodeSpeed; //节点速度
	stScureDiffPropFunc scureDiffProp; //曲线一次微分
	stScurePropFunc scureProp; //等周期S曲线
};
//变周期插补参数
struct stPVTInterpolationParam
{
	double equalCycleTime; //等周期时间
	int direction; //运动方向
	int interParagraphNum; //变周期段号
	double proFuncSum; //曲线总映射值
	double interCycleSpeed;  //插补周期速度
	double interCycleSpeedLast; //上一周期的速度
	double sectionTargetLength; //相对位移
	double sectionLengthAccumulate; //绝对位移
	double sectionLengthAccumulateLast; //上一周期的绝对位移
	bool bottomTimeFinished; //触底时间完成标志
	bool interPlanningFinished; //插补完成标志位
};
//S曲线插补结构体
struct stInterpolationParamPVT
{
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

	//加加速度
	double jerkLimitOfAcc;
	double jerkLimitOfDec;
	//加速度
	double accLimitOfAcc;
	double accLimitOfDec;

	//插补规划标志位
	bool interPlanning;

	//S曲线段数
	int sectionNum;
	//插补计数
	int interCycleCnt;
	//目标长度
	double targetLength;
	//插补完成标志位
	bool interFinished;
};

/************************************************************************
Function:ComputeEachSectionTime  速度规划

Description:按S曲线加减速模式，以弧长作为运动总位移计算各运动阶段时间

Input：运动位移，即弧长displacement
始末点速度mSpeedStart,mSpeedEnd
指令速度mCommandSped
最大允许速度mSpeedLimit
最大允许加速度accLimit
最大允许加加速度jerkLimit

Output：S曲线的各段时间*mScurveSectionTime

Return：计算成功返回true
计算失败返回false
************************************************************************/
bool ComputeEachSectionTime(double displacement,
	double mSpeedStart,
	double mSpeedEnd,
	double mCommandSpeed,
	double mSpeedLimit,
	double accLimit,
	double jerkLimit,
	stScurveSectionTime *mScurveSectionTime);
//计算加速时间
void ComputeAccTimeKnowVsToVe(double startSpeed,
	double endSpeed,
	double speedLimit,
	double accLimit,
	double jerkLimit,
	double &displacement,
	stScurveSectionTime &scurveSectionTime);
bool DoSmallLineVelocityPlanning(double displacement,
	double startSpeed,
	double &endSpeed,
	double commandSpeed,
	double speedLimit,
	double accLimit,
	double jerkLimit,
	stScurveSectionTime &scurveSectionTime,
	int &errorCode);
/************************************************************************
Function:ComputeAccSegmentMotionParameters

Description:计算加速段运动参数：加速段末端速度，加速段位移，加速时间

Input：起点速度 mStartSpeed
末端速度 mEndSpeed
指令速度 mCommandSpeed
最大加速度限制 mAccLimit
最大加加速度限制 mJerkLimit

Output：加速位移 displacement
加速段时间 mScurveSectionTime

Return：计算成功返回true
计算失败返回false
************************************************************************/
bool ComputeAccSegmentMotionParameters(double mStartSpeed,
	double mEndSpeed,
	double mCommandSpeed,
	double mAccLimit,
	double mJerkLimit,
	double &displacement,
	stScurveSectionTime &mScurveSectionTime);
/************************************************************************
Function:ComputeAccSectionTime

Description:计算加速时间

Input：起点速度 mStartSpeed
末点速度 mEndSpeed
指令速度 mCommandSpeed
最大加速度限制 mAccLimit
最大加加速度限制 mJerkLimit

Output：加速时间 *mScurveSectionTime

Return：计算成功返回true
计算失败返回false
************************************************************************/
bool ComputeAccSectionTime(double mStartSpeed,
	double mEndSpeed,
	double mCommandSpeed,
	double mAccLimit,
	double mJerkLimit,
	stScurveSectionTime *mScurveSectionTime);
/************************************************************************
Function:ComputeMaxDisplacement

Description:按S曲线加减速模式，以时间T为限制条件，计算最大位移

Input：始末点速度mSpeedA,mSpeedC
指令速度mCommandSpeed
最大允许速度mSpeedLimit
最大允许加速度mAccLimit
最大允许加加速度mJerkLimit
PVT指定时间 mWholeTime

Output：S曲线的各段时间*mScurveSectionTime
运动位移，即弧长*mDisplacement
最大速度 *mMaxSpeed

Return：计算成功返回true
计算失败返回false
************************************************************************/
bool ComputeMaxDisplacement(double mSpeedStart,
	double mSpeedEnd,
	double mCommandSpeed,
	double mSpeedLimit,
	double mAccLimit,
	double mJerkLimit,
	double mWholeTime,
	stScurveSectionTime *mScurveSectionTime,
	double *mDisplacement,
	double *mMaxSpeed);
/************************************************************************
Function:ComputeMinDisplacement

Description:按S曲线加减速模式，以时间T为限制条件，计算最小位移

Input：始末点速度mSpeedA,mSpeedC
指令速度mCommandSpeed
最大允许速度mSpeedLimit
最大允许加速度mAccLimit
最大允许加加速度mJerkLimit
PVT指定时间 mWholeTime

Output：S曲线的各段时间*mScurveSectionTime
运动位移，即弧长*mDisplacement
最小速度 *minSpeedTemp

Return：计算成功返回true
计算失败返回false
************************************************************************/
bool ComputeMinDisplacement(double mSpeedStart,
	double mSpeedEnd,
	double mCommandSpeed,
	double mSpeedLimit,
	double mAccLimit,
	double mJerkLimit,
	double mWholeTime,
	stScurveSectionTime *mScurveSectionTime,
	double *mDisplacement,
	double *minSpeedTemp);
/************************************************************************
Function:ComputeScurveTimeOfPVT

Description:已知P、V、T,将位姿同步中较短时间拉长至时间一致

Input：起点速度 startSpeed
末端速度 endSpeed
指令速度 commandSpeed
最大加速度限制 accLimit
最大加加速度限制 jerkLimit
给定时间 givenTime
目标位移 displacement

Output：实际的加加速度 jerkLimitOfAcc、jerkLimitOfDec
给定时间下的S曲线各段时间 *scurveSectionTime

Return：计算成功返回true
计算失败返回false
************************************************************************/
bool ComputeScurveTimeOfPVT(double startSpeed,
	double endSpeed,
	double speedLimit,
	double commandSpeed,
	double accLimit,
	double jerkLimit,
	double givenTime,
	double displacement,
	double &jerkLimitOfAcc,
	double &jerkLimitOfDec,
	stScurveSectionTime *scurveSectionTime);
/************************************************************************
Function:ComputeEqualTimeDecedEndSpeed

Description:已知P、Vs、T,计算相同时间下的末端速度，如果相同时间下没有在位移内，只能降低末端速度值

Input：起点速度 startSpeed
末端最大速度 endMaxSpeed
末端最小速度 endMinSpeed
指令速度 commandSpeed
最大加速度限制 accLimit
最大加加速度限制 jerkLimit
同步时间 synchronTime
最短规划时间 minScurveSectionTime
目标位移 displacement

Output:目标速度 targetEndSpeed
Return：计算成功返回true
计算失败返回false
************************************************************************/
bool ComputeEqualTimeDecedEndSpeed(double startSpeed,
	double endMaxSpeed,
	double endMinSpeed,
	double speedLimit,
	double commandSpeed,
	double accLimit,
	double jerkLimit,
	double synchronTime,
	stScurveSectionTime minScurveSectionTime,
	double displacement,
	double &targetEndSpeed);
/************************************************************************
Function:ComputeErrorCompensation

Description:以修正梯形模式计算当前周期的误差补偿量

Input：插补周期 mCycleTime
误差补偿总总周期数 mErrorCompenWholeCycleQuantity
误差补偿最大加速度 mErrorCompensationMaxAcc
已补偿周期数*mErrorCompensatedCycleQuantity
上一周期的误差补偿累加量 *mErrorCompensationAccumulateCurrentPeriod

Output：当前周期的误差补偿量 *mErrorCompensationLastPeriod
误差补偿完成标志位 *mErrorCompensationFinished
************************************************************************/
void ComputeErrorCompensation(double mCycleTime,
	int mErrorCompenWholeCycleQuantity,
	double mErrorCompensationMaxAcc,
	int *mErrorCompensatedCycleQuantity,
	double *mErrorCompensationAccumulateLastPeriod,
	double *mErrorCompensationCurrentPeriod,
	bool *mErrorCompensationFinished);
/************************************************************************
Function:ComputeInterpolateLength

Description：计算周期插补长度

Input：displacement        位移
startSpeed          起始速度
cycleTime		   插补周期
jerkLimitOfAcc      加速段加加速度限制
jerkLimitOfDec      减速段加加速度限制
scurveSectionTime   S曲线各段时间

InOutput: interParamScurve      S曲线插补参数
sectionNum            插补段数
interpolationCycleCnt 插补循环次数
planningFinished      规划完成标志位
interpolationFinished 插补完成标志位

Output：  targetLength   目标位移
targetSpeed    目标速度

Version:
Return：计算成功返回true，存在错误返回false
************************************************************************/
bool ComputeInterpolateLength(double displacement,
	double startSpeed,
	double cycleTime,
	double jerkLimitOfAcc,
	double jerkLimitOfDec,
	stScurveSectionTime scurveSectionTime,
        stInterpolationParamPVT *interParamScurve,
	int *sectionNum,
	int *interpolationCycleCnt,
	bool *planningFinished,
	double *targetLength,
	double *targetSpeed,
	bool *interpolationFinished);
/************************************************************************
Function:ComputeProportionalFunction

Description:计算对应各阶段的映射关系函数、比例函数和速度值

Input：初始速度：mStartSpeed
最大允许加加速度：mJerkLimit
加速段最大允许加速度：mMaxAccOfAccSection
减速段最大允许加速度：mMaxAccOfDecSection
S曲线的各段时间：mScurveSectionTime

Output：PVT插补参数：mPVTInterpolationParam
变周期参数:mScureVarCycleParam

Return：计算成功返回true
计算失败返回false
************************************************************************/
bool ComputeProportionalFunction(double mStartSpeed,
	double mJerkLimit,
	double mMaxAccOfAccSection,
	double mMaxAccOfDecSection,
	stPVTInterpolationParam &mPVTInterpolationParam,
	stScureVarCycleParam &mScureVarCycleParam,
	stScurveSectionTime mScurveSectionTime);
/************************************************************************
Function:ComputeVelocityMappingInterva

Description:计算经关系映射后除S曲线节点外的速度值

Input： S曲线各节点速度值：mInterCycleSpeed
最大允许加加速度mJerkLimit
加速段最大允许加速度：mMaxAccOfAccSection
减速段最大允许加速度：mMaxAccOfDecSection
映射关系函数：mPropFunc
映射关系函数导数：比例函数：mDiffPropFunc
S曲线的各段时间：mSectionTime

Output：映射后S曲线各段速度值：mInterCycleSpeed
************************************************************************/
void ComputeVelocityMappingInterva(double &mInterCycleSpeed,
	double mJerkLimit,
	double mMaxAccOfAccSection,
	double mMaxAccOfDecSection,
	double mPropFunc,
	double mDiffPropFunc,
	stScurveNodeSpeed scurveNodeSpeed,
	stScurveSectionTime mSectionTime);
/************************************************************************
Function:ComputeVelocityMappingEndpoint

Description:计算经关系映射各S曲线节点末尾段的速度值

Input：S曲线各节点速度值：scurveNodeSpeed
最大允许加加速度mJerkLimit
加速段最大允许加速度：mMaxAccOfAccSection
减速段最大允许加速度：mMaxAccOfDecSection
映射关系函数：mPropFunc
映射关系函数导数：比例函数：mDiffProp1 mDiffProp2
S曲线的各段时间：mSectionTime

Output：S曲线映射后各段节点速度值：mEndNodeSpeed1 mEndNodeSpeed2
************************************************************************/
void ComputeVelocityMappingEndpoint(double &mEndNodeSpeed1,
	double &mEndNodeSpeed2,
	double mJerkLimit,
	double mMaxAccOfAccSection,
	double mMaxAccOfDecSection,
	double mPropFunc,
	double mDiffProp1,
	double mDiffProp2,
	stScurveNodeSpeed mScurveNodeSpeed,
	stScurveSectionTime mSectionTime);
/************************************************************************
Function:ComputeVarCycleInterpolation[变周期]

Description:计算每一周期的目标长度、速度、加速度和加加速度

Input：目标位移：displacment
起始速度：startSpeed
加速段最大加速度：mMaxAccOfAccSection
减速段最大加速度：mMaxAccOfDecSection
最大允许加加速度mJerkLimit
变周期参数 mScureVarCycleParam
S曲线规划时间 mScurveSectionTime

InOut: PVT插补参数：mPVTInterpolationParam
插补周期：mInterCycleCnt

Output：目标长度：mTargetLength
目标速度：mTargetSpeed
目标加速度：mTargetAcc
PVT完成标志位：mLengthInterFinished
返回值：return true
************************************************************************/
bool ComputeVarCycleInterpolation(double displacment,
	double startSpeed,
	double mMaxAccOfAccSection,
	double mMaxAccOfDecSection,
	double mJerkLimit,
	stScureVarCycleParam mScureVarCycleParam,
	stScurveSectionTime mScurveSectionTime,
	int &mInterCycleCnt,
	stPVTInterpolationParam &mPVTInterpolationParam,
	double &mTargetLength,
	double &mTargetSpeed,
	double &mTargetAcc,
	bool &mLengthInterFinished);
/************************************************************************
Function:ComputeDecEndSpeed

Description:计算减速到目标点处的速度

Input：小线段弧长 mArcLength
减速起点速度 mStartSpeed
终点处的指令速度 mCommandEndSpeed
加速度限制 mAccLimit
加加速度限制 mJerkLimit

Output：末端速度 &mEndSpeed

Return：计算成功返回true
计算失败返回false
************************************************************************/
bool ComputeDecEndSpeed(double mArcLength,
        double mStartSpeed,
        double mCommandEndSpeed,
        double mAccLimit,
        double mJerkLimit,
        double &mEndSpeed);
