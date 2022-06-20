
#include "AccDecPlanningMethod.h"
#include "MotionSharedData.h"

/**************************************S曲线加减速**********************************/
/************************************************************************
Function:ScurveAccDecMethodPlanPretreat

Description:使用S曲线加减速方法规划加速段，匀速段和减速段时间

Input：
mDisplacement 位移
mStartVelocity 起点速度
mEndVelocity 终点速度
mCommandVelocity 指令速度
mAccLimit 最大加速度

Output：
*mInterParamLinearAccDecMethod  规划的各段时间与总时间,规划的累积位移,规划的节点速度

Version: LMPP 2.0
Return：计算成功返回true，存在错误返回false
************************************************************************/
bool ScurveAccDecMethodPlanPretreat(double mDisplacement,
	double mStartVelocity,
	double mEndVelocity,
	double mCommandVelocity,
	double mVelocityLimit,
	double mAccLimit,
	double mJerkLimit,
	stInterpolationParamScurve *mInterParamScurve)
{
	if (mDisplacement < ACCURACY)
	{
		mInterParamScurve->t1 = 0;
		mInterParamScurve->t2 = 0;
		mInterParamScurve->t3 = 0;
		mInterParamScurve->t4 = 0;
		mInterParamScurve->t5 = 0;
		mInterParamScurve->t6 = 0;
		mInterParamScurve->t7 = 0;
		mInterParamScurve->WholeTime = 0;

		return true;
	}

	//S曲线加减速各段时间变量
	double t1 = 0;
	double t2 = 0;
	double t3 = 0;
	double t4 = 0;
	double t5 = 0;
	double t6 = 0;
	double t7 = 0;

	//实际能达到的加速度
	double mMaxAcc1Actual = 0;
	double mMaxAcc2Actual = 0;

	//节点处速度
	double v1 = 0;
	double v2 = 0;
	double v3 = 0;
	double v4 = 0;
	double v5 = 0;
	double v6 = 0;

	//各段位移
	double s1 = 0;
	double s2 = 0;
	double s3 = 0;
	double s4 = 0;
	double s5 = 0;
	double s6 = 0;
	double s7 = 0;

	//判断指令速度与最大速度限制的关系
	//如果指令速度大于速度上线，则以速度上线作为最大速度
	if (mCommandVelocity > mVelocityLimit)
	{
		mCommandVelocity = mVelocityLimit;
	}

	//判断终止速度是否小于指令速度,如果是，则无解，返回false
    if (fabs(mEndVelocity - mCommandVelocity) > ACCURACY && mEndVelocity > mCommandVelocity)
	{
		return false;
	}
    else if (fabs(mEndVelocity - mCommandVelocity)<ACCURACY && mEndVelocity > mCommandVelocity)
	{
		mEndVelocity = mCommandVelocity;
	}

	/***********************************半个S曲线规划阶段*************************************/
	//判断起始速度是否大于指令速度，如果是，则进行S曲线减速规划
    if (fabs(mStartVelocity - mCommandVelocity) > ACCURACY && mStartVelocity > mCommandVelocity)
	{
		t1 = 0;
		t2 = 0;
		t3 = 0;

		//速度规划
		double mSpeedEndTemp = mStartVelocity - mAccLimit*mAccLimit / mJerkLimit;
        if (fabs(mSpeedEndTemp - mEndVelocity) < ACCURACY)
		{
			t5 = mAccLimit / mJerkLimit;
			t6 = 0;
			t7 = t5;
		}
        else if (fabs(mSpeedEndTemp - mEndVelocity) > ACCURACY && mSpeedEndTemp < mEndVelocity)
		{
            t5 = sqrt((mStartVelocity - mEndVelocity) / mJerkLimit);
			t6 = 0;
			t7 = t5;
		}
        else if (fabs(mSpeedEndTemp - mEndVelocity) > ACCURACY && mSpeedEndTemp > mEndVelocity)
		{
			t5 = mAccLimit / mJerkLimit;
			t6 = (mStartVelocity - mEndVelocity) / mAccLimit - mAccLimit / mJerkLimit;
			t7 = t5;
		}

		//位移规划
		double mDisplacementTemp = (mStartVelocity + mEndVelocity)*(t5 + t6 + t7) / 2;
        if (fabs(mDisplacementTemp - mDisplacement) < POSITION_AND_LENGTH_ERROR)
		{
			t4 = 0;
		}
        else if (fabs(mDisplacement - mDisplacementTemp) > POSITION_AND_LENGTH_ERROR && mDisplacement > mDisplacementTemp)
		{
			t4 = (mDisplacement - mDisplacementTemp) / mStartVelocity;
		}
        else if (fabs(mDisplacement - mDisplacementTemp) > POSITION_AND_LENGTH_ERROR && mDisplacement < mDisplacementTemp)
		{
			t4 = 0;
			t5 = 0;
			t6 = 0;
			t7 = 0;
			return false;
		}
	}

	/***********************************整个S曲线规划阶段*************************************/
	else
	{
        if (fabs(mStartVelocity - mCommandVelocity)<ACCURACY && mStartVelocity > mCommandVelocity)
		{
			mStartVelocity = mCommandVelocity;
		}
		/*************S曲线第一种情况计算,假设能都达到指令速度*****************/
		//第一种情况下的最大速度估计值
		double mMaxSpeed1 = mCommandVelocity;

		//加速过程有匀加速段的情况
		if ((mMaxSpeed1 - mStartVelocity) > mAccLimit*mAccLimit / mJerkLimit)
		{
			t1 = mAccLimit / mJerkLimit;
			t2 = (mMaxSpeed1 - mStartVelocity) / mAccLimit - t1;
			t3 = t1;
		}
		//加速段没有匀加速段的情况
		else
		{
            t1 = pow(((mMaxSpeed1 - mStartVelocity) / mJerkLimit), 0.5);
			t2 = 0;
			t3 = t1;
		}

		//减速段有匀减速段的情况
		if ((mMaxSpeed1 - mEndVelocity) > mAccLimit*mAccLimit / mJerkLimit)
		{
			t5 = mAccLimit / mJerkLimit;
			t6 = (mMaxSpeed1 - mEndVelocity) / mAccLimit - t5;
			t7 = t5;
		}
		//减速度段没有匀减速段的情况
		else
		{
            t5 = pow(((mMaxSpeed1 - mEndVelocity) / mJerkLimit), 0.5);
			t6 = 0;
			t7 = t5;
		}

		//判断是否有匀速段
		//先假设没有匀速段,此时t4 = 0,参考石川论文式3、式4
		t4 = 0;
		//首先确定实际能达到的最大加速度
		mMaxAcc1Actual = mJerkLimit*t1;
		mMaxAcc2Actual = mJerkLimit*t5;
		v1 = mStartVelocity + 0.5*mJerkLimit*t1*t1;
		v2 = v1 + mMaxAcc1Actual*t2;
		v3 = v2 + mMaxAcc1Actual*t3 - 0.5*mJerkLimit*t3*t3;
		v4 = v3;
		v5 = v4 - 0.5*mJerkLimit*t5*t5;
		v6 = v5 - mMaxAcc2Actual*t6;

		s1 = mStartVelocity*t1 + mJerkLimit*t1*t1*t1 / 6;
		s2 = s1 + v1*t2 + mMaxAcc1Actual*t2*t2 / 2;
		s3 = s2 + v2*t3 + mMaxAcc1Actual*t3*t3 / 2 - mJerkLimit*t3*t3*t3 / 6;
		s4 = s3 + v3*t4;
		s5 = s4 + v4*t5 - mJerkLimit*t5*t5*t5 / 6;
		s6 = s5 + v5*t6 - mMaxAcc2Actual*t6*t6 / 2;
		s7 = s6 + v6*t7 - mMaxAcc2Actual*t7*t7 / 2 + mJerkLimit*t7*t7*t7 / 6;

		//有匀速段的情况
        if (fabs(mDisplacement - s7)<ACCURACY)
		{
			t4 = 0;
		}
		//恰好没有匀速段且最大速度就是指令速度的情况
        else if ((fabs(mDisplacement - s7)>ACCURACY) && (s7 < mDisplacement))
		{
			t4 = (mDisplacement - s7) / v4;
		}
		//没有匀速段且指令速度大于实际所需最大速度的情况，进入第二种分析情况
		else
		{
			//没有匀速段，t4为0
			t4 = 0;

			//以下定义的时间变量均是以起始速度为计算起点时的时间变量，并不是最终的7段时间，还应根据始末点速度大小重新分配
			double t1Temp = 0;
			double t2Temp = 0;
			double t3Temp = 0;
			double t4Temp = 0;
			double t5Temp = 0;
			double t6Temp = 0;
			double t7Temp = 0;

			//假设情况下的位移
			double mTotalS = 0;
			double mTotalS1 = 0;
			double mTotalS2 = 0;

			//对始末点速度较大的一侧做完整的三角形加速
			//选择速度大的一侧作为vs，速度小的一侧做ve
			double vs = (mEndVelocity > mStartVelocity) ? mEndVelocity : mStartVelocity;
			double ve = mStartVelocity + mEndVelocity - vs;

			//第二种情况下的最大速度估计值，这里要考虑是否三角形加速导致速度超过指令速度
			double mMaxSpeed2 = vs + mAccLimit*mAccLimit / mJerkLimit;
			//vs经三角形加速能够达到mMaxSpeed2，mMaxSpeed2小于mCommandVelocity
			if (mMaxSpeed2 <= mCommandVelocity)
			{
				t1Temp = mAccLimit / mJerkLimit;
				t2Temp = 0;
				t3Temp = t1Temp;
				t4Temp = 0;
				t5Temp = t1Temp;
				t6Temp = (mMaxSpeed2 - ve - mAccLimit*mAccLimit / mJerkLimit) / mAccLimit;
				t7Temp = t1Temp;

				//计算该假设情况下的位移
				mTotalS1 = (vs + mMaxSpeed2) / 2 * (t1Temp + t3Temp);
				mTotalS2 = (ve + mMaxSpeed2) / 2 * (t5Temp + t6Temp + t7Temp);
				mTotalS = mTotalS1 + mTotalS2;

				//位移比较
                if (fabs(mTotalS - mDisplacement) < ACCURACY)
				{
					t1 = t1Temp;
					t2 = t2Temp;
					t3 = t3Temp;
					t4 = t4Temp;
					t5 = t5Temp;
					t6 = t6Temp;
					t7 = t7Temp;
				}
                else if ((fabs(mTotalS - mDisplacement) > ACCURACY) && (mTotalS < mDisplacement))
				{
					//第二种情况计算的位移小于实际位移情况下，需要增加速度大的一侧的匀加速段，重新计算最大速度
					//这里虽然最大速度mMaxSpeed22相对于mMaxSpeed2增大了，但不会超过mCommandVelocity，因为如果出现这种情况，为满足位移要求，就会出现匀速段，这样在第一种情况计算中就应该就出来了
                    //double mMaxSpeed22 = -mAccLimit*mAccLimit/(2*mJerkLimit) + sqrt(mDisplacement*mAccLimit + (vs*vs + ve*ve)/2 + mAccLimit*mAccLimit*(vs + ve)/(2*mJerkLimit) + pow(mAccLimit,4)/(4*mJerkLimit*mJerkLimit));
					//石川公式(10)
                    double mMaxSpeed22 = -mAccLimit*mAccLimit / (2 * mJerkLimit) + sqrt(mDisplacement*mAccLimit + (vs*vs + ve*ve) / 2 - mAccLimit*mAccLimit*(vs + ve) / (2 * mJerkLimit) + pow(mAccLimit, 4) / (4 * mJerkLimit*mJerkLimit));
					t1Temp = mAccLimit / mJerkLimit;
					t3Temp = t1Temp;
					t2Temp = (mMaxSpeed22 - vs - mAccLimit*t1Temp) / mAccLimit;
					t4Temp = 0;
					t5Temp = t1Temp;
					t7Temp = t1Temp;
					t6Temp = (mMaxSpeed22 - ve - mAccLimit*t5Temp) / mAccLimit;
					if (vs == mStartVelocity)
					{
						t1 = t1Temp;
						t2 = t2Temp;
						t3 = t3Temp;
						t4 = t4Temp;
						t5 = t5Temp;
						t6 = t6Temp;
						t7 = t7Temp;
					}
					else
					{
						t1 = t7Temp;
						t2 = t6Temp;
						t3 = t5Temp;
						t4 = t4Temp;
						t5 = t3Temp;
						t6 = t2Temp;
						t7 = t1Temp;
					}
				}
			}

			//第二种情况下的假设位移仍然大于实际位移时，进入第三种情况
			//或者vs经三角形加速超过指令速度，这时就演变为第一种情况，由于已经由第一种情况进入第二种情况，所以此时求得的位移比大于实际位移，因此进入第三种情况
            if ((mMaxSpeed2 > mCommandVelocity) || (fabs(mTotalS - mDisplacement) > ACCURACY) && (mTotalS > mDisplacement))
			{
				//重新定义计算起点与终点
				vs = (mEndVelocity < mStartVelocity) ? mEndVelocity : mStartVelocity;
				ve = mStartVelocity + mEndVelocity - vs;

				//第三种情况下的最大速度估计值
				double mMaxSpeed3 = vs + mAccLimit*mAccLimit / mJerkLimit;

				//这里要考虑是否三角形加速导致速度超过指令速度，
				if (mMaxSpeed3 <= mCommandVelocity)
				{
					//判断最大速度与ve的关系
					if (mMaxSpeed3 >= ve)
					{
						//速度小的一侧做完整三角形加减速，计算各段时间
						t1Temp = mAccLimit / mJerkLimit;
						t2Temp = 0;
						t3Temp = t1Temp;
						t4Temp = 0;
                        t5Temp = pow((mMaxSpeed3 - ve) / mJerkLimit, 0.5);
						t6Temp = 0;
						t7Temp = t5Temp;

						//计算此种情况下位移
						mTotalS1 = (vs + mMaxSpeed3) / 2 * (t1Temp + t3Temp);
						mTotalS2 = (ve + mMaxSpeed3) / 2 * (t5Temp + t7Temp);
						mTotalS = mTotalS1 + mTotalS2;
                        if (fabs(mTotalS - mDisplacement) < ACCURACY)
						{
							if (vs == mStartVelocity)
							{
								t1 = t1Temp;
								t2 = t2Temp;
								t3 = t3Temp;
								t4 = t4Temp;
								t5 = t5Temp;
								t6 = t6Temp;
								t7 = t7Temp;
							}
							else
							{
								t1 = t7Temp;
								t2 = t6Temp;
								t3 = t5Temp;
								t4 = t4Temp;
								t5 = t3Temp;
								t6 = t2Temp;
								t7 = t1Temp;
							}
						}
					}
					//继续增大速度
                    if ((fabs(mTotalS - mDisplacement) > ACCURACY) && (mTotalS < mDisplacement) || mMaxSpeed3 < ve)
					{
						//需要使用二分法求解最大速度，其取值范围在mMaxSpeed2与mMaxSpeed3之间，此时必有解
						//定义迭代误差变量
						double mInterativeErr1 = 0;

						//定义迭代次数计数变量
						double mInterativeTimes1 = 0;

						//定义速度取值上下线
						double mSpeedUpperLimit1 = mMaxSpeed2;
						double mSpeedLowerLimit1 = mMaxSpeed3;

						//需要满足最大速度大于等于末端速度
						//首先将最大速度增加到ve，如果此时的位移大于实际位移，则肯定误解
						double mMaxSpeed32 = ve;
						mTotalS1 = 1 / (2 * mAccLimit)*mMaxSpeed32*mMaxSpeed32 + mAccLimit / (2 * mJerkLimit)*mMaxSpeed32 + mAccLimit / mJerkLimit*vs - vs*vs / (2 * mAccLimit) - vs*mAccLimit / (2 * mJerkLimit);
                        mTotalS2 = (ve + mMaxSpeed32)*pow(((mMaxSpeed32 - ve) / mJerkLimit), 0.5);
						mTotalS = mTotalS1 + mTotalS2;
						//无解情况
						if (mTotalS > mDisplacement)
						{
							return false;
						}
						//继续增大最大速度
						else if (mTotalS < mDisplacement)
						{
							//定义迭代所求的最大速度，初始值为mMaxSpeed2与mMaxSpeed3的中间速度
							mMaxSpeed32 = (mSpeedUpperLimit1 + mSpeedLowerLimit1) / 2;
							while (mInterativeTimes1 <= INTERATINV_TIMES)
							{
								if (mMaxSpeed32 >= ve)
								{
									mTotalS1 = 1 / (2 * mAccLimit)*mMaxSpeed32*mMaxSpeed32 + mAccLimit / (2 * mJerkLimit)*mMaxSpeed32 + mAccLimit / mJerkLimit*vs - vs*vs / (2 * mAccLimit) - vs*mAccLimit / (2 * mJerkLimit);
                                    mTotalS2 = (ve + mMaxSpeed32)*pow(((mMaxSpeed32 - ve) / mJerkLimit), 0.5);
									mTotalS = mTotalS1 + mTotalS2;
									mInterativeErr1 = mTotalS - mDisplacement;
								}

								//根据位移随速度单调变化的性质获得下一次迭代的速度取值范围
                                if ((fabs(mInterativeErr1) > INTERATIVE_ERROR) && mInterativeErr1 < 0 || mMaxSpeed32 < ve)
								{
									//重新计算迭代速度下限
									mSpeedLowerLimit1 = (mSpeedUpperLimit1 + mSpeedLowerLimit1) / 2;

								}
                                else if ((fabs(mInterativeErr1) > INTERATIVE_ERROR) && mInterativeErr1 > 0)
								{
									//重新计算迭代速度上限
									mSpeedUpperLimit1 = (mSpeedUpperLimit1 + mSpeedLowerLimit1) / 2;

								}
								else
								{
									break;
								}
								//重新计算迭代速度
								mMaxSpeed32 = (mSpeedUpperLimit1 + mSpeedLowerLimit1) / 2;

								//计数迭代次数
								mInterativeTimes1++;
							}
						}
						//对迭代结果进行判断
						//无解情况
                        if (mInterativeTimes1 >= INTERATINV_TIMES && fabs(mInterativeErr1) > INTERATIVE_ERROR)
						{
							return false;
						}
						//根据迭代得到的最大速度计算各段时间
						t1Temp = mAccLimit / mJerkLimit;
						t2Temp = (mMaxSpeed32 - vs - mAccLimit*t1Temp) / mAccLimit;
						t3Temp = t1Temp;
						t4Temp = 0;
                        t5Temp = pow(((mMaxSpeed32 - ve) / mJerkLimit), 0.5);
						t6Temp = 0;
						t7Temp = t5Temp;

						//时间段赋值
						if (vs == mStartVelocity)
						{
							t1 = t1Temp;
							t2 = t2Temp;
							t3 = t3Temp;
							t4 = t4Temp;
							t5 = t5Temp;
							t6 = t6Temp;
							t7 = t7Temp;
						}
						else
						{
							t1 = t7Temp;
							t2 = t6Temp;
							t3 = t5Temp;
							t4 = t4Temp;
							t5 = t3Temp;
							t6 = t2Temp;
							t7 = t1Temp;
						}
					}
				}

				//减小速度
				//存在两种情况需要减小速度
                if (mMaxSpeed3 > mCommandVelocity || (fabs(mTotalS - mDisplacement) > INTERATIVE_ERROR) && (mTotalS > mDisplacement) && mMaxSpeed3 > ve)
				{
					//需要使用二分法求解最大速度，其取值范围在mMaxSpeed3与max{mStartVelocity，mEndVelocity}之间，即mMaxSpeed3与ve之间
					//首先验证在[ve,mMaxSpeed3]之间是否解，即当最大速度为ve时，计算得到的位移deltaS1是否小于实际位移S
					//如果mMaxSpeed3大于mCommandVelocity，则迭代上线为mCommandVelocity
					
					double deltaS1 = (vs + ve)*mAccLimit / mJerkLimit;
					//double deltaS1 = (vs + ve)*sqrt((ve-vs)/mJerkLimit);
					//无解情况
					if (deltaS1 > mDisplacement)
					{
						return false;
					}
					else
					{
						//定义迭代误差变量
						double mInterativeErr2 = 0;

						//定义迭代次数计数变量
						double mInterativeTimes2 = 0;

						//定义速度取值上下线
						double mSpeedUpperLimit2 = mMaxSpeed3;
						if (mMaxSpeed3 > mCommandVelocity)
						{
							mSpeedUpperLimit2 = mCommandVelocity;
						}
						double mSpeedLowerLimit2 = ve;

						//定义迭代所求的最大速度，初始值为mMaxSpeed2与mMaxSpeed3的中间速度
						double mMaxSpeed33 = (mSpeedUpperLimit2 + mSpeedLowerLimit2) / 2;
						while (mInterativeTimes2 <= INTERATINV_TIMES)
						{
                            mTotalS1 = (vs + mMaxSpeed33)*pow(((mMaxSpeed33 - vs) / mJerkLimit), 0.5);
                            mTotalS2 = (ve + mMaxSpeed33)*pow(((mMaxSpeed33 - ve) / mJerkLimit), 0.5);
							mTotalS = mTotalS1 + mTotalS2;
							mInterativeErr2 = mTotalS - mDisplacement;

							//根据位移随速度单调变化的性质获得下一次迭代的速度取值范围
                            if ((fabs(mInterativeErr2) > INTERATIVE_ERROR) && mInterativeErr2 < 0)
							{
								//重新计算迭代速度下限
								mSpeedLowerLimit2 = (mSpeedUpperLimit2 + mSpeedLowerLimit2) / 2;

							}
                            else if ((fabs(mInterativeErr2) > INTERATIVE_ERROR) && mInterativeErr2 > 0)
							{
								//重新计算迭代速度上限
								mSpeedUpperLimit2 = (mSpeedUpperLimit2 + mSpeedLowerLimit2) / 2;

							}
							else
							{
								break;
							}
							//重新计算迭代速度
							mMaxSpeed33 = (mSpeedUpperLimit2 + mSpeedLowerLimit2) / 2;

							//计数迭代次数
							mInterativeTimes2++;
						}
						//对迭代结果进行判断
                        if (mInterativeTimes2 >= INTERATINV_TIMES && fabs(mInterativeErr2) > INTERATIVE_ERROR)
						{
							return false;
						}
						//根据迭代得到的最大速度计算各段时间
                        t1Temp = pow(((mMaxSpeed33 - vs) / mJerkLimit), 0.5);
						t2Temp = 0;
						t3Temp = t1Temp;
						t4Temp = 0;
                        t5Temp = pow(((mMaxSpeed33 - ve) / mJerkLimit), 0.5);
						t7Temp = t5Temp;

						//时间段赋值
						if (vs == mStartVelocity)
						{
							t1 = t1Temp;
							t2 = t2Temp;
							t3 = t3Temp;
							t4 = t4Temp;
							t5 = t5Temp;
							t6 = t6Temp;
							t7 = t7Temp;
						}
						else
						{
							t1 = t7Temp;
							t2 = t6Temp;
							t3 = t5Temp;
							t4 = t4Temp;
							t5 = t3Temp;
							t6 = t2Temp;
							t7 = t1Temp;
						}
					}
				}
			}
		}
	}
	mInterParamScurve->t1 = t1;
	mInterParamScurve->t2 = t2;
	mInterParamScurve->t3 = t3;
	mInterParamScurve->t4 = t4;
	mInterParamScurve->t5 = t5;
	mInterParamScurve->t6 = t6;
	mInterParamScurve->t7 = t7;
	mInterParamScurve->WholeTime = t1 + t2 + t3 + t4 + t5 + t6 + t7;
	return true;
}

/************************************************************************
Function:ComputeRoundError

Description：计算圆整误差

Input：
mDisplacement   位移
mJerkLimit      最大加加速度
mStartSpeed     起点速度
mEndSpeed       终点速度
mCycleTime      周期


Output：
*mInterParamScurve S曲线插补参数

Version:
Return：计算成功返回true，存在错误返回false
************************************************************************/
bool ComputeRoundError(double mDisplacement,
	double mJerkLimit,
	double mStartSpeed,
	double mEndSpeed,
	double mCycleTime,
	stInterpolationParamScurve *mInterParamScurve)
{
	//计算各段圆整后的周期数--根据时间t获得周期数，再处理成整数
	mInterParamScurve->cycleQuantity1 = (int)(mInterParamScurve->t1 / mCycleTime);
    if (fabs((mInterParamScurve->cycleQuantity1*mCycleTime + mCycleTime) - mInterParamScurve->t1) < ACCURACY)
	{
		mInterParamScurve->cycleQuantity1 = mInterParamScurve->cycleQuantity1 + 1;
	}

	mInterParamScurve->cycleQuantity2 = (int)(mInterParamScurve->t2 / mCycleTime);
    if (fabs((mInterParamScurve->cycleQuantity2*mCycleTime + mCycleTime) - mInterParamScurve->t2) < ACCURACY)
	{
		mInterParamScurve->cycleQuantity2 = mInterParamScurve->cycleQuantity2 + 1;
	}

	mInterParamScurve->cycleQuantity3 = (int)(mInterParamScurve->t3 / mCycleTime);
    if (fabs((mInterParamScurve->cycleQuantity3*mCycleTime + mCycleTime) - mInterParamScurve->t3) < ACCURACY)
	{
		mInterParamScurve->cycleQuantity3 = mInterParamScurve->cycleQuantity3 + 1;
	}

	mInterParamScurve->cycleQuantity4 = (int)(mInterParamScurve->t4 / mCycleTime);
    if (fabs((mInterParamScurve->cycleQuantity4*mCycleTime + mCycleTime) - mInterParamScurve->t4) < ACCURACY)
	{
		mInterParamScurve->cycleQuantity4 = mInterParamScurve->cycleQuantity4 + 1;
	}

	mInterParamScurve->cycleQuantity5 = (int)(mInterParamScurve->t5 / mCycleTime);
    if (fabs((mInterParamScurve->cycleQuantity5*mCycleTime + mCycleTime) - mInterParamScurve->t5) < ACCURACY)
	{
		mInterParamScurve->cycleQuantity5 = mInterParamScurve->cycleQuantity5 + 1;
	}

	mInterParamScurve->cycleQuantity6 = (int)(mInterParamScurve->t6 / mCycleTime);
    if (fabs((mInterParamScurve->cycleQuantity6*mCycleTime + mCycleTime) - mInterParamScurve->t6) < ACCURACY)
	{
		mInterParamScurve->cycleQuantity6 = mInterParamScurve->cycleQuantity6 + 1;
	}

	mInterParamScurve->cycleQuantity7 = (int)(mInterParamScurve->t7 / mCycleTime);
    if (fabs((mInterParamScurve->cycleQuantity7*mCycleTime + mCycleTime) - mInterParamScurve->t7) < ACCURACY)
	{
		mInterParamScurve->cycleQuantity7 = mInterParamScurve->cycleQuantity7 + 1;
	}

	//各段实际时间
	double t1 = mInterParamScurve->cycleQuantity1*mCycleTime;
	double t2 = mInterParamScurve->cycleQuantity2*mCycleTime;
	double t3 = mInterParamScurve->cycleQuantity3*mCycleTime;
	double t4 = mInterParamScurve->cycleQuantity4*mCycleTime;
	double t5 = mInterParamScurve->cycleQuantity5*mCycleTime;
	double t6 = mInterParamScurve->cycleQuantity6*mCycleTime;
	double t7 = mInterParamScurve->cycleQuantity7*mCycleTime;

	//计算时间圆整之后的实际最大加速度
	(mInterParamScurve->maxAccOfAccSectionAfterRound) = mJerkLimit*t1;
	(mInterParamScurve->maxAccOfDecSectionAfterRound) = mJerkLimit*t5;

	//计算圆整后各节点实际速度
	mInterParamScurve->vs = mStartSpeed;
	mInterParamScurve->v1 = mInterParamScurve->vs + mJerkLimit*t1*t1 / 2;
	mInterParamScurve->v2 = mInterParamScurve->v1 + (mInterParamScurve->maxAccOfAccSectionAfterRound)*t2;
	mInterParamScurve->v3 = mInterParamScurve->v2 + (mInterParamScurve->maxAccOfAccSectionAfterRound)*t3 - mJerkLimit*t3*t3 / 2;
	mInterParamScurve->v4 = mInterParamScurve->v3;
	mInterParamScurve->v5 = mInterParamScurve->v4 - mJerkLimit*t5*t5 / 2;
	mInterParamScurve->v6 = mInterParamScurve->v5 - (mInterParamScurve->maxAccOfDecSectionAfterRound)*t6;
	mInterParamScurve->ve = mInterParamScurve->v6 - (mInterParamScurve->maxAccOfDecSectionAfterRound)*t7 + mJerkLimit*t7*t7 / 2;

	//圆整完的实际终点速度小于0的情况，做出调整
    while (fabs(mInterParamScurve->ve) > ACCURACY && mInterParamScurve->ve < 0)
	{
		//先减小第7段时间
		if (mInterParamScurve->cycleQuantity7 > 0)
		{
			mInterParamScurve->cycleQuantity7 = mInterParamScurve->cycleQuantity7 - 1;
			t7 = mInterParamScurve->cycleQuantity7*mCycleTime;
			mInterParamScurve->ve = mInterParamScurve->v6 - (mInterParamScurve->maxAccOfDecSectionAfterRound)*t7 + mJerkLimit*t7*t7 / 2;
		}
		//若第7段为0，,则减小第6段时间
		else if (mInterParamScurve->cycleQuantity6 > 0)
		{
			mInterParamScurve->cycleQuantity7 = 0;
			t7 = 0;

			mInterParamScurve->cycleQuantity6 = mInterParamScurve->cycleQuantity6 - 1;
			t6 = mInterParamScurve->cycleQuantity6*mCycleTime;
			mInterParamScurve->v6 = mInterParamScurve->v5 - (mInterParamScurve->maxAccOfDecSectionAfterRound)*t6;
			mInterParamScurve->ve = mInterParamScurve->v6;
		}
		//若第7段和第6段都为0，则减小第5段时间
		else if (mInterParamScurve->cycleQuantity5 > 0)
		{
			mInterParamScurve->cycleQuantity7 = 0;
			t7 = 0;

			mInterParamScurve->cycleQuantity6 = 0;
			t6 = 0;

			mInterParamScurve->cycleQuantity5 = mInterParamScurve->cycleQuantity5 - 1;
			t5 = mInterParamScurve->cycleQuantity5*mCycleTime;
			mInterParamScurve->v5 = mInterParamScurve->v4 - mJerkLimit*t5*t5 / 2;
			mInterParamScurve->ve = mInterParamScurve->v5;
			(mInterParamScurve->maxAccOfDecSectionAfterRound) = mJerkLimit*t5;
		}

		//出现一下情况，一般为计算错误造成
		if (mInterParamScurve->cycleQuantity5 <= 0
			&& mInterParamScurve->cycleQuantity6 <= 0
			&& mInterParamScurve->cycleQuantity7 <= 0)
		{
			return false;
		}
	}

	//计算在圆整时间下的各段累加位移
	mInterParamScurve->length1 = mInterParamScurve->vs*t1 + mJerkLimit*t1*t1*t1 / 6;
	mInterParamScurve->length2 = mInterParamScurve->length1 + mInterParamScurve->v1*t2 + (mInterParamScurve->maxAccOfAccSectionAfterRound)*t2*t2 / 2;
	mInterParamScurve->length3 = mInterParamScurve->length2 + mInterParamScurve->v2*t3 + (mInterParamScurve->maxAccOfAccSectionAfterRound)*t3*t3 / 2 - mJerkLimit*t3*t3*t3 / 6;
	mInterParamScurve->length4 = mInterParamScurve->length3 + mInterParamScurve->v3*t4;
	mInterParamScurve->length5 = mInterParamScurve->length4 + mInterParamScurve->v4*t5 - mJerkLimit*t5*t5*t5 / 6;
	mInterParamScurve->length6 = mInterParamScurve->length5 + mInterParamScurve->v5*t6 - (mInterParamScurve->maxAccOfDecSectionAfterRound)*t6*t6 / 2;
	mInterParamScurve->length7 = mInterParamScurve->length6 + mInterParamScurve->v6*t7 - (mInterParamScurve->maxAccOfDecSectionAfterRound)*t7*t7 / 2 + mJerkLimit*t7*t7*t7 / 6;

	//计算插补位移误差
	double mWholeErr = mDisplacement - mInterParamScurve->length7;

	//计算匀速段的补偿周期数（位移误差先用匀速段来补偿）
	int mCompensationPeriodNum = (int)(mWholeErr / mInterParamScurve->v4 / mCycleTime);
    if (fabs((mCompensationPeriodNum + 1)*mInterParamScurve->v4*mCycleTime - mWholeErr) < ACCURACY && mCompensationPeriodNum >= 0)
	{
		mCompensationPeriodNum = mCompensationPeriodNum + 1;
	}
    else if (fabs((mCompensationPeriodNum - 1)*mInterParamScurve->v4*mCycleTime - mWholeErr) < ACCURACY && mCompensationPeriodNum <= 0)
	{
		mCompensationPeriodNum = mCompensationPeriodNum - 1;
	}
	//更新匀速段时间
	mInterParamScurve->cycleQuantity4 = mInterParamScurve->cycleQuantity4 + mCompensationPeriodNum;
	if (mInterParamScurve->cycleQuantity4 < 0)
	{
		mInterParamScurve->cycleQuantity4 = 0;
	}

	mInterParamScurve->wholeCycleQuantity = mInterParamScurve->cycleQuantity1 + mInterParamScurve->cycleQuantity2 + mInterParamScurve->cycleQuantity3 + mInterParamScurve->cycleQuantity4 + mInterParamScurve->cycleQuantity5 + mInterParamScurve->cycleQuantity6 + mInterParamScurve->cycleQuantity7;

	//更新规划时间
	mInterParamScurve->t1 = mInterParamScurve->cycleQuantity1*mCycleTime;
	mInterParamScurve->t2 = mInterParamScurve->cycleQuantity2*mCycleTime;
	mInterParamScurve->t3 = mInterParamScurve->cycleQuantity3*mCycleTime;
	mInterParamScurve->t4 = mInterParamScurve->cycleQuantity4*mCycleTime;
	mInterParamScurve->t5 = mInterParamScurve->cycleQuantity5*mCycleTime;
	mInterParamScurve->t6 = mInterParamScurve->cycleQuantity6*mCycleTime;
	mInterParamScurve->t7 = mInterParamScurve->cycleQuantity7*mCycleTime;
	mInterParamScurve->WholeTime = mInterParamScurve->t1 + mInterParamScurve->t2 + mInterParamScurve->t3 + mInterParamScurve->t4 + mInterParamScurve->t5 + mInterParamScurve->t6 + mInterParamScurve->t7;

	//计算误差补偿时间--
	double mErrorCompensationTime = int(mInterParamScurve->wholeCycleQuantity / 8) * 8 * mCycleTime;
	(mInterParamScurve->errorCompenWholeCycleQuantity) = int(mInterParamScurve->wholeCycleQuantity / 8) * 8;

	//计算误差补偿最大加速度
	double mResidualError = mWholeErr - mCompensationPeriodNum*mInterParamScurve->v4*mCycleTime;    //需要修正梯形加减速方法补偿的剩余误差
	(mInterParamScurve->errorCompensationMaxAcc) = mResidualError / (mErrorCompensationTime*mErrorCompensationTime*(1.0 / 4 / PI + 1.0 / 8));

	return true;
}

/************************************************************************
Function:ComputeErrorCompensationLength

Description：计算修正梯形加减速方法计算的周期误差补偿长度

Input：
mCycleTime                         周期
mErrorCompenWholeCycleQuantity     误差补偿周期数
mErrorCompensationMaxAcc           误差补偿最大加速度

Output：
*mErrorCompensatedCycleQuantity         已补偿周期数
*mErrorCompensationAccumulateLastPeriod 误差补偿累计量
*mErrorCompensationCurrentPeriod        当前周期的误差补偿量
*mErrorCompensationFinished)            误差补偿完成标志位

Version:
Return：计算成功返回true，存在错误返回false
************************************************************************/
bool ComputeErrorCompensationLength(double mCycleTime,
                                    stInterpolationParamScurve *mInterParamScurve,
									int *mCompensatedCycleQuantity,
									double *mAccumulateCompensation,
									double *mCurrentPeriodCompenLength,
									bool *mCompensationFinished)
{
	double mT = mInterParamScurve->errorCompenWholeCycleQuantity*mCycleTime;

	int mCurrentCycleQuantity = (*mCompensatedCycleQuantity) + 1;
	double t = mCurrentCycleQuantity*mCycleTime;

	int m = mInterParamScurve->errorCompenWholeCycleQuantity / 8;
	double mS = 0;
	if (mCurrentCycleQuantity > 0 && mCurrentCycleQuantity < m)
	{
        mS = -(mT / 4 / PI)*(mT / 4 / PI)*mInterParamScurve->errorCompensationMaxAcc*sin(4 * PI / mT*t) + 1.0 / 4 / PI*mInterParamScurve->errorCompensationMaxAcc*mT*t;
	}
	else if (mCurrentCycleQuantity >= m && mCurrentCycleQuantity < 3 * m)
	{
		mS = 0.5*mInterParamScurve->errorCompensationMaxAcc*t*t + (1.0 / 4 / PI - 0.125)*mInterParamScurve->errorCompensationMaxAcc*mT*t + (1.0 / 128 - 1.0 / 16 / PI / PI)*mInterParamScurve->errorCompensationMaxAcc*mT*mT;
	}
	else if (mCurrentCycleQuantity >= 3 * m && mCurrentCycleQuantity < 5 * m)
	{
        mS = -(mT / 4 / PI)*(mT / 4 / PI)*mInterParamScurve->errorCompensationMaxAcc*cos(4 * PI*(t - 3 * mT / 8) / mT) + (1.0 / 4 / PI + 1.0 / 4)*mInterParamScurve->errorCompensationMaxAcc*mT*t - 1.0 / 16 * mInterParamScurve->errorCompensationMaxAcc*mT*mT;
	}
	else if (mCurrentCycleQuantity >= 5 * m && mCurrentCycleQuantity < 7 * m)
	{
		mS = -0.5*mInterParamScurve->errorCompensationMaxAcc*t*t + (1.0 / 4 / PI + 7.0 / 8)*mInterParamScurve->errorCompensationMaxAcc*mT*t + (1.0 / 16 / PI / PI - 33.0 / 128)*mInterParamScurve->errorCompensationMaxAcc*mT*mT;
	}
	else if (mCurrentCycleQuantity >= 7 * m && mCurrentCycleQuantity <= 8 * m)
	{
        mS = (mT / 4 / PI)*(mT / 4 / PI)*mInterParamScurve->errorCompensationMaxAcc*cos(4 * PI*(t - 7.0 / 8 * mT) / mT) + 1.0 / 4 / PI*mInterParamScurve->errorCompensationMaxAcc*mT*t + 1.0 / 8 * mInterParamScurve->errorCompensationMaxAcc*mT*mT;
	}
	else if (mCurrentCycleQuantity > 8 * m)
	{
		mS = (*mAccumulateCompensation);
	}

	(*mCurrentPeriodCompenLength) = mS - (*mAccumulateCompensation);
	(*mAccumulateCompensation) = mS;
	(*mCompensatedCycleQuantity) = mCurrentCycleQuantity;

	if ((*mCompensatedCycleQuantity) >= mInterParamScurve->errorCompenWholeCycleQuantity)
	{
		(*mCompensationFinished) = true;
	}

	return true;
}
/************************************************************************
Function:ComputeInterpolateLength

Description：计算周期插补长度

Input：
mDisplacement              位移
mCycleTime                 周期
mJerkLimit                 加加速度限制
mErrCompenCurrentPeriod    当前周期误差补偿量

Output：
*mErrCompenCurrentPeriodAccumulate,
*mInterParamS_curve,
*mInterParagraphNum,
*mInterCycleCnt,
bool *mInterPlanningFinished,
*mInterpolationFinished,
*mInterpolatedTime,
*mTargetLineLength,
*mSectionLengthAccumulate,
*mCurrentSpeed

Version:
Return：计算成功返回true，存在错误返回false
************************************************************************/
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
							  bool *mInterpolationFinished)
{
	if (!(*mPlanningFinished))
	{
		//初始化计数变量
		(*mSectionNum) = 1;
		(*mInterpolationCycleCnt) = 1;

		//初始化已插补时间变量
		(*mInterpolatedTime) = 0;

		//初始化目标插补位移
		(*mTargetLineLength) = 0;
		(*mSectionLengthAccumulate) = 0;

		//初始化误差补偿累计变量
		(*mAccumulateCompensation) = 0;

		(*mPlanningFinished) = true;
	}
	double mTargetCircularLengthTemp = 0;
	//计算位移的中间变量
	double t = 0;

	//计算中每个周期的补偿量暂存变量
	double mCompensation = 0;

	//计算当前周期所在位置的绝对位移长度值
	switch (*mSectionNum)
	{
	case 1:
		if (*mInterpolationCycleCnt > mInterParamScurve->cycleQuantity1)
		{
			(*mInterpolationCycleCnt) = 1;
			(*mSectionNum)++;
			(*mSectionLengthAccumulate) = (*mTargetLineLength);
			(*mAccumulateCompensation) = 0;
			//之后进入case 2
		}
		else
		{
			//计算补偿量
			(*mAccumulateCompensation) = (*mAccumulateCompensation) + mCurrentPeriodCompenLength;
			mCompensation = (*mAccumulateCompensation);

			t = (*mInterpolationCycleCnt)*mCycleTime;
			mTargetCircularLengthTemp = mInterParamScurve->vs*t + mJerkLimit*t*t*t / 6 + mCompensation;
			(*mInterpolationCycleCnt)++;
			(*mInterpolatedTime) = (*mInterpolatedTime) + mCycleTime;

			//检测插补长度是否超限
			if (mTargetCircularLengthTemp >= mDisplacement)
			{
				(*mInterpolationCycleCnt) = 0;
				(*mSectionNum) = 0;

				(*mInterpolationFinished) = true;

				//插补长度为总位移长度
				mTargetCircularLengthTemp = mDisplacement;
			}

			//检测插补周期数是否超限
			int mInterpolatedCycleCnt = (int)((*mInterpolatedTime + mCycleTime / 10) / mCycleTime);
			if (mInterpolatedCycleCnt >= mInterParamScurve->wholeCycleQuantity)
			{
				(*mInterpolationCycleCnt) = 0;
				(*mSectionNum) = 0;

				(*mInterpolationFinished) = true;

				//弧长为总弧长
				mTargetCircularLengthTemp = mDisplacement;
			}

			break;
		}


	case 2:
		if (*mInterpolationCycleCnt > mInterParamScurve->cycleQuantity2)
		{
			(*mInterpolationCycleCnt) = 1;
			(*mSectionNum)++;
			(*mSectionLengthAccumulate) = (*mTargetLineLength);
			(*mAccumulateCompensation) = 0;
			//之后进入case 3
		}
		else
		{
			//计算补偿量
			(*mAccumulateCompensation) = (*mAccumulateCompensation) + mCurrentPeriodCompenLength;
			mCompensation = (*mAccumulateCompensation);

			t = (*mInterpolationCycleCnt)*mCycleTime;
			mTargetCircularLengthTemp = (*mSectionLengthAccumulate) + mInterParamScurve->v1*t + mInterParamScurve->maxAccOfAccSectionAfterRound*t*t / 2 + mCompensation;
			(*mInterpolationCycleCnt)++;
			(*mInterpolatedTime) = (*mInterpolatedTime) + mCycleTime;

			//检测弧长是否超限
			if (mTargetCircularLengthTemp >= mDisplacement)
			{
				(*mInterpolationCycleCnt) = 0;
				(*mSectionNum) = 0;

				(*mInterpolationFinished) = true;

				//弧长为总弧长
				mTargetCircularLengthTemp = mDisplacement;
			}

			//检测插补周期数是否超限
			int mInterpolatedCycleCnt = (int)((*mInterpolatedTime + mCycleTime / 10) / mCycleTime);
			if (mInterpolatedCycleCnt >= mInterParamScurve->wholeCycleQuantity)
			{
				(*mInterpolationCycleCnt) = 0;
				(*mSectionNum) = 0;

				(*mInterpolationFinished) = true;

				//弧长为总弧长
				mTargetCircularLengthTemp = mDisplacement;
			}

			break;
		}


	case 3:
		if (*mInterpolationCycleCnt > mInterParamScurve->cycleQuantity3)
		{
			(*mInterpolationCycleCnt) = 1;
			(*mSectionNum)++;
			(*mSectionLengthAccumulate) = (*mTargetLineLength);
			(*mAccumulateCompensation) = 0;
			//之后进入case 4
		}
		else
		{
			//计算补偿量
			(*mAccumulateCompensation) = (*mAccumulateCompensation) + mCurrentPeriodCompenLength;
			mCompensation = (*mAccumulateCompensation);

			t = (*mInterpolationCycleCnt)*mCycleTime;
			mTargetCircularLengthTemp = (*mSectionLengthAccumulate) + mInterParamScurve->v2*t + mInterParamScurve->maxAccOfAccSectionAfterRound*t*t / 2 - mJerkLimit*t*t*t / 6 + mCompensation;
			(*mInterpolationCycleCnt)++;
			(*mInterpolatedTime) = (*mInterpolatedTime) + mCycleTime;

			//检测弧长是否超限
			if (mTargetCircularLengthTemp >= mDisplacement)
			{
				(*mInterpolationCycleCnt) = 0;
				(*mSectionNum) = 0;

				(*mInterpolationFinished) = true;

				//弧长为总弧长
				mTargetCircularLengthTemp = mDisplacement;
			}

			//检测插补周期数是否超限
			int mInterpolatedCycleCnt = (int)((*mInterpolatedTime + mCycleTime / 10) / mCycleTime);
			if (mInterpolatedCycleCnt >= mInterParamScurve->wholeCycleQuantity)
			{
				(*mInterpolationCycleCnt) = 0;
				(*mSectionNum) = 0;

				(*mInterpolationFinished) = true;

				//弧长为总弧长
				mTargetCircularLengthTemp = mDisplacement;
			}

			break;
		}

	case 4:
		if (*mInterpolationCycleCnt > mInterParamScurve->cycleQuantity4)
		{
			(*mInterpolationCycleCnt) = 1;
			(*mSectionNum)++;
			(*mSectionLengthAccumulate) = (*mTargetLineLength);
			(*mAccumulateCompensation) = 0;
			//之后进入case 5
		}
		else
		{
			//计算补偿量
			(*mAccumulateCompensation) = (*mAccumulateCompensation) + mCurrentPeriodCompenLength;
			mCompensation = (*mAccumulateCompensation);

			t = (*mInterpolationCycleCnt)*mCycleTime;
			mTargetCircularLengthTemp = (*mSectionLengthAccumulate) + mInterParamScurve->v3*t + mCompensation;
			(*mInterpolationCycleCnt)++;
			(*mInterpolatedTime) = (*mInterpolatedTime) + mCycleTime;

			//检测弧长是否超限
			if (mTargetCircularLengthTemp >= mDisplacement)
			{
				(*mInterpolationCycleCnt) = 0;
				(*mSectionNum) = 0;

				(*mInterpolationFinished) = true;

				//弧长为总弧长
				mTargetCircularLengthTemp = mDisplacement;
			}

			//检测插补周期数是否超限
			int mInterpolatedCycleCnt = (int)((*mInterpolatedTime + mCycleTime / 10) / mCycleTime);
			if (mInterpolatedCycleCnt >= mInterParamScurve->wholeCycleQuantity)
			{
				(*mInterpolationCycleCnt) = 0;
				(*mSectionNum) = 0;

				(*mInterpolationFinished) = true;

				//弧长为总弧长
				mTargetCircularLengthTemp = mDisplacement;
			}

			break;
		}

	case 5:
		if (*mInterpolationCycleCnt > mInterParamScurve->cycleQuantity5)
		{
			(*mInterpolationCycleCnt) = 1;
			(*mSectionNum)++;
			(*mSectionLengthAccumulate) = (*mTargetLineLength);
			(*mAccumulateCompensation) = 0;
			//之后进入case 6
		}
		else
		{
			//计算补偿量
			(*mAccumulateCompensation) = (*mAccumulateCompensation) + mCurrentPeriodCompenLength;
			mCompensation = (*mAccumulateCompensation);

			t = (*mInterpolationCycleCnt)*mCycleTime;
			mTargetCircularLengthTemp = (*mSectionLengthAccumulate) + mInterParamScurve->v4*t - mJerkLimit*t*t*t / 6 + mCompensation;
			(*mInterpolationCycleCnt)++;
			(*mInterpolatedTime) = (*mInterpolatedTime) + mCycleTime;

			//检测弧长是否超限
			if (mTargetCircularLengthTemp >= mDisplacement)
			{
				(*mInterpolationCycleCnt) = 0;
				(*mSectionNum) = 0;

				(*mInterpolationFinished) = true;

				//弧长为总弧长
				mTargetCircularLengthTemp = mDisplacement;
			}

			//检测插补周期数是否超限
			int mInterpolatedCycleCnt = (int)((*mInterpolatedTime + mCycleTime / 10) / mCycleTime);
			if (mInterpolatedCycleCnt >= mInterParamScurve->wholeCycleQuantity)
			{
				(*mInterpolationCycleCnt) = 0;
				(*mSectionNum) = 0;

				(*mInterpolationFinished) = true;

				//弧长为总弧长
				mTargetCircularLengthTemp = mDisplacement;
			}

			break;
		}

	case 6:
		if (*mInterpolationCycleCnt > mInterParamScurve->cycleQuantity6)
		{
			(*mInterpolationCycleCnt) = 1;
			(*mSectionNum)++;
			(*mSectionLengthAccumulate) = (*mTargetLineLength);
			(*mAccumulateCompensation) = 0;
			//之后进入case 7
		}
		else
		{
			//计算补偿量
			(*mAccumulateCompensation) = (*mAccumulateCompensation) + mCurrentPeriodCompenLength;
			mCompensation = (*mAccumulateCompensation);

			t = (*mInterpolationCycleCnt)*mCycleTime;
			mTargetCircularLengthTemp = (*mSectionLengthAccumulate) + mInterParamScurve->v5*t - mInterParamScurve->maxAccOfDecSectionAfterRound*t*t / 2 + mCompensation;
			(*mInterpolationCycleCnt)++;
			*mInterpolatedTime = (*mInterpolatedTime) + mCycleTime;

			//检测弧长是否超限
			if (mTargetCircularLengthTemp >= mDisplacement)
			{
				(*mInterpolationCycleCnt) = 0;
				(*mSectionNum) = 0;

				(*mInterpolationFinished) = true;

				//弧长为总弧长
				mTargetCircularLengthTemp = mDisplacement;
			}

			//检测插补周期数是否超限
			int mInterpolatedCycleCnt = (int)((*mInterpolatedTime + mCycleTime / 10) / mCycleTime);
			if (mInterpolatedCycleCnt >= mInterParamScurve->wholeCycleQuantity)
			{
				(*mInterpolationCycleCnt) = 0;
				(*mSectionNum) = 0;

				(*mInterpolationFinished) = true;

				//弧长为总弧长
				mTargetCircularLengthTemp = mDisplacement;
			}

			break;
		}

	case 7:
	{
			  //计算补偿量
			  (*mAccumulateCompensation) = (*mAccumulateCompensation) + mCurrentPeriodCompenLength;
			  mCompensation = (*mAccumulateCompensation);

			  t = (*mInterpolationCycleCnt)*mCycleTime;
			  mTargetCircularLengthTemp = (*mSectionLengthAccumulate) + mInterParamScurve->v6*t - mInterParamScurve->maxAccOfDecSectionAfterRound*t*t / 2 + mJerkLimit*t*t*t / 6 + mCompensation;
			  //检测弧长是否超限
			  if (mTargetCircularLengthTemp >= mDisplacement)
			  {
				  (*mInterpolationCycleCnt) = 0;
				  (*mSectionNum) = 0;

				  //最后一次计算的时间
				  (*mInterpolatedTime) = (*mInterpolatedTime) + mCycleTime;
				  (*mInterpolationFinished) = true;

				  //弧长为总弧长
				  mTargetCircularLengthTemp = mDisplacement;

				  break;
			  }

			  //检测插补是否完成
			  if (*mInterpolationCycleCnt >= mInterParamScurve->cycleQuantity7)
			  {
				  (*mInterpolationCycleCnt) = 0;
				  (*mSectionNum) = 0;
				  //最后一次计算的时间
				  (*mInterpolatedTime) = (*mInterpolatedTime) + mCycleTime;
				  (*mInterpolationFinished) = true;

				  //弧长为总弧长
				  mTargetCircularLengthTemp = mDisplacement;

				  break;
			  }
			  else
			  {
				  (*mInterpolationCycleCnt)++;
				  (*mInterpolatedTime) = (*mInterpolatedTime) + mCycleTime;
				  break;
			  }
	}

	default:
		return false;
		break;
	}
	double mTargetLengthLastPeriod = (*mTargetLineLength);
	*mTargetLineLength = mTargetCircularLengthTemp;

	//计算当前周期的速度
	(*mCurrentSpeed) = ((*mTargetLineLength) - mTargetLengthLastPeriod) / mCycleTime;

	return true;
}

/**********************************************************************************/



