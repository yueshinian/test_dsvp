#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <motorDriven_ZQ/ControlRelay.h>
#include <iostream>
#include "cagvinfo.h"
#include "ccommandinfo.h"
#include "cmotorinfo.h"
#include "cprotocol.h"
#include "cupdpsocket.h"
#include "extern.h"
#include <pthread.h>
#include <vector>
#include <math.h>

#include "JogMotion.h"
#include "MotionSharedData.h"
//电机控制接收
class MotorControlHandller
{
public:
    void callback(const geometry_msgs::Twist::ConstPtr & message)
    {
        mCrtMsg = *message;
        mNewMsg = true;
        //ROS_INFO("get new msg!\n");
    }

    MotorControlHandller()
        : mCrtMsg()
        , mNewMsg(false)
    {
    }

    const bool newMsg()
    {
        return mNewMsg;
    }

    const geometry_msgs::Twist & fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

protected:
    geometry_msgs::Twist mCrtMsg;
    bool mNewMsg;
};
//继电器控制
class RelayControlHandller
{
public:
    void callback(const motorDriven_ZQ::ControlRelay::ConstPtr & message)
    {
        mCrtMsg = *message;
        mNewMsg = true;
        //ROS_INFO("get new msg!\n");
    }

    RelayControlHandller()
        : mCrtMsg()
        , mNewMsg(false)
    {
    }

    const bool newMsg()
    {
        return mNewMsg;
    }

    const motorDriven_ZQ::ControlRelay & fetchMsg()
    {
        mNewMsg = false;
        return mCrtMsg;
    }

protected:
    motorDriven_ZQ::ControlRelay mCrtMsg;
    bool mNewMsg;
};

