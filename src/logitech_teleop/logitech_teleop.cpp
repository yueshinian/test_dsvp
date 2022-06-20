#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h> 
#include <sensor_msgs/Joy.h>
using namespace std;

//设置速度限位
double BURGER_MAX_LIN_VEL = 0.5;  // 0.5
double BURGER_MAX_ANG_VEL = 0.345; // 0.345

float joySpeed = 0;
float joyYaw = 0;
float joyTime=0;
bool isNewMsg = false;

void joystickHandler(const sensor_msgs::Joy::ConstPtr& Joy)  // axes[4] to axes[1]
{
  joyTime = ros::Time::now().toSec();
  isNewMsg=true;
  joySpeed = 0;
  joyYaw = 0;
  //直线速度
  if(Joy->axes[1]!=0){
    joySpeed =(Joy->axes[1])*BURGER_MAX_LIN_VEL;
    //速度限制
    if(fabs(joySpeed)>BURGER_MAX_LIN_VEL){
        joySpeed = BURGER_MAX_LIN_VEL;
    }
  }
    //转弯速度
  if(Joy->axes[3]!=0){
    joyYaw =(Joy->axes[3])*BURGER_MAX_ANG_VEL;
    if(fabs(joyYaw)>BURGER_MAX_ANG_VEL){
        joyYaw = BURGER_MAX_ANG_VEL;
    }
  }
  if(Joy->axes[1] == 0){
        joySpeed =0;
    }
  if(Joy->axes[3] == 0){
    joyYaw =0;
  }
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "log_teleop");
    ros::NodeHandle nh;
    ros::Publisher pubTwist =nh.advertise<geometry_msgs::Twist>("/X1/cmd_vel",1);
    ros::Subscriber subTwist =nh.subscribe<sensor_msgs::Joy>("/joy",5,joystickHandler);
    ros::Rate loop_rate(100);//指定循环的频率
    geometry_msgs::Twist twist;
    while(ros::ok())
    {
        if(isNewMsg){
            isNewMsg=false;
            twist.linear.x=joySpeed;
            twist.angular.z=joyYaw;
        }else{
            twist.linear.x=0;
            twist.angular.z=0;
        }
        pubTwist.publish(twist);
        ROS_INFO("linear_x:%.3lf   angular_z:%.3lf",twist.linear.x,twist.angular.z);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
