#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h> 
#include <sensor_msgs/Joy.h>
using namespace std;

//设置速度限位
double BURGER_MAX_LIN_VEL = 0.5;  // 0.5
double BURGER_MAX_ANG_VEL = 0.345; // 0.345

class LogTeleop
{
   private:
    ros::Publisher pub;
    ros::Subscriber sub;
   public:
    ros::NodeHandle n;
    bool isCallback;
    geometry_msgs::Twist twist;

    LogTeleop();
   void LogCallback(const sensor_msgs::Joy::ConstPtr& Joy);
   void publish();
};
 LogTeleop::LogTeleop()
{
    pub=n.advertise<geometry_msgs::Twist>("/X1/cmd_vel",1) ;
    sub=n.subscribe<sensor_msgs::Joy>("/joy",20,&LogTeleop::LogCallback,this);
    isCallback = false;
}
 
void LogTeleop::LogCallback(const sensor_msgs::Joy::ConstPtr& Joy)
{
    isCallback = true;

    //直线速度
    if(Joy->axes[1]!=0)
    {
        twist.linear.x =(Joy->axes[1])*BURGER_MAX_LIN_VEL;
        //速度限制
        if(fabs(twist.linear.x)>BURGER_MAX_LIN_VEL)
        {
            twist.linear.x = BURGER_MAX_LIN_VEL;
        }
    }
    //转弯速度
    if(Joy->axes[3]!=0)
    {
        twist.angular.z =(Joy->axes[3])*BURGER_MAX_ANG_VEL;
        if(fabs(twist.angular.z)>BURGER_MAX_ANG_VEL)
        {
            twist.angular.z = BURGER_MAX_ANG_VEL;
        }
    }
    if(Joy->axes[1] == 0)
    {
        twist.linear.x =0;
    }
    if(Joy->axes[3] == 0)
    {
	twist.angular.z =0;
    }
    ROS_INFO("linear_x:%.3lf   angular_z:%.3lf",twist.linear.x,twist.angular.z);
    pub.publish(twist);
}

void LogTeleop::publish()
{
     //ROS_INFO("linear_x:%.3lf angular_z:%.3lf",twist.linear.x,twist.angular.z);
     //pub.publish(twist);
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "log_teleop");
    LogTeleop  logteleop;

    ros::Rate loop_rate(10);//指定循环的频率
    while(ros::ok())
    {
        if(!logteleop.isCallback)
        {
            //logteleop.publish();
            logteleop.isCallback = false;
        }
        logteleop.isCallback = false;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
