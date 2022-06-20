/*
exploration_with_graph_planner.cpp
the interface for drrt planner

Created and maintained by Hongbiao Zhu (hongbiaz@andrew.cmu.edu)
05/25/2020
 */

#include <chrono>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <mutex>

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>


#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "dsvplanner/clean_frontier_srv.h"
#include "dsvplanner/dsvplanner_srv.h"
#include "graph_planner/GraphPlannerCommand.h"
#include "graph_planner/GraphPlannerStatus.h"

using namespace std::chrono;
using namespace std;
#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"

geometry_msgs::Point wayPoint;
geometry_msgs::Point wayPoint_pre;
graph_planner::GraphPlannerCommand graph_planner_command;
std_msgs::Float32 effective_time;
std_msgs::Float32 total_time;

bool simulation = false;   // control whether use graph planner to follow path
bool begin_signal = false; // trigger the planner
bool gp_in_progress = false;
bool wp_state = false;
bool return_home = false;
double current_odom_x = 0;
double current_odom_y = 0;
double current_odom_z = 0;
double previous_odom_x = 0;
double previous_odom_y = 0;
double previous_odom_z = 0;
double dtime = 0.0;
double init_x = 2;
double init_y = 0;
double init_z = 2;
double return_home_threshold = 1.5;
double robot_moving_threshold = 2;
std::string map_frame = "map";

steady_clock::time_point plan_start;
steady_clock::time_point plan_over;
steady_clock::duration time_span;

//lym
std::mutex mtx;
vector<geometry_msgs::PointStamped> homevector;
int addtionalExploration=1;
bool noRotAtStop = false;
bool twoWayDrive = true;
bool recOdom = false;
bool autonomyMode = false;
int try_return_home=0;
int go_straight=0;
bool go_straight_signal=false;
bool explorationIt = true;
bool start_exploration = false;
double pre_odom_x = 0;
double pre_odom_y = 0;
double pre_odom_z = 0;
int clearing_cloud_count=0;
ros::Publisher joy_waypoint_pub;
ros::Publisher pubClearing;
void homePointCallback(const geometry_msgs::PointStamped::ConstPtr &point_msg){
  std::lock_guard<std::mutex> lck (mtx);
  geometry_msgs::PointStamped home_point;
    home_point.point.x = current_odom_x;
    home_point.point.y = current_odom_y;
    home_point.point.z = current_odom_z;
    home_point.header.frame_id = "/map";
    homevector.emplace_back(home_point);
    for(int i=0;i<homevector.size();++i){
      geometry_msgs::PointStamped temp = homevector[i];
      std::cout<<"this is "<< i <<" th point"<<endl;
      std::cout<<temp.point.x<<endl;
       std::cout<<temp.point.y<<endl;
    }
}
void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy) {//lym,产生探索开始信号，手动更新 waypoint，避免摆动
  if (joy->buttons[4] > 0.5) {
    begin_signal = true;
  }
  double joySpeed = sqrt( joy->axes[1] * joy->axes[1]);
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[1] == 0) joySpeed = 0;
  double joyYaw = joy->axes[3];
  if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

  if (joy->axes[1] < 0 && !twoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }

  if(joy->buttons[4] > 0.5 && autonomyMode){
    autonomyMode = false;
  }
  else if(joy->buttons[4] > 0.5 && !autonomyMode){
    autonomyMode = true;
  }
  /*
  if(joy->buttons[3]>0.5)
  {
      go_straight++;
      if(go_straight>3)
        go_straight=3;
  }
  if(joy->buttons[0]>0.5)
  {
    go_straight--;
    if(go_straight < -3)
      go_straight=-3;
  }
  if(joy->buttons[1]>0.5)
    go_straight=0;
  if(joy->buttons[2]>0.5)
  {
    geometry_msgs::PointStamped wp;
    wp.header.frame_id = map_frame;
    wp.header.stamp = ros::Time::now();
    wp.point.x = current_odom_x+go_straight;
    wp.point.y = current_odom_y;
    wp.point.z = current_odom_z;
    joy_waypoint_pub.publish(wp);
  }
  */
}

void excute_clearCloud (const ros::TimerEvent& e){
    double dis_x = abs(current_odom_x - pre_odom_x);
    double dis_y = abs(current_odom_y - pre_odom_y);
    double dis_z = abs(current_odom_z - pre_odom_z);
    //printf(cursclean);
    //std::cout<<"distance is "<<sqrt(dis_x*dis_x + dis_y*dis_y  + dis_z*dis_z)<< ' '<<"dis_x is "<<dis_x<<std::endl;
    //printf(cursup);
    if( sqrt(dis_x*dis_x + dis_y*dis_y  + dis_z*dis_z) < 0.5 && (dis_x < 0.6) && start_exploration){
          std_msgs::Float32 clear_size;
          clear_size.data=8.0;
          pubClearing.publish(clear_size);
          ROS_WARN("clearing terrain cloud %d",++clearing_cloud_count);
          if(return_home){
            gp_in_progress=false;
            ROS_WARN("next return home plan!");
          }

    }
    pre_odom_x = current_odom_x;
    pre_odom_y = current_odom_y;
    pre_odom_z = current_odom_z;
}

void gp_status_callback(
    const graph_planner::GraphPlannerStatus::ConstPtr &msg) {
  if (msg->status == graph_planner::GraphPlannerStatus::STATUS_IN_PROGRESS)
    gp_in_progress = true;
  else {
    gp_in_progress = false;
  }
}

void waypoint_callback(const geometry_msgs::PointStamped::ConstPtr &msg) {
  wayPoint = msg->point;
  wp_state = true;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
  current_odom_x = msg->pose.pose.position.x;
  current_odom_y = msg->pose.pose.position.y;
  current_odom_z = msg->pose.pose.position.z;
  recOdom=true;
}

void begin_signal_callback(const std_msgs::Bool::ConstPtr &msg) {
  begin_signal = msg->data;
}

bool robotPositionChange() {
  double dist = sqrt(
      (current_odom_x - previous_odom_x) * (current_odom_x - previous_odom_x) +
      (current_odom_y - previous_odom_y) * (current_odom_y - previous_odom_y) +
      (current_odom_z - previous_odom_z) * (current_odom_z - previous_odom_z));
  if (dist < robot_moving_threshold)
    return false;
  previous_odom_x = current_odom_x;
  previous_odom_y = current_odom_y;
  previous_odom_z = current_odom_z;
  return true;
}

void ReturnHomeCallback(const std_msgs::Bool::ConstPtr &msg){
  return_home=msg->data;
}

void gotoxy(int x, int y) { printf("%c[%d;%df", 0x1B, y, x); }

int main(int argc, char **argv) {
  ros::init(argc, argv, "exploration");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  ros::Publisher waypoint_pub =
      nh.advertise<geometry_msgs::PointStamped>("/way_point", 5);
  ros::Publisher gp_command_pub =
      nh.advertise<graph_planner::GraphPlannerCommand>("/graph_planner_command",
                                                       1);
  //  ros::Publisher effective_plan_time_pub =
  //  nh.advertise<geometry_msgs::PointStamped>("/effective_plan_time", 1);
  ros::Publisher effective_plan_time_pub =
      nh.advertise<std_msgs::Float32>("/runtime", 1);
  ros::Publisher total_plan_time_pub =
      nh.advertise<std_msgs::Float32>("/totaltime", 1);
  ros::Subscriber gp_status_sub =
      nh.subscribe<graph_planner::GraphPlannerStatus>("/graph_planner_status",
                                                      1, gp_status_callback);
  ros::Subscriber waypoint_sub = nh.subscribe<geometry_msgs::PointStamped>(
      "/way_point", 1, waypoint_callback);
  ros::Subscriber odom_sub =
      nh.subscribe<nav_msgs::Odometry>("/state_estimation", 1, odom_callback);
  ros::Subscriber begin_signal_sub = nh.subscribe<std_msgs::Bool>(
      "/start_exploring", 1, begin_signal_callback);
  ros::Publisher stop_signal_pub =
      nh.advertise<std_msgs::Bool>("/stop_exploring", 1);

  //lym
  ros::Publisher homepoint_pub = nh.advertise<geometry_msgs::PointStamped>("/homePoint", 1);//local planner
  ros::Publisher homepoint_pub_ = nh.advertise<geometry_msgs::PointStamped>("/homePointVector", 1);//local planner
  //ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);//lym
  joy_waypoint_pub=nh.advertise<geometry_msgs::PointStamped>("/way_point",1);//lym
  pubClearing =nh.advertise<std_msgs::Float32>("/cloud_clearing", 5);//lym
  ros::Timer clearCloud_timer = nh.createTimer(ros::Duration(5), excute_clearCloud);//lym
  ros::Subscriber subReturnHome = nh.subscribe<std_msgs::Bool>("/returnHomeStatu", 1,ReturnHomeCallback);
  ros::Subscriber subHomePoint = nh.subscribe("/homePointVector",1, homePointCallback);

  nhPrivate.getParam("simulation", simulation);
  nhPrivate.getParam("/interface/dtime", dtime);
  nhPrivate.getParam("/interface/initX", init_x);
  nhPrivate.getParam("/interface/initY", init_y);
  nhPrivate.getParam("/interface/initZ", init_z);
  nhPrivate.getParam("/interface/returnHomeThres", return_home_threshold);
  nhPrivate.getParam("/interface/robotMovingThres", robot_moving_threshold);
  nhPrivate.getParam("/interface/tfFrame", map_frame);
  nhPrivate.getParam("/interface/autoExp", begin_signal);

  ros::Duration(1.0).sleep();
  ros::spinOnce();
  //lym
  while (!begin_signal ) {
    ros::Duration(0.5).sleep();
    ros::spinOnce();
    if(!recOdom)
      ROS_INFO("Waiting for Odometry");
    else
      ROS_WARN("Waiting for Exploration Start");
  }

  ROS_INFO("Starting the planner: Performing initialization motion");
  geometry_msgs::PointStamped wp;
  wp.header.frame_id = map_frame;
  wp.header.stamp = ros::Time::now();
  wp.point.x = init_x + current_odom_x;
  wp.point.y = init_y + current_odom_y;
  wp.point.z = init_z + current_odom_z;
  //lym
  geometry_msgs::PointStamped homePoint;
  homePoint.point.x = current_odom_x;
  homePoint.point.y = current_odom_y;
  homePoint.point.z = current_odom_z;
  homePoint.header.frame_id = map_frame;
  homepoint_pub_.publish(homePoint);

  ros::Duration(0.5).sleep(); // wait for sometime to make sure waypoint can be
                              // published properly

  waypoint_pub.publish(wp);
  bool wp_ongoing = true;
  int init_count=0;
  while (wp_ongoing) { // Keep publishing initial waypoint until the robot
                       // reaches that point
    ++init_count;
    ros::Duration(0.1).sleep();
    ros::spinOnce();
    waypoint_pub.publish(wp);
    double dist =
        sqrt((wp.point.x - current_odom_x) * (wp.point.x - current_odom_x) +
             (wp.point.y - current_odom_y) * (wp.point.y - current_odom_y));
    if (dist < 0.5)
      wp_ongoing = false;
    if(init_count>100)
       wp_ongoing = false;
  }

  ros::Duration(1.0).sleep();

  std::cout << std::endl
            << "\033[1;32mExploration Started\033[0m\n"
            << std::endl;
  total_time.data = 0;
  plan_start = steady_clock::now();
  // Start planning: The planner is called and the computed goal point sent to
  // the graph planner.
  int iteration = 0;
  //lym
  int begin_signal_count=0;
  while (ros::ok()) {
    if(!begin_signal){
      begin_signal_count++;
      ros::Duration(0.1).sleep();
      ros::spinOnce();
      if(begin_signal_count>10){
        std::cout << "\033[1;32mPause Exploration! Waiting restart... \033[0m" <<std::endl;
        begin_signal_count=0;
      }
      continue;
    }
    if (!return_home) {
      /*
      if (iteration != 0) {
        for (int i = 0; i < 8; i++) {
          printf(cursup);
          printf(cursclean);
        }
      }
      */
      std::cout << "Planning iteration " << iteration << std::endl;
      dsvplanner::dsvplanner_srv planSrv;
      dsvplanner::clean_frontier_srv cleanSrv;
      planSrv.request.header.stamp = ros::Time::now();
      planSrv.request.header.seq = iteration;
      planSrv.request.header.frame_id = map_frame;
      if (ros::service::call("drrtPlannerSrv", planSrv)) {
        if (planSrv.response.goal.size() ==
            0) { // usually the size should be 1 if planning successfully
          ros::Duration(1.0).sleep();
          continue;
        }

        if (planSrv.response.mode.data == 2) {
          return_home = true;
          std::cout << std::endl
                    << "\033[1;32mExploration completed, returning home\033[0m"
                    << std::endl
                    << std::endl;
          effective_time.data = 0;
          effective_plan_time_pub.publish(effective_time);
          continue;
        } else {
          return_home = false;
          plan_over = steady_clock::now();
          time_span = plan_over - plan_start;
          effective_time.data = float(time_span.count()) *
                                steady_clock::period::num /
                                steady_clock::period::den;
          effective_plan_time_pub.publish(effective_time);
        }
        total_time.data += effective_time.data;
        total_plan_time_pub.publish(total_time);

        if (!simulation) { // when not in simulation mode, the robot will go to
                           // the goal point according to graph planner
          for (size_t i = 0; i < planSrv.response.goal.size(); i++) {
            graph_planner_command.command =
                graph_planner::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
            graph_planner_command.location = planSrv.response.goal[i];
            gp_command_pub.publish(graph_planner_command);
            ros::Duration(dtime).sleep(); // give sometime to graph planner for
                                          // searching path to goal point
            ros::spinOnce();              // update gp_in_progree
            int count = 200;
            previous_odom_x = current_odom_x;
            previous_odom_y = current_odom_y;
            previous_odom_z = current_odom_z;
            while (gp_in_progress) { // if the waypoint keep the same for 20
                                     // (200*0.1)
              ros::Duration(0.1).sleep(); // seconds, then give up the goal
              wayPoint_pre = wayPoint;
              ros::spinOnce();
              bool robotMoving = robotPositionChange();
              if (robotMoving) {
                count = 200;
              } else {
                count--;
              }
              if (count <= 0) { // when the goal point cannot be reached, clean
                                // its correspoinding frontier if there is
                cleanSrv.request.header.stamp = ros::Time::now();
                cleanSrv.request.header.frame_id = map_frame;
                ros::service::call("cleanFrontierSrv", cleanSrv);
                ros::Duration(0.1).sleep();
                break;
              }
            }
          }
          graph_planner_command.command =
              graph_planner::GraphPlannerCommand::COMMAND_DISABLE;
          gp_command_pub.publish(graph_planner_command);
        } else { // simulation mode is used when testing this planning algorithm
                 // with bagfiles where robot will
          // not move to the planned goal. When in simulation mode, robot will
          // keep replanning every two seconds
          for (size_t i = 0; i < planSrv.response.goal.size(); i++) {
            graph_planner_command.command =
                graph_planner::GraphPlannerCommand::COMMAND_GO_TO_LOCATION;
            graph_planner_command.location = planSrv.response.goal[i];
            gp_command_pub.publish(graph_planner_command);
            ros::Duration(2).sleep();
            break;
          }
        }
        plan_start = steady_clock::now();
      } else {
        std::cout << "Cannot call drrt planner." << std::flush;

        ros::Duration(1.0).sleep();
      }
      iteration++;
    } else {
      ros::spinOnce();
      if (sqrt(
           pow(current_odom_x-homePoint.point.x,2) + 
           pow(current_odom_y-homePoint.point.y,2) +
           pow(current_odom_z-homePoint.point.z,2) )
           <= return_home_threshold) {
             if(homevector.size()>0){
               homevector.pop_back();
               homePoint=homevector.back();
               homepoint_pub.publish(homevector.back());
             }else{
               printf(cursclean);
        std::cout << "\033[1;32mReturn home completed\033[0m" << std::endl;
        printf(cursup);

        std_msgs::Bool stop_exploring;
        stop_exploring.data = true;
        stop_signal_pub.publish(stop_exploring);
             }
      }
      else{
        if(homevector.size()>0){
          homePoint=homevector.back();
          homepoint_pub.publish(homevector.back());
        } 
        if(addtionalExploration%2){
          addtionalExploration++;
          return_home=false;
          continue;
        }
        if(!gp_in_progress){//lym，额外的try to return home，防止一次return 失败
         printf(cursup);
         printf(cursclean);
         std::cout << "\033[1;32mTry Return home \033[0m" << ++try_return_home<<std::endl;
         graph_planner_command.command = graph_planner::GraphPlannerCommand::COMMAND_GO_TO_ORIGIN;
         gp_command_pub.publish(graph_planner_command);
         ros::Duration(dtime*2).sleep(); 
         ros::spinOnce();              // update gp_in_progree
         int time_count =50;        //5s检测是是否waypoint未改变
         while (gp_in_progress && return_home) { 
              ros::Duration(0.1).sleep(); 
              wayPoint_pre = wayPoint;
              ros::spinOnce();
              bool robotMoving = robotPositionChange();
              if (robotMoving) {
                time_count = 50;
              } else {
                time_count--;
              }
              if (time_count <= 0) { 
                break;
              }
         }
      }
      }
      ros::Duration(0.1).sleep();
    }
  }
}
