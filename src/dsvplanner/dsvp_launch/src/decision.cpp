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
#include <string>
#include <time.h>
#include <thread>
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
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>  //文件输入输出
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_types.h>  //点类型相关定义
#include <pcl/visualization/cloud_viewer.h>  //点类型相关定义
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>

#include "dsvplanner/clean_frontier_srv.h"
#include "dsvplanner/dsvplanner_srv.h"
#include "graph_planner/GraphPlannerCommand.h"
#include "graph_planner/GraphPlannerStatus.h"

#include "volumetric_msgs/LoadMap.h"
#include "volumetric_msgs/SaveMap.h"

ros::Publisher begin_signal_pub;
ros::Publisher pubHome;
ros::Publisher pubReturnHome;
ros::Publisher initialposePub;
std_msgs::Bool begin_signal;
geometry_msgs::PoseWithCovarianceStamped    reLocationInitPose;
bool setReLocationInitPose = false;
std::string file_path_ = "/home/yz/catkin_ws/octomapSet/octomap";
std::string file_path_pcl_ = "/home/yz/catkin_ws/octomapSet/pcl";
std::string global_map_path ;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloudMap(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr loadCloudMap(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr cloudMapDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr globalMap(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr globalMapDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> mapDwzFilter;
double pclMapResolution=0.1;
double globalMapResolution = 0.1;
bool savePclMapSingnl=false;
bool saveGlobalMap = false;
int mode=0;
double current_odom_x = 0;
double current_odom_y = 0;
double current_odom_z = 0;
void exploration_control(){
    if(begin_signal.data){
          begin_signal.data=false;
      }else{
          begin_signal.data=true;
      }
      begin_signal_pub.publish(begin_signal);
}
bool saveOctomap(){
  volumetric_msgs::SaveMap  saveMapSrv;
  std::string file_path=file_path_ + std::to_string(rand()%RAND_MAX) + ".bt";
  saveMapSrv.request.file_path=file_path;
  if(ros::service::call("/dsvplanner/save_map", saveMapSrv)){
      std::cout<<"Succeed to save the octomap ------"<<file_path<<std::endl;
      return true;
  }else{
      std::cout<<"Fail to save the octomap!"<<std::endl;
      return false;
  }
}
bool loadOctomap(){
    volumetric_msgs::LoadMap loadMapSrv;
    loadMapSrv.request.file_path = file_path_;
    if(ros::service::call("/dsvplanner/load_map", loadMapSrv)){
        std::cout<<"Succeed to load the octomap"<<std::endl;
        return true;
    }else{
        std::cout<<"Fail to load the octomap"<<std::endl;
        return false;
    }
}

void savePCLHandle(const sensor_msgs::PointCloud2::ConstPtr &pcl){
    if(!savePclMapSingnl)
        return ;
    cloudMap->clear();
    cloudMapDwz->clear();
    pcl::fromROSMsg(*pcl, *cloudMap);
    laserDwzFilter.setInputCloud(cloudMap);
    laserDwzFilter.setLeafSize(pclMapResolution, pclMapResolution, pclMapResolution);
    laserDwzFilter.filter(*cloudMapDwz);
    std::string file_path_pcl_base=file_path_pcl_ + std::to_string(rand()%RAND_MAX) ;
    //save ply 
    std::string file_path_pcl1=file_path_pcl_base +  ".ply";
    std::cout<<"Save the pcl map! Waiting..."<<std::endl;
    if(pcl::io::savePLYFileBinary<pcl::PointXYZI>(file_path_pcl1, *cloudMap)==0) //二进制方式保存
        std::cout<<"Succeed to save the pcl map --- "<<file_path_pcl1<<std::endl;
    else
        std::cout<<"Fail to save the pcl map ply!  "<<std::endl;
    //save pcd
    std::string file_path_pcl2=file_path_pcl_base + ".pcd";
    std::cout<<"Save the pcl map! Waiting..."<<std::endl;
    if(pcl::io::savePCDFileBinary<pcl::PointXYZI>(file_path_pcl2, *cloudMap)==0) 
        std::cout<<"Succeed to save the pcl map --- "<<file_path_pcl2<<std::endl;
    else
        std::cout<<"Fail to save the pcl map pcd!  "<<std::endl;
    //save vtk
    pcl::PCLPointCloud2 cloud2;
    pcl::toPCLPointCloud2(*cloudMap, cloud2);
    std::string file_path_pcl3=file_path_pcl_base + ".vtk";
    std::cout<<"Save the pcl map! Waiting..."<<std::endl;
    if(pcl::io::saveVTKFile(file_path_pcl3, cloud2)==0)  
        std::cout<<"Succeed to save the pcl map --- "<<file_path_pcl3<<std::endl;
    else
        std::cout<<"Fail to save the pcl map vtk!  "<<std::endl;
    savePclMapSingnl=false;
}

void loadPcl(const std::string file_name){
	if (pcl::io::loadPCDFile<pcl::PointXYZI>(file_name, *loadCloudMap) == 0){
		std::cout<<"load the pcl map success!"<<std::endl;
	}
    else{
        std::cout<<"load the pcl map failed !"<<std::endl;
    }
}

void saveGlobalMapHandle(const sensor_msgs::PointCloud2::ConstPtr &pcl){
    if(!saveGlobalMap)
        return ;
    globalMap->clear();
    globalMapDwz->clear();
    pcl::fromROSMsg(*pcl, *globalMap);
    //laserCloud->clear();
   // pcl::fromROSMsg(*pcl, *globalMap);

    int laserCloudSize = globalMap->points.size();
    for (int i = 0; i < laserCloudSize; i++) {
      float temp = globalMap->points[i].x;
      globalMap->points[i].x = globalMap->points[i].z;
      globalMap->points[i].z = globalMap->points[i].y;
      globalMap->points[i].y = temp;
    }

    mapDwzFilter.setInputCloud(globalMap);
    mapDwzFilter.setLeafSize(globalMapResolution, globalMapResolution, globalMapResolution);
    mapDwzFilter.filter(*globalMapDwz);
    //save pcd
    std::string file_path_pcl2="/home/yz/catkin_ws/relocation/src/hdl_localization/data/map2.pcd";
    std::cout<<"Save the global map! Waiting..."<<std::endl;
    if(pcl::io::savePCDFileBinary<pcl::PointXYZI>(file_path_pcl2, *globalMap)==0) 
        std::cout<<"Succeed to save the global map --- "<<file_path_pcl2<<std::endl;
    else
        std::cout<<"Fail to save the global map pcd!  "<<std::endl;
    //save vtk
    saveGlobalMap=false;
}

void excute_cmd(const std::string &cmd ){
    FILE *fp=NULL;
    char result_buf[1024];
    const char *sysCommand= cmd.data();
    fp=popen(sysCommand,"r");
    if(fp==NULL)
        std::cout<<"popen failed!"<<std::endl;
    std::cout<<"*******************************"<<std::endl;
    while(fgets(result_buf,sizeof(result_buf),fp)!=NULL)
    {
        if(result_buf[strlen(result_buf)-1]='\n')
            result_buf[strlen(result_buf)-1]='\0';
        std::cout<<result_buf<<std::endl;
    }
    pclose(fp);
}

void pubHomeCmd(){
    geometry_msgs::PointStamped homePoint;
    homePoint.point.x = current_odom_x;
    homePoint.point.y = current_odom_y;
    homePoint.point.z = current_odom_z;
    homePoint.header.frame_id = "/map";
    pubHome.publish(homePoint);
}
void returnHomeStatue(bool statu){
    std::cout<<"return statu "<<std::endl;
    std_msgs::Bool returnHome_signal ;
    returnHome_signal.data= statu;
    pubReturnHome.publish(returnHome_signal);
}
void excute(const int &mode){
    std::cout<<"************************"<<"A NEW CMD"<<"**********************"<<std::endl;
    std::cout<<"the current mode is "<<mode<<std::endl;
    switch(mode){
        default:
            std::cout<<"the current mode is "<<mode<<std::endl;
            break;
        case 0:
            std::cout<<"mode 0 --> init success! waiting your control..."<<std::endl
                                <<"mode 1 --> save the pcl map / -1 load"<<std::endl
                                <<"mode 2 --> save the octomap / -2 load"<<std::endl
                                <<"mode 3 --> start local_planner / -3 close"<<std::endl
                                <<"mode 4 --> start loam / -4 close"<<std::endl
                                <<"mode 5 -->start logitelep / -5 close" <<std::endl
                                <<"mode 6 -->start exploration / -6 close"<<std::endl
                                <<"mode 7 -->set home point / -7 reset homwpoint"<<std::endl
                                <<"mode 8 --> try to return home / -8 continue exploration"<<std::endl
                                <<"mode 9--> set relocation initpose"<<std::endl;
            break;
        case 1:
            savePclMapSingnl=true;
            break;
        case 2:
            saveOctomap();
            break;
        case 3:
            excute_cmd("python planner.py");
            break;
        case 4:
           excute_cmd("cd && python loam.py ");
            break;
        case 5:
            excute_cmd("cd && python legitelep.py");
            break;
        case 6:
            excute_cmd("");
            break;
        case 7:
            pubHomeCmd();
            break;
        case 8:
            returnHomeStatue(true);
            break;
        case 9:
            setReLocationInitPose=true;
            break;
        case 10:
            initialposePub.publish(reLocationInitPose);
            break;
        case 11:
            saveGlobalMap=true;
            break;
        case -1:
            //loadOctomap();
            break;
        case -2:
            loadPcl(file_path_pcl_);
            break;
        case -3:
           excute_cmd(" rosnode kill /pathFollower ");
            break;
        case -4:
            excute_cmd(" rosnode kill /base_link_to_camera && rosnode kill /camera_init_to_map && rosnode kill /featureAssociation && rosnode kill /imageProjection && rosnode kill /mapOptmization&& rosnode kill /transformFusion");
            break;
        case -5:
            excute_cmd("rosnode kill /log_teleop");
            break;
        case -6:
            excute_cmd("");
        case -7:
            break;
        case -8:
           returnHomeStatue(false);
            break;
    }
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
  current_odom_x = msg->pose.pose.position.x;
  current_odom_y = msg->pose.pose.position.y;
  current_odom_z = msg->pose.pose.position.z;
  if(setReLocationInitPose){
    setReLocationInitPose=false;
    reLocationInitPose.header = msg->header;
    reLocationInitPose.pose.pose.position.x = msg->pose.pose.position.x;
    reLocationInitPose.pose.pose.position.y = msg->pose.pose.position.y;
    reLocationInitPose.pose.pose.position.z = msg->pose.pose.position.z;
    reLocationInitPose.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    reLocationInitPose.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    reLocationInitPose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    reLocationInitPose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
  }
 //initialposePub.publish(reLocationInitPose);
}

void joystickHandler(const sensor_msgs::Joy::ConstPtr &joy) {//lym,产生探索开始信号
  if (joy->buttons[4] > 0.5) exploration_control();
  if (joy->buttons[0] > 0.5)     --mode;
  if (joy->buttons[1] > 0.5)    excute(mode);
  if (joy->buttons[2] > 0.5)    mode=0;
  if (joy->buttons[3] > 0.5)    mode++;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "decision");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  begin_signal.data=false;
  reLocationInitPose.header.frame_id="/map";
  reLocationInitPose.pose.pose.position.x=0;
  reLocationInitPose.pose.pose.position.y=0;
  reLocationInitPose.pose.pose.position.z=0;
  reLocationInitPose.pose.pose.orientation.x=0;
  reLocationInitPose.pose.pose.orientation.y=0;
  reLocationInitPose.pose.pose.orientation.z=0;
  reLocationInitPose.pose.pose.orientation.w=1;
  srand((unsigned)time(NULL));

  begin_signal_pub = nh.advertise<std_msgs::Bool>("/start_exploring", 1); //people control or automode exploration
  pubHome = nh.advertise<geometry_msgs::PointStamped>("/homePointVector", 1);   //set follow point
  pubReturnHome = nh.advertise<std_msgs::Bool>("/returnHomeStatu", 1);//set return home or continue to explore
  initialposePub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);//publish to relocation node
  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy>("/joy", 5, joystickHandler);//sub joy msg
  ros::Subscriber subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround",5,savePCLHandle);//save pcl map;/explored_areas;laser_cloud_surround
  ros::Subscriber odom_sub =nh.subscribe<nav_msgs::Odometry>("/state_estimation", 1, odom_callback);//help to set follow point & publishi initpose of relocation
  //ros::Subscriber initialposeSub =nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialposeLast", 1, initialposeLastHandle);//TODO 
  ros::Subscriber subMap = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround2",5,saveGlobalMapHandle);

  std::cout << "\033[1;32mDecision Started! Please push the button B to choose a mode ...\033[0m" << std::endl;
  ros::spin();  
}
