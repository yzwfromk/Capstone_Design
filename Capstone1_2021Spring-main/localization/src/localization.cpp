#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <string>
#include <signal.h>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/thread.hpp>

#include "ros/ros.h"

#include "std_msgs/Int8.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

int cont_after_switch=1;

// LiDAR data will be plotted on this virtual plane
float virtual_plane_resolution = 0.01;
int virtual_plane_width = 1000;
int virtual_plane_height = 1000;
cv::Mat virtual_plane = cv::Mat::zeros(virtual_plane_width, virtual_plane_height, CV_8UC3);


geometry_msgs::Vector3 robot_pos;
float pos_x;
float pos_y;
float pos_o;

float control_x;
float control_y;
float control_o;

std_msgs::Float32MultiArray obs_pos;


int lidar_size;
float lidar_degree[400];
float lidar_distance[400];


boost::mutex virtual_plane_mutex;

float PI = 4*atanf(1);
#define RAD2DEG(x) ((x)*180./PI)



bool check_point_range(int cx, int cy) //Checking whether the input point is in the virtual_plane
{
    return (cx<virtual_plane_width-1)&&(cx>0)&&(cy<virtual_plane_height-1)&&(cy>0);
}


float vectorMean(std::vector<float> V){
    float total = 0;
    for (int i=0; i<V.size(); i++){
        total += V[i];
    }
    return (total /(float)V.size());
}



vector<float> lineAnalysis(Vec4i line){ 
  vector<float> line_info;
  float slope;
  float perp; //the length of perpendicular 
  float perp_x; // x coordinate of the foot of perpendicular
  float perp_y; // y coordinate of the foot of perpendicular
  float length; //the length of the line

  if(line[2]-line[0] == 0){ //if the line is vertical
    slope = 2*atan(1);
    perp = (line[2]+line[0]-virtual_plane_width)/2.0; 
    perp_x = perp; 
    perp_y = 0; 
  }
  else{ 
    slope = (line[3]-line[1])/(float)(line[2]-line[0]); 
    perp_x = (slope*(line[0]-virtual_plane_width/2)-(line[1]-virtual_plane_height/2))*slope/(slope*slope+1); 
    perp_y = (-slope*(line[0]-virtual_plane_width/2)+(line[1]-virtual_plane_height/2))/(slope*slope+1); 
    perp = sqrt(pow(perp_x,2)+pow(perp_y,2)); 
    slope = atanf(slope);
  }
  length = sqrt(pow(line[3]-line[1],2)+pow(line[2]-line[0],2));

  line_info.push_back(slope);//the slope of the line(rad).
//between -pi/2 ~ pi/2

  line_info.push_back(perp_x);
  line_info.push_back(perp_y);
  line_info.push_back(perp);
  line_info.push_back(length);
  return line_info;
}








  void rectangular_map(vector<Vec4i> lines, float perp_difference_threshold, float slope_difference_threshold){

    float slope;
    float length;
    float perp;
    float perp_x;
    float perp_y;
    Vec4i l;

    pos_x=robot_pos.x;
    pos_y=robot_pos.y;
    pos_o=robot_pos.z;

    int robot_orientation_case, line_quadrant_case;
    vector<float> pos_o_raw, pos_x_raw, pos_y_raw;


//Debugging: cout<<endl<<"input xyo is "<<pos_x<<"/"<<pos_y<<"/"<<pos_o<<endl;

    for(int i = 0; i < lines.size(); i++)
    {
      l = lines[i];
      vector<float> la;
      la = lineAnalysis(l);
      slope =abs(la[0]);
      perp_x = la[1];
      perp_y = la[2];
      perp = abs(la[3]);






      //case categorization
      if( 0<pos_o && pos_o<PI/2 ){ //What quadrant is where robot orientation vector in?
        robot_orientation_case=1;
      }else if( PI/2<pos_o && pos_o<PI ){
        robot_orientation_case=2;
      }else if(PI<pos_o && pos_o<1.5*PI  ){
        robot_orientation_case=3;
      }else if(( 1.5*PI<pos_o && pos_o<2*PI  )){
        robot_orientation_case=4;
      }

      if( perp_x>=0 && perp_y >=0 ){
        line_quadrant_case=4;
      }else if( perp_x<=0 && perp_y >=0 ){
        line_quadrant_case=3;
      }else if( perp_x<=0 && perp_y <=0 ){
        line_quadrant_case=2;
      }else if(( perp_x>=0 && perp_y <=0 )){
        line_quadrant_case=1;
      }



//Debugging: cout<<"CASE "<<robot_orientation_case<<line_quadrant_case<<endl;

      switch (robot_orientation_case){

        case 1:
          switch (line_quadrant_case){
            case 1:
              if ( (abs(slope-pos_o)<slope_difference_threshold) && (abs(perp-(300-pos_y))<perp_difference_threshold) ){
                pos_o_raw.push_back(slope);
                pos_y_raw.push_back(300-perp);
              }
              break;
            case 2:
              if ( (abs(slope-(PI/2-pos_o))<slope_difference_threshold )&&( abs(perp-(pos_x))<perp_difference_threshold )){
                pos_o_raw.push_back(PI/2-slope);
                pos_x_raw.push_back(perp);
              }
              if( pos_x<50){
                if ( (abs(slope-(PI/2-pos_o))<slope_difference_threshold )&&( abs(perp-(100+pos_x))<perp_difference_threshold )){
                  pos_o_raw.push_back(PI/2-slope);
                  pos_x_raw.push_back(-100+perp);
              }
              }
              break;
            case 3:
              if ( abs(slope-pos_o)<slope_difference_threshold && (abs(perp-(pos_y))<perp_difference_threshold)  ){
                pos_o_raw.push_back(slope);
                pos_y_raw.push_back(perp);
              }
              break;
            case 4:
              if ( abs(slope-(PI/2-pos_o))<slope_difference_threshold && (abs(perp-(500-pos_x))<perp_difference_threshold) ){
                pos_o_raw.push_back(PI/2-slope);
                pos_x_raw.push_back(500-perp);
              }
          }
          break;

        case 2:
          switch (line_quadrant_case){
            case 1:
              if ( (abs(slope- (pos_o-PI/2))<slope_difference_threshold )&& abs(perp-pos_x)<perp_difference_threshold  ){
                pos_o_raw.push_back(slope+PI/2);
                pos_x_raw.push_back(perp);
              }

              if( pos_x<50){
                if ( (abs(slope- (pos_o-PI/2))<slope_difference_threshold )&&( abs(perp-(100+pos_x))<perp_difference_threshold )){
                  pos_o_raw.push_back(slope+PI/2);
                  pos_x_raw.push_back(-100+perp);
              }
              }
              break;
            case 2:
              if ( abs(slope-(PI-pos_o))<slope_difference_threshold && abs(perp-(pos_y))<perp_difference_threshold  ){
                pos_o_raw.push_back(PI-slope);
                pos_y_raw.push_back(perp);
              }
              break;
            case 3:
              if ( abs(slope-(pos_o-PI/2))<slope_difference_threshold && abs(perp-(500-pos_x))<perp_difference_threshold  ){
                pos_o_raw.push_back(slope+PI/2);
                pos_x_raw.push_back(500-perp);
              }
              break;
            case 4:
              if ( abs(slope-(PI-pos_o))<slope_difference_threshold && abs(perp-(300-pos_y))<perp_difference_threshold  ){
                pos_o_raw.push_back(PI-slope);
                pos_y_raw.push_back(300-perp);
              }
              break;
            }

          break;

        case 3:
          switch (line_quadrant_case){
            case 1:
              if ( abs(slope-(pos_o-PI))<slope_difference_threshold && abs(pos_y-perp)<perp_difference_threshold  ){
                pos_o_raw.push_back(slope+PI);
                pos_y_raw.push_back(perp);
              }
              break;
            case 2:
              if ( abs(slope-(1.5*PI-pos_o))<slope_difference_threshold && abs(pos_x-(500-perp))<perp_difference_threshold  ){
                pos_o_raw.push_back(1.5*PI-slope);
                pos_x_raw.push_back(500-perp);
              }
              break;
            case 3:
              if ( abs(slope-(pos_o-PI))<slope_difference_threshold && abs(pos_y-(300-perp))<perp_difference_threshold  ){
                pos_o_raw.push_back(slope+PI);
                pos_y_raw.push_back(300-perp);
              }
              break;
            case 4:
              if ( abs(slope- (1.5*PI-pos_o))<slope_difference_threshold && abs(perp-pos_x)<perp_difference_threshold  ){
                pos_o_raw.push_back(1.5*PI-slope);
                pos_x_raw.push_back(perp);
              }

              if( pos_x<50){
                if ( (abs(slope- (1.5*PI-pos_o))<slope_difference_threshold )&&( abs(perp-(100+pos_x))<perp_difference_threshold )){
                  pos_o_raw.push_back(1.5*PI-slope);
                  pos_x_raw.push_back(-100+perp);
              }
              }
              break;
          }
          break;

        case 4:
          switch (line_quadrant_case){
            case 1:
              if ( abs(slope-(pos_o-1.5*PI))<slope_difference_threshold && abs(pos_x-(500-perp) )<perp_difference_threshold ){
                pos_o_raw.push_back(slope+1.5*PI);
                pos_x_raw.push_back(500-perp);
              }
              break;
            case 2:
              if ( abs(slope-(2*PI-pos_o))<slope_difference_threshold && abs(pos_y-(300-perp))<perp_difference_threshold  ){
                pos_o_raw.push_back(2*PI-slope);
                pos_y_raw.push_back(300-perp);
              }
              break;
            case 3:

              if ( abs(slope-(pos_o-1.5*PI))<slope_difference_threshold && abs(pos_x-perp)<perp_difference_threshold  ){
                pos_o_raw.push_back(slope+1.5*PI);
                pos_x_raw.push_back(perp);
              }

              if( pos_x<50){
                if ( (abs(slope-(pos_o-1.5*PI))<slope_difference_threshold )&&( abs(perp-(100+pos_x))<perp_difference_threshold )){
                  pos_o_raw.push_back(slope+1.5*PI);
                  pos_x_raw.push_back(-100+perp);
              }
              }
              break;
            case 4:
              if ( abs(slope- (2*PI-pos_o))<slope_difference_threshold && abs(perp-pos_y)<perp_difference_threshold ){
                pos_o_raw.push_back(2*PI-slope);
                pos_y_raw.push_back(perp);
              }
              break;
          }break;
      }
    }

//Debugging: cout<<"pos_o_raw/x/y/line number "<<pos_o_raw.size()<<"/"<<pos_x_raw.size()<<"/"<<pos_y_raw.size()<<"/"<<lines.size()<<endl;

    //cout<<pos_o_raw.size()<<"/"<<lines.size()<<endl;


    //Exception handling
    if(pos_o==PI/2 || pos_o==PI || pos_o==1.5*PI || pos_o==2*PI){
      pos_o=pos_o-0.01;
    }else if(pos_o==0){
      pos_o=pos_o+0.01;
    }

    if(lines.size()>0){

      bool is_correction_done=true;

      if(pos_o_raw.size()<lines.size()/5){//indicates pos_o is errorneous. Below codes are for correcting the false pos_o.
      //The more strict this condition is, the less strict below condition should be.
      //Below condition should be eased if the rotation speed is large

        
        if(abs(pos_o-PI/2)<0.1){// slope_difference_threshold should be large as this condition is eased.
        //Recommended value of this condition is the half of the slope_difference_threshold
          pos_o=PI/2+(PI/2-pos_o); 
        }else if(abs(pos_o-PI)<0.1){
          pos_o=PI+(PI-pos_o);
        } else if(abs(pos_o-1.5*PI)<0.1){
          pos_o=1.5*PI+(1.5*PI-pos_o);
        } else if(0<pos_o && pos_o<0.1){
          pos_o=2*PI-pos_o;
          is_correction_done=false;
        }else if(0>pos_o){
          pos_o=2*PI+pos_o;
          is_correction_done=false;
        } else if( 0<2*PI-pos_o && 2*PI-pos_o<0.1 && is_correction_done){
          pos_o=(2*PI-pos_o);
        } else if(2*PI-pos_o<0  && is_correction_done){
          pos_o=pos_o-2*PI;
        }

        if(pos_x_raw.size()<2 && pos_y_raw.size()>2){
          pos_x=pos_x;
          pos_y=vectorMean(pos_y_raw);
        }else if(pos_y_raw.size()<3 && pos_x_raw.size()>1 ){
          pos_y=pos_y;
          pos_x=vectorMean(pos_x_raw);
        }else if(pos_y_raw.size()<3 && pos_x_raw.size()<2){
          pos_x=pos_x;
          pos_y=pos_y;
        }


      }else if(pos_x_raw.size()<2 && pos_y_raw.size()>2){
          pos_x=pos_x;
          pos_y=vectorMean(pos_y_raw);
          pos_o=vectorMean(pos_o_raw);
      }else if(pos_y_raw.size()<3 && pos_x_raw.size()>1){
          pos_y=pos_y;
          pos_x=vectorMean(pos_x_raw);
          pos_o=vectorMean(pos_o_raw);
      }else if(pos_x_raw.size()>1 && pos_y_raw.size()>2){
        pos_o=vectorMean(pos_o_raw);
        pos_x=vectorMean(pos_x_raw);
        pos_y=vectorMean(pos_y_raw);
      }else if(pos_o==PI/2 || pos_o==PI || pos_o==1.5*PI || pos_o==2*PI){
        pos_o=pos_o-0.01;
      }else if(pos_o==0){
        pos_o=pos_o+0.01;
      }else{
        pos_x=pos_x;
        pos_y=pos_y;
        pos_o=pos_o;
      }
    }


//Debugging:
//cout<<"output xyo is "<<pos_x<<"/"<<pos_y<<"/"<<pos_o<<endl;

    robot_pos.x=pos_x;
    robot_pos.y=pos_y;
    robot_pos.z=pos_o;

}



void lidar_Callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    virtual_plane_mutex.lock();
    int count = (scan->angle_max -scan->angle_min)/ scan->angle_increment+1;
    lidar_size=count;
    for(int i = 0; i < count; i++)
    {
        lidar_degree[i] = scan->angle_min + scan->angle_increment * i;
        lidar_distance[i]=scan->ranges[i];
    }
    virtual_plane_mutex.unlock();
}



int zone_info;
void entrance_Callback(const std_msgs::Int8::ConstPtr& zone)
{
  zone_info=zone->data;

}


float linear_vel_now, linear_vel_prev=0;
float angular_vel_now, angular_vel_prev=0;
float Ts;
int delivery_mode=0;

void velocity_odometry_Callback(const geometry_msgs::Twist::ConstPtr& targetVel){
  linear_vel_now = targetVel->linear.x;
  angular_vel_now = targetVel->angular.z;

  float lin_scaling=0.001;
  float ang_scaling=0.0001;

  if (zone_info==2 && cont_after_switch>=1000 && delivery_mode==0){

    robot_pos.x=robot_pos.x+linear_vel_prev*cos(robot_pos.z)*lin_scaling;
    robot_pos.y=robot_pos.y+linear_vel_prev*sin(robot_pos.z)*lin_scaling;
    robot_pos.z=robot_pos.z+angular_vel_prev*ang_scaling;

  }
  
  linear_vel_prev=linear_vel_now;
  angular_vel_prev=angular_vel_now;
}


void delivery_mode_Callback(const std_msgs::Int8::ConstPtr& delivery){
  delivery_mode=delivery->data;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle nh; 
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1000, lidar_Callback);
    ros::Subscriber sub1 = nh.subscribe<std_msgs::Int8>("/zone", 1, entrance_Callback); 
    ros::Subscriber commandVel = nh.subscribe<geometry_msgs::Twist>("/command_vel", 10, velocity_odometry_Callback);
    ros::Subscriber delivery = nh.subscribe<std_msgs::Int8>("/ball_delivery", 10, delivery_mode_Callback);

    ros::Publisher pub = nh.advertise<geometry_msgs::Vector3>("/robot_pos", 1); 
    ros::Publisher pub1 = nh.advertise<std_msgs::Float32MultiArray>("/obs_pos", 1);


// Debugging:
// cv::Mat zone = cv::Mat::zeros(600, 600, CV_8UC3);
// line(zone, Point(50, 50), Point(550, 50), Scalar(255,255,255), 1);
// line(zone, Point(50, 50), Point(50, 250), Scalar(255,255,255), 1);
// line(zone, Point(550, 50), Point(550, 350), Scalar(255,255,255), 1);
// line(zone, Point(50, 350), Point(550, 350), Scalar(255,255,255), 1);

// circle(zone, Point(270, 130), 10, cv::Scalar(255,255,255), -1);
// circle(zone, Point(230, 270), 10, cv::Scalar(255,255,255), -1);
// circle(zone, Point(400, 260), 10, cv::Scalar(255,255,255), -1);

// circle(zone, Point(550, 200), 10, cv::Scalar(0,255,0), -1);


    while(ros::ok){


        //Substracting the obstacle data from laser scan, and Drawing remaining wall data on the virtual virtual_plane
        vector<float> wall_distance, wall_degree;
        vector<float> obs_distance, obs_degree;

        wall_distance.push_back(lidar_distance[0]);
        wall_degree.push_back(lidar_degree[0]);

        int view_angle=0;
        float sum=lidar_distance[0];
        float avg_distance;
        float obstacle_width;


        for(int i = 1; i < lidar_size; i++)
        {

          if(lidar_distance[i] >0){ //smaller than the maximum liDAR range(distance)
              wall_distance.push_back(lidar_distance[i]);
              wall_degree.push_back(lidar_degree[i]);
              view_angle++;
              sum=sum+lidar_distance[i];
          }

          if(abs(lidar_distance[i] - lidar_distance[i-1]) > 0.3){
          //distance changed abruptly -> outermost wall or an another distant obstacle is detected.
            view_angle=view_angle-1;
            sum=sum-lidar_distance[i];
            avg_distance=sum/(view_angle+1);
            obstacle_width=avg_distance*(PI/180)*view_angle;
//Debugging:cout<<obstacle_width<<"/"<<view_angle<<endl;

            if(0.07<obstacle_width && obstacle_width < 0.15 && view_angle>0 && abs(lidar_distance[i-1]-avg_distance)<0.2 ){//threshold should be larger than the maximum width of the obstacle
              //this means that formerly detected object has small width, which means it is likely to be an obstacle

              obs_distance.push_back(lidar_distance[i-1-view_angle/2]+0.15/2);
              if (lidar_degree[i-1-view_angle/2]>=PI/2 && lidar_degree[i-1-view_angle/2]<=1.5*PI){
                obs_degree.push_back(lidar_degree[i-1-view_angle/2]-PI/2);
              } else if(lidar_degree[i-1-view_angle/2]<=PI/2){
                obs_degree.push_back(lidar_degree[i-1-view_angle/2]-PI/2);
              } else if(lidar_degree[i-1-view_angle/2]>=1.5*PI && lidar_degree[i-1-view_angle/2]<=2*PI){
                obs_degree.push_back(lidar_degree[i-1-view_angle/2]-2.5*PI);
              }


              for(int j=0; j<view_angle+2; j++){
                wall_distance.pop_back();
                wall_degree.pop_back();
              } //remove the added lidar data as much as the view angle of the obstacle
              wall_distance.push_back(lidar_distance[i]);
              wall_degree.push_back(lidar_degree[i]);


            }
              view_angle=0; //Initiallization
              sum=lidar_distance[i];

          }

        }

        //Drawing

// Debugging:
// int cxi0 = virtual_plane_width/2 + (int)(wall_distance[0]*sin(wall_degree[0])/virtual_plane_resolution);
// int cyi0 = virtual_plane_height/2 + (int)(wall_distance[0]*cos(wall_degree[0])/virtual_plane_resolution);
// circle(virtual_plane, Point(cxi0, cyi0), 3, cv::Scalar(255,0,0), -1);


        for(int i = 1; i<wall_distance.size(); i++){
          int cxi = virtual_plane_width/2 + (int)(wall_distance[i]*sin(wall_degree[i])/virtual_plane_resolution);
          int cyi = virtual_plane_height/2 + (int)(wall_distance[i]*cos(wall_degree[i])/virtual_plane_resolution);
          int cxi0 = virtual_plane_width/2 + (int)(wall_distance[i-1]*sin(wall_degree[i-1])/virtual_plane_resolution);
          int cyi0 = virtual_plane_height/2 + (int)(wall_distance[i-1]*cos(wall_degree[i-1])/virtual_plane_resolution);

          if(check_point_range(cxi,cyi) && check_point_range(cxi0,cyi0) && sqrt(pow(cxi-cxi0,2)+pow(cyi-cyi0,2)) < 0.6/virtual_plane_resolution)
          {
            line(virtual_plane, Point(cxi, cyi), Point(cxi0, cyi0), Scalar(255,255,255), 1); //line 함수는 (그릴 대상, 양 끝점1,양끝점2, 선 색깔, 선 타입)으로 입력하여 선을 그려줌
          }
        }

        cv::Mat gray;
        cvtColor(virtual_plane,gray,COLOR_BGR2GRAY);
        cv::Mat edges;
        Canny(gray,edges,20,200);
        vector<Vec4i> lines; 
        HoughLinesP(edges, lines, 0.5, CV_PI/180, 20, 20, 90);
        //Inspects lines from the given image gray. 3rd and 4th input is the resolution of the parameters of line modelling 'r=xcos(th)+ysin(th)'
        //5th input is the minimum data required to be considered as a line.
        //6th input is the minimum length required to be considered as a line.
        //7th input is the maximum vacant length not to be considered as different lines.



//Debugging:: Vec4i l;
//         for(int i = 0; i < lines.size(); i++)
//    {
//      l = lines[i];
//      line(virtual_plane, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 1);
//    }




//Switching function
    if( delivery_mode != 1) {
        if (zone_info==1){
          robot_pos.x=-20;
          robot_pos.y=50;
          robot_pos.z=2*PI-0.2;
          //cout<<"localization not yet"<<endl;
        }else if (zone_info==2 && cont_after_switch<=1000){
          rectangular_map(lines, 30, 0.3);
          cont_after_switch++;
          if( robot_pos.x==-20 && robot_pos.y==50 && robot_pos.z==2*PI-0.2){
            robot_pos.x=-20;
            robot_pos.y=50;
            robot_pos.z=2*PI-0.09;
            rectangular_map(lines, 30, 0.3);
          }

          if( robot_pos.x==-20 && robot_pos.y==50 && robot_pos.z==0.09){
            robot_pos.x=-20;
            robot_pos.y=50;
            robot_pos.z=2*PI-0.2;
            rectangular_map(lines, 30, 0.3);
          }

        }else{
          rectangular_map(lines, 30, 0.2); //slope_difference_threshold should be smaller than pi/2 at least.
        }

        //Getting obstacle location
        obs_pos.data.clear();

        if(zone_info==2){

            for (int i=0; i<obs_distance.size(); i++){
              obs_pos.data.push_back(obs_distance[i]);
              obs_pos.data.push_back(obs_degree[i]);
            }
        }
      }

//Debugging:
// for(int i=0; i<obs_pos.data.size()/2; i++ ){
//   //cout<<obs_pos.data[2*i]<<"/"<<obs_pos.data[2*i+1]<<endl;
// }

// Debugging: circle(virtual_plane,Point(virtual_plane_width/2,virtual_plane_height/2),10, cv::Scalar(0,0,255), -1);
// circle(zone, Point(50+int(robot_pos.x), 350-int(robot_pos.y)), 3, cv::Scalar(255,0,0), -1);
//cv::imshow("Harvesting zone map", virtual_plane);


      	pub.publish(robot_pos);
        pub1.publish(obs_pos);

        ros::spinOnce();
    }

    return 0;
}
