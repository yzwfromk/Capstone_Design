#include <ros/ros.h>
#include "string.h"
#include <opencv2/highgui.hpp>
#include <opencv2/plot.hpp>
#include <iostream>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/opencv.hpp"
#include <vector>
#include "std_msgs/Int8.h"
#include "core_msgs/ball_position.h"
#include "core_msgs/goal_position.h"
#include <math.h>

// #define DEBUG
// #define DEBUG_IMG

using namespace cv;
using namespace std;

Mat buffer, buffer_depth;
ros::Publisher pubBall;
ros::Publisher pubGoal;
//ros::Publisher pub_markers;

float FOCAL_LENGTH = 589.37;
int WIDTH = 639;
int HEIGHT = 479;

float RADIUS = 0.15/2; //[m]
float THRESHOLD = 3;
float FOV = 28.5*M_PI/180;
float dHeight = 0.178 - RADIUS;

int delivery_mode=0;
void delivery_mode_Callback(const std_msgs::Int8::ConstPtr& delivery){
  delivery_mode=delivery->data;
}


bool isBlack(int row, int col, Mat img){
  return img.at<uchar>(row,col)<100;
}

bool goal_check(int row, int col, int r, Mat img){
  // return (img.at<uchar>(row-r+1,col) > 45 && img.at<uchar>(row-r+1,col) < 75 && img.at<uchar>(row,col) > 45 && img.at<uchar>(row,col) < 75);
  return (img.at<uchar>(row-r*0.7,col) > 40);
}

void print(int index, int row, int col, int phase, bool flag, float detect_r, float pred_r, float dist)
{
  char str[30];
  if (flag){
    return;
    snprintf(str, sizeof(str), "[Phase %d] SUCCESS (%d-th)", phase, index);
    putText(buffer, str, Point(col-2*detect_r, row), 1, 1, Scalar(0, 0, 255), 1, 8);
    cout << "[Phase " << phase << "] (" << index << "-th) SUCCESS (r_detect, r_pred): (" << detect_r << ", " << pred_r << ") distance: " << dist  << endl;
  }
  else {
    snprintf(str, sizeof(str), "[Phase %d] filterted (%d-th)", phase, index);
    putText(buffer, str, Point(col-2*detect_r, row), 1, 1, Scalar(255, 0, 0), 1, 8);
    cout << "[Phase " << phase << "] (" << index << "-th) filtered (r_detect, r_pred): (" << detect_r << ", " << pred_r << ") distance: " << dist<< endl;
  }
}

bool filtering1(int row, int col, int r, Mat img){
  return (isBlack(row,col-r/2,img) && isBlack(row,col+r/2,img) && isBlack(row,col,img));
}

bool filtering2(int row, int col, float r, Mat img){
  int targetPoint;
  int rFloor = (int)cvFloor(r);
  if (isBlack(row,col,img)){
    targetPoint = (isBlack(row,col-rFloor,img)) ? col + rFloor : col - rFloor;
    if (buffer_depth.at<float>(row,col)-buffer_depth.at<float>(row,targetPoint) <= RADIUS) {return false;}
  }
  else {
    targetPoint = row - rFloor;
    if (buffer_depth.at<float>(targetPoint+THRESHOLD,col)-buffer_depth.at<float>(row,col) < 2*RADIUS) {return false;}
  }
  return true;
}

bool filtering3(int row, int col){ // too close to detect
  return (buffer_depth.at<float>(row,col) < 0.4) ? true : false;
}

vector<Vec4f> filtering(vector<Vec3f> circles, Mat img){
  vector<Vec4f> filtered; // 0:col, 1:radius, 2:distance
  Vec4f circle;
  int size = circles.size();
  int col, row;
  float r, r_pred, distance, f, angle;
  for (int i=0; i<size; i++){
    col = (int)cvRound(circles[i][0]);
    row = (int)cvRound(circles[i][1]);
    r = circles[i][2];
    if (filtering1(row,col,r,img)){ // in case of detecing floor as a ball
#ifdef DEBUG
      print(i, row, col, 1, false, 0, 0, 0);
#endif
      continue;
    }

    f = sqrt(pow(FOCAL_LENGTH,2)+pow((WIDTH/2.-col),2));
    angle = atan((2.5*(col-319.5)/320)/4.6621);
    distance = (buffer_depth.at<float>(row, col) + RADIUS) / cos(angle);
    r_pred = RADIUS*f/distance;

    if (filtering2(row,col,r,img)){ // block by pillar of other balls
      int rFloor = (int)cvFloor(r);
      int targetPoint;
      float targetPointDist, angle;
      if (isBlack(row,col,img)){ // blocked by pillar or scooper
        targetPoint = (isBlack(row,col-rFloor+THRESHOLD,img)) ? col+rFloor-THRESHOLD : col-rFloor+THRESHOLD;
        angle = atan((col-319.5)/320*tan(FOV));
        targetPointDist = buffer_depth.at<float>(row,targetPoint)/cos(angle);
      }
      else {
        targetPoint = row-rFloor+THRESHOLD;
        angle = atan((2.5*(col-319.5)/320)/4.6621);
        targetPointDist = buffer_depth.at<float>(targetPoint,col) / cos(angle);
      }
      float r_pred = RADIUS*FOCAL_LENGTH/targetPointDist;
      if (abs(r_pred-r) < r/5){
        circle[0] = col;
        circle[1] = row;
        circle[2] = r;
        circle[3] = targetPointDist;
        filtered.push_back(circle);
#ifdef DEBUG
        print(i, row, col, 2, true, r, r_pred, targetPointDist);
#endif
      }
      else {
#ifdef DEBUG
        print(i, row, col, 2, false, r, r_pred, targetPointDist);
#endif
      }
      continue;
    }
    if (filtering3(row,col)){
      continue;
    }
    if (abs(r_pred-r) < r/7){
      circle[0] = col;
      circle[1] = row;
      circle[2] = r;
      circle[3] = distance;
      filtered.push_back(circle);
#ifdef DEBUG
      print(i, row, col, 3, true, r, r_pred, distance);
#endif
      continue;
    }
#ifdef DEBUG
    print(i,row,col,3,false, r, r_pred, distance);
#endif

    // if (row >= HEIGHT-1){  // right before harvesting the ball
    //   if (filtering3(row,col,cvFloor(circle[2]),img, distance)){
    //     circle[0] = col;
    //     circle[1] = row;
    //     circle[2] = r;
    //     circle[3] = distance;
    //     filtered.push_back(circle);
    //     continue;
    //   }
    // }
    // cout << "(r,c): (" <<row << ", " << col << "), rad:" << r << ", f: " << f << ", dist: " << distance << endl;
    // cout << "r_predictd: " << r_pred << ", r_pixel: " << r << endl;

  }
  return filtered;
}

void ball_detect(){
     Mat hsv, gray;  //assign a memory to save the edge images
     Mat hsvCh[3];
     cvtColor(buffer, hsv, CV_BGR2HSV);
     split(hsv, hsvCh);
// #ifdef DEBUG_IMG
//      cv::imshow("h", hsvCh[0]);
//      waitKey(10);
//      cv::imshow("s", hsvCh[1]);
//      waitKey(10);
//      cv::imshow("v", hsvCh[2]);
//      waitKey(10);
// #endif
     GaussianBlur(hsvCh[1],hsvCh[1], Size(5,5), 2.5, 2.5);
     // cv::imshow("Gaussian Blurred", hsvCh[0]);
     // waitKey(10);
     threshold(hsvCh[1], gray, 100, 255, THRESH_BINARY);
// #ifdef DEBUG_IMG
//      cv::imshow("Binary", gray);
//      waitKey(10);
//      cv::imshow("h", hsvCh[0]);
//      waitKey(10);
// #endif
     vector<Vec3f> circles; //assign a memory to save the result of circle detection
     vector<Vec4f> filteredCircles; //<row,col,r,dist>
     HoughCircles(gray,circles,HOUGH_GRADIENT, 1, gray.rows/30, 30, 10, 7, 200); //proceed circle detection
     //Circles (Cx, Cy, r)

     //before Filtering
#ifdef DEBUG
     for(int k=0,i=0;k<circles.size();k++){
         Vec3f params = circles[k];  //the information of k-th circle
         float c_c=cvRound(params[0]);  //x position of k-th circle
         float c_r=cvRound(params[1]);  //y position
         float r=cvRound(params[2]); //radius

         Point center(c_c,c_r);  //declare a Point Point(coloum, row)
         circle(buffer,center,r,Scalar(255,0,255),1); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth
     }
#endif
     filteredCircles = filtering(circles, gray);
     Vec4f params; //assign a memory to save the information of circles
     int c_c,c_r,r;
     int nBalls = filteredCircles.size();
     int goalIndex = -1;
     core_msgs::ball_position msgBall;  //create a message for ball positions
     core_msgs::goal_position msgGoal;
     for (int i=0; i<filteredCircles.size(); i++){
        params = filteredCircles[i];
        c_c = (int)cvRound(params[0]);
        c_r = (int)cvRound(params[1]);
        r = (int)cvFloor(params[2]);
        if (goal_check(c_r,c_c,r, hsvCh[0])){
          nBalls -= 1;
        }
     }

     msgBall.size = nBalls; //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
     msgBall.angle.resize(nBalls);  //adjust the size of array
     msgBall.dist.resize(nBalls);  //adjust the size of array

     for(int k=0,i=0;k<filteredCircles.size();k++){
         params = filteredCircles[k];  //the information of k-th circle
         c_c=(int)cvRound(params[0]);  //x position of k-th circle
         c_r=(int)cvRound(params[1]);  //y position
         r=(int)cvRound(params[2]); //radius

         // 원 출력을 위한 원 중심 생성
         Point center(c_c,c_r);  //declare a Point Point(coloum, row)
         // cy = 3.839*(exp(-0.03284*cy))+1.245*(exp(-0.00554*cy));   //convert the position of the ball in camera coordinate to the position in base coordinate. It is related to the calibration process. You shoould modify this.
         // cx = (0.002667*cy+0.0003)*cx-(0.9275*cy+0.114);
         if (goal_check(c_r,c_c,r, hsvCh[0])){
           circle(buffer,center,r,Scalar(0,255,0),3); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth
           // msgGoal.angle = atan((c_c-319.5)/320*tan(FOV)); // [rad]
           // // msgGoal.angle = atan((2.5*(c_c-319.5)/320)/4.6621);
           // msgGoal.dist = params[3];
           // pubGoal.publish(msgGoal);
#ifdef DEBUG
           cout << "[Goal] Distance: " << msgGoal.dist << ", Angle= " << msgGoal.angle << endl;
#endif
           continue;
         }
         circle(buffer,center,r,Scalar(0,0,255),2); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth
         msgBall.angle[i]=atan((c_c-319.5)/320*tan(FOV));  //[rad]
         // msgBall.angle[i] = atan((2.5*(c_c-319.5)/320)/4.6621); // [rad]
         msgBall.dist[i]=params[3];   //[m]
#ifdef DEBUG
         cout << "[Ball] Distance" << msgBall.dist[i] << ", Angle= " << msgBall.angle[i] << endl;
         cout << endl;
#endif
         ++i;

     }
     pubBall.publish(msgBall);  //publish a message
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
   try
   {
     buffer = cv_bridge::toCvShare(msg, "bgr8")->image;  //transfer the image data into buffer
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
}

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
   try
   {
     buffer_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
     buffer_depth.convertTo(buffer_depth, CV_32F, 0.001);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to '16UC1'.", msg->encoding.c_str());
   }
}


int main(int argc, char **argv)
{
   ros::init(argc, argv, "ball_detect_node"); //init ros nodd
   ros::NodeHandle nh; //create node handler
   image_transport::ImageTransport it(nh); //create image transport and connect it to node hnalder
   image_transport::Subscriber sub_rgb = it.subscribe("/kinect_rgb", 1, imageCallback); //create subscriber
   image_transport::Subscriber sub_depth = it.subscribe("/kinect_depth", 1, depthCallback);
   ros::Subscriber delivery = nh.subscribe<std_msgs::Int8>("/ball_delivery", 10, delivery_mode_Callback);

   pubBall = nh.advertise<core_msgs::ball_position>("/ball_position", 10); //setting publisher
   pubGoal = nh.advertise<core_msgs::goal_position>("/goal_position", 10); //setting publisher
   ros::Rate loop_rate(10);
   while (ros::ok()){
     if(!buffer.empty() && !buffer_depth.empty() && (delivery_mode != 1)){
       ball_detect();
     }
#ifdef DEBUG_IMG
     if (!buffer.empty()){
       cv::imshow("view", buffer);  //show the image with a window
       cv::waitKey(1);
     }
#endif
     loop_rate.sleep();
     ros::spinOnce();
   }
   return 0;
}
