#include <ros/ros.h>
#include "string.h"
#include <opencv2/highgui.hpp>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <vector>
#include "core_msgs/ball_position.h"
#include <math.h>

using namespace cv;
using namespace std;

Mat MAP = cv::Mat::zeros(520, 520, CV_8UC3);     // final map

int nSubBalls;
float ballAngle[20];
float ballDist[20];

int CENCOL = 260;
int CENROW = 260;

int camX = 260;
int camY = 260 + 2 * 100;

#define nTrueBalls 11
float trueX[nTrueBalls] = {0.3, 0, 0, -0.875, 0.5, 0.375, -0.55, -0.575, -0.275, 0.225, 0};
float trueY[nTrueBalls] = {-0.125, 0, 1.5, 1.4, 2, 0.625, 0.875, 0.45, -0.825, -1.325, -1.55};
bool detected[nTrueBalls];

void ballPos_callback(const core_msgs::ball_position::ConstPtr& pos)
{
    nSubBalls = pos->size;
    if (nSubBalls > 20) nSubBalls = 20;
    for(int i = 0; i < nSubBalls; i++)
    {
        ballAngle[i] = pos->angle[i];
        ballDist[i] = pos->dist[i];
    }
}

void reset()
{
  Rect roi(Point(20,20),Point(490,490));
  MAP(roi)=Scalar(0);
  circle(MAP, Point(camX,camY), 3, cv::Scalar(255,0,0), -1);
}

void drawTruePoint()
{
  for (int i=0; i<nTrueBalls; i++){
    int X = (int)cvRound(CENCOL + trueX[i]*100);
    int Y = (int)cvRound(CENROW - trueY[i]*100);
    circle(MAP, Point(X,Y), 4, cv::Scalar(255,255,255), 3);
    detected[i] = false;
  }
}

vector<float> findClosestPoint(int i, vector<float>& err)
{
  int X = (int)cvRound(camX - ballDist[i]*sin(ballAngle[i])*100);
  int Y = (int)cvRound(camY - ballDist[i]*cos(ballAngle[i])*100);
  float min = 9999.;
  float dist;
  int target = -1;
  for (int j=0; j<nTrueBalls; j++){
    int tmpX = (int)cvRound(CENCOL + trueX[j]*100);
    int tmpY = (int)cvRound(CENROW - trueY[j]*100);
    dist = sqrt(pow(X-tmpX,2)+pow(Y-tmpY,2));
    if (dist < min){
      target = j;
      min = dist;
    }
  }
  if (target != -1) {
    detected[target] = true;
    circle(MAP, Point(X, Y), 3, cv::Scalar(0,0,255), -1);

    float trueAng = atan(-trueX[target]/(trueY[target]+2));
    float trueDist = sqrt(trueX[target]*trueX[target] + (trueY[target]+2)*(trueY[target]+2));
    float errDist = abs(trueDist-ballDist[i])/trueDist*100;
    float errAngle = (trueAng!=0) ? abs((trueAng-ballAngle[i])/trueAng)*100 : 0;
    err.push_back(errDist);
    err.push_back(errAngle);
    // cout << "true angle : " << trueAng << ", estimate angle : " << ballAngle[i] << endl;
    // cout << "error angle : " << errAngle << endl;
  }
  err.push_back(0);
  err.push_back(0);
  return err;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "ball_detect_test_node"); //init ros nodd
   ros::NodeHandle nh; //create node handler
   ros::Subscriber sub_rgb = nh.subscribe("/ball_position", 100, ballPos_callback);
   ros::Rate lr(1);
   line(MAP, Point(10, 10), Point(510, 10), Scalar(255,255,255), 1);
   line(MAP, Point(10, 10), Point(10, 510), Scalar(255,255,255), 1);
   line(MAP, Point(510, 10), Point(510, 510), Scalar(255,255,255), 1);
   line(MAP, Point(10, 510), Point(510, 510), Scalar(255,255,255), 1);

   while (ros::ok()){
     reset();
     drawTruePoint();
     float errDist = 0., errAngle = 0.;
     vector<float> err;
     err.clear();
     for (int i=0; i<nSubBalls; i++){
       err = findClosestPoint(i, err);
       errDist += err[0];
       errAngle += err[1];
     }
     errDist /= nSubBalls;
     errAngle /= nSubBalls;
     cout << "[Error estimate] " << nSubBalls << " / " << nTrueBalls << " detected, dist_err = " << errDist << " %, angle_err = " << errAngle << " %" << endl;
     imshow("map", MAP);
     waitKey(10);
     ros::spinOnce(); //spin.
     lr.sleep();
   }
   return 0;
}
