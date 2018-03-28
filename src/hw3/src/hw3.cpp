#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <Eigen/Dense>
#include <cmath>
#include <boost/thread.hpp>
#include <visualization_msgs/Marker.h>
#define t 0.005

using namespace Eigen;
using namespace std;

//ros::NodeHandle nh;
double G_x;
double G_y;
double G_z;
double A_x;
double A_y;
double A_z;

visualization_msgs::Marker line_strip;
Matrix<double, 3, 3> C;
Matrix<double, 3, 3> C1;
Matrix<double, 3, 3> I;
Matrix<double, 3, 3> B;
Matrix<double, 3, 1> gg;
Matrix<double, 3, 1> gg1;
Matrix<double, 3, 1> wb;
Matrix<double, 3, 1> vg;
Matrix<double, 3, 1> ab;
Matrix<double, 3, 1> ag;
Matrix<double, 3, 1> sg;

double sigma;
int cnt ,cnt1; 

void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ros::NodeHandle nh;

  G_x = msg->angular_velocity.x;
  G_y = msg->angular_velocity.y;
  G_z = msg->angular_velocity.z;
  A_x = msg->linear_acceleration.x;
  A_y = msg->linear_acceleration.y;
  A_z = msg->linear_acceleration.z;

  cout << "cnt = " << cnt << endl; 

  wb << G_x,G_y,G_z;
  gg << A_x,A_y,A_z; 
  
  sigma = sqrt(wb.transpose()*wb)*t;

  B << 0,-G_z*t,G_y*t,
       G_z*t,0,-G_x*t,
       -G_y*t,G_x*t,0;

  cout << "B =" << B << endl;

  C = C*(I+sin(sigma)/sigma*B+((1-cos(sigma))/sigma/sigma)*B*B);

//  C = C*(I + (sin(sigma)/sigma)*B + ((1-cos(sigma))/(sigma*sigma))*B*B);

  cout << "C = " << endl << C << endl;
//  cout << "gg = " << endl << gg << endl;

  if (cnt == 0){
    gg1 = C*gg;
    cnt = 1;
  }

  ab << A_x,A_y,A_z;
  ag = C*ab; 
  vg = vg + t*(ag-gg1);
  sg = sg + t*vg;

  cout << "gg1 =" << gg1 << endl;

  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 20000);
  ros::Rate r(30);

//  while (ros::ok())
//  {

//    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "points_and_lines";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.5;
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    geometry_msgs::Point p;
      p.x = sg(0,0);
      p.y = sg(1,0);
      p.z = sg(2,0);
    line_strip.points.push_back(p);
    marker_pub.publish(line_strip);

//    cout << "sg = " << sg << endl;
//    r.sleep();

    cnt1 += 1;
    
    cout << "cnt1 = " << cnt1 << endl;

}

int main(int argc, char **argv){

  C.setIdentity();
//  C1.setZero();
  I.setIdentity();
  B.setZero();
  gg.setZero();
  gg1.setZero();
  vg.setZero();
  wb.setZero();
  ab.setZero();
  ag.setZero();
  sg.setZero();
  cnt = 0;
  cnt1 = 0;
  ros::init(argc, argv, "IMU");
  ros::NodeHandle nh;
//  while(ros::ok()){
  ros::Subscriber sub = nh.subscribe("/imu/data", 20000, chatterCallback);

   ros::spin();
 //  }
  
}
