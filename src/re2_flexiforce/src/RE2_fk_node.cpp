/*
 * Author: Jordan Dowdy
 * Date: 17/08/22
 * Version: 2.0
 * 
 * 
 * FK for RE2
 * 
 * 
 */

#include <iostream>
#include <string>
#include <fstream>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/LU>

// ROS libs
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/Pose.h>
#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "kdl/frames.hpp"
#include <tf_conversions/tf_kdl.h>
#include <cmath>
#include <complex>

// Custom Message for NAC
#include <re2_flexiforce/NAC_input.h>

geometry_msgs::Pose re2Pose;

float t1 = 0;
float t2 = 0;
float t3 = 0;
float t4 = 0;
float t5 = 0;
float t6 = 0;

KDL::Rotation rot;
float px;
float py;
float pz;

double quatX;
double quatY;
double quatZ;
double quatW;

float l0 = 15.5*0.0254+0.0365;
float l2 = 12.5*0.0254;
float l4 = 10*0.0254;

void joint_callback(re2_flexiforce::NAC_input data){


    t1 = data.position[0];
    t2 = data.position[1];
    t3 = data.position[2];
    t4 = data.position[3];
    t5 = data.position[4];
    t6 = data.position[5];


}

int main( int argc, char **argv ){

    ros::init(argc, argv, "re2_fk_ndoe" );
    ros::NodeHandle n;
    ros::Rate loop_rate( 350 ); // Hz

    std::string group_name = "arna_manipulator";
    ros::Subscriber feedback_subscriber = n.subscribe("nac_input", 1, joint_callback);
    ros::Publisher joint_pub = n.advertise<geometry_msgs::Pose>("re2_fk", 1);

    sleep(3);
    while(ros::ok()){

        ros::spinOnce();


        //x-axis rotation
        rot(0,0) = cos(t6)*(cos(t5)*(cos(t4)*(sin(t1)*sin(t3) - cos(t1)*cos(t3)*sin(t2)) - cos(t1)*cos(t2)*sin(t4)) + sin(t5)*(cos(t3)*sin(t1) + cos(t1)*sin(t2)*sin(t3))) - sin(t6)*(sin(t4)*(sin(t1)*sin(t3) - cos(t1)*cos(t3)*sin(t2)) + cos(t1)*cos(t2)*cos(t4));
        rot(1,0) = sin(t6)*(sin(t4)*(cos(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)) - cos(t2)*cos(t4)*sin(t1)) - cos(t6)*(cos(t5)*(cos(t4)*(cos(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)) + cos(t2)*sin(t1)*sin(t4)) + sin(t5)*(cos(t1)*cos(t3) - sin(t1)*sin(t2)*sin(t3)));
        rot(2,0) = - cos(t6)*(cos(t5)*(sin(t2)*sin(t4) - cos(t2)*cos(t3)*cos(t4)) + cos(t2)*sin(t3)*sin(t5)) - sin(t6)*(cos(t4)*sin(t2) + cos(t2)*cos(t3)*sin(t4));

        //y-axis rotation
        rot(0,1) = - cos(t6)*(sin(t4)*(sin(t1)*sin(t3) - cos(t1)*cos(t3)*sin(t2)) + cos(t1)*cos(t2)*cos(t4)) - sin(t6)*(cos(t5)*(cos(t4)*(sin(t1)*sin(t3) - cos(t1)*cos(t3)*sin(t2)) - cos(t1)*cos(t2)*sin(t4)) + sin(t5)*(cos(t3)*sin(t1) + cos(t1)*sin(t2)*sin(t3)));
        rot(1,1) = cos(t6)*(sin(t4)*(cos(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)) - cos(t2)*cos(t4)*sin(t1)) + sin(t6)*(cos(t5)*(cos(t4)*(cos(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)) + cos(t2)*sin(t1)*sin(t4)) + sin(t5)*(cos(t1)*cos(t3) - sin(t1)*sin(t2)*sin(t3)));
        rot(2,1) = sin(t6)*(cos(t5)*(sin(t2)*sin(t4) - cos(t2)*cos(t3)*cos(t4)) + cos(t2)*sin(t3)*sin(t5)) - cos(t6)*(cos(t4)*sin(t2) + cos(t2)*cos(t3)*sin(t4));

        //z-axis rotation
        rot(0,2) = cos(t5)*(cos(t3)*sin(t1) + cos(t1)*sin(t2)*sin(t3)) - sin(t5)*(cos(t4)*(sin(t1)*sin(t3) - cos(t1)*cos(t3)*sin(t2)) - cos(t1)*cos(t2)*sin(t4));
        rot(1,2) = sin(t5)*(cos(t4)*(cos(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)) + cos(t2)*sin(t1)*sin(t4)) - cos(t5)*(cos(t1)*cos(t3) - sin(t1)*sin(t2)*sin(t3));
        rot(2,2) = sin(t5)*(sin(t2)*sin(t4) - cos(t2)*cos(t3)*cos(t4)) - cos(t2)*cos(t5)*sin(t3);

        //position
        px = l4*(sin(t4)*(sin(t1)*sin(t3) - cos(t1)*cos(t3)*sin(t2)) + cos(t1)*cos(t2)*cos(t4)) + l2*cos(t1)*cos(t2);
        py = l2*cos(t2)*sin(t1) - l4*(sin(t4)*(cos(t1)*sin(t3) + cos(t3)*sin(t1)*sin(t2)) - cos(t2)*cos(t4)*sin(t1));
        pz = l0 + l4*(cos(t4)*sin(t2) + cos(t2)*cos(t3)*sin(t4)) + l2*sin(t2);

        //turning rotation matrix to quaternion
        rot.GetQuaternion(quatX,quatY,quatZ,quatW);

        
        //creating pose msg
        re2Pose.position.x = px;
        re2Pose.position.y = py;
        re2Pose.position.z = pz;
        re2Pose.orientation.x = quatX;
        re2Pose.orientation.y = quatY;
        re2Pose.orientation.z = quatZ;
        re2Pose.orientation.w = quatW;

        joint_pub.publish(re2Pose);
        loop_rate.sleep();


    }


}