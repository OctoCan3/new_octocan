/*
 * Author: Jordan Dowdy
 * Date: 17/08/22
 * Version: 2.0
 * 
 * 
 * IK for RE2
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
#include <cmath>
#include <complex>

// ROS libs
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_kdl.h>
#include "kdl/frames.hpp"
#include <kdl_conversions/kdl_msg.h>
#include <kdl_parser/kdl_parser.hpp>
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include <kdl/frames_io.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <boost/scoped_ptr.hpp>

// Custom Message for NAC
#include <re2_flexiforce/NAC_input.h>

const int NUM_OF_JOINTS {6};
std::string group_name = "arna_manipulator";
sensor_msgs::JointState jointFeedback;
re2_flexiforce::NAC_input desiredCartData;
float solSet[8][6] = { {0,0,0,0,0,0}, 
                       {0,0,0,0,0,0},
                       {0,0,0,0,0,0},
                       {0,0,0,0,0,0},
                       {0,0,0,0,0,0},
                       {0,0,0,0,0,0},
                       {0,0,0,0,0,0},
                       {0,0,0,0,0,0} };
float jointChange[8] = {0,0,0,0,0,0,0};

//used to iterate through solution sets
float signs[2] = {1, -1};

//pre-allocate space for variables
float c,s,a,b,d,f,P,Q,px,py,pz,x1,x2,x3,y_1,y2,y3,z1,z2,z3;
KDL::Rotation rot;
float t1,t2,t3,t4,t5,t6;
    //link lenghts
float l0 = 15.5*0.0254+0.0365;
float l2 = 12.5*0.0254;
float l4 = 10*0.0254;
    //used to find closest solution
int min_index = 0;
float min_val = 100000;

int NaNSol = 0;

float maxVel = 5;
float maxAcc = 5;
//min velocity before it clips to 0
float minVel = 0.05;
ros::Time lastTime;
float deltaT = 0;

void joint_callback(sensor_msgs::JointState data){

    jointFeedback = data;

}

float LTE(const float &ac,const float &bs,const float &df,const float &sign){
    /*
    LTE (Linear Trigonometric Equation) finds cos and sin which can be used
    to find theta

    sign is either -1 or 1 and just determines which solution is given.

    The equation must have the form ac*cos+bs*sin-df=0 and cos and sin 
    can be found with the following equations.
    */

    if((pow(ac,2)+pow(bs,2)-pow(df,2))<0){
        c = -(-ac*df + sign*bs*std::real(sqrt(pow(ac,2)+pow(bs,2)-pow(df,2))))/(pow(ac,2)+pow(bs,2));
        s = (bs*df+sign*ac*std::real(sqrt(pow(ac,2)+pow(bs,2)-pow(df,2))))/(pow(ac,2)+pow(bs,2));
    }

    else {
    c = -(-ac*df + sign*bs*sqrt(pow(ac,2)+pow(bs,2)-pow(df,2)))/(pow(ac,2)+pow(bs,2));
    s = (bs*df+sign*ac*sqrt(pow(ac,2)+pow(bs,2)-pow(df,2)))/(pow(ac,2)+pow(bs,2));
    }


    return atan2(s,c);


}

void IK_callback(geometry_msgs::Pose Td){
    /*
    geometry_msgs::Pose Td -> void (updates re2_flexiforce::NAC_input desiredCartData)

    This function takes in a desired pose and preforms inverse kinematics to determine
    joint configurations to achieve this. 
    

    */
    //ros uses quaternions so need to convert to rotation matrix
    rot = KDL::Rotation::Quaternion(Td.orientation.x,Td.orientation.y,Td.orientation.z,Td.orientation.w);
    
    //assiging varriable names
    px = Td.position.x;
    py = Td.position.y;
    pz = Td.position.z;
    x1 = rot(0,0);
    x2 = rot(1,0);
    x3 = rot(2,0);
    y_1 = rot(0,1);
    y2 = rot(1,1);
    y3 = rot(2,1);
    z1 = rot(0,2);
    z2 = rot(1,2);
    z3 = rot(2,2);
        

    //RE2 has 8 different solutions so we need to iterate through the equations
    int solIter = 0;
    for(float sign1 : signs){
        for(float sign2 : signs){
            for(float sign3 : signs){

                //t4 sol
                c = (pow(l0,2) - 2*l0*pz + pow(px,2) + pow(py,2) + pow(pz,2) - pow(l2,2) - pow(l4,2))/(2*l2*l4);
                s = sqrt(1-pow(c,2));
                t4 = atan2(sign1*s,c);
                solSet[solIter][3] = t4;

                //t6 sol
                a = (2*l4*px*y_1 - 2*l0*l4*y3 + 2*l4*py*y2 + 2*l4*pz*y3);
                b = (2*l4*px*x1 - 2*l0*l4*x3 + 2*l4*py*x2 + 2*l4*pz*x3);
                d = (-pow(l0,2) - pow(l4,2) - pow(px,2) - pow(py,2) - pow(pz,2) + pow(l2,2) + 2*l0*pz);
                t6 = LTE(a,b,d,sign2);
                solSet[solIter][5] = t6;
                
                //t1 sol
                b = (- px - l4*y_1*cos(t6) - l4*x1*sin(t6));
                a = (py + l4*y2*cos(t6) + l4*x2*sin(t6));
                t1 = LTE(a,b,0,sign3);
                solSet[solIter][0] = t1;

                //t2 sol
                s = (pz - l0 + l4*y3*cos(t6) + l4*x3*sin(t6))/l2;
                c = (px*cos(t1) + py*sin(t1) + l4*x1*cos(t1)*sin(t6) + l4*y2*cos(t6)*sin(t1) + l4*x2*sin(t1)*sin(t6) + l4*y_1*cos(t1)*cos(t6))/l2;
                t2 = atan2(s,c);
                solSet[solIter][1] = t2;

                //t3 sol
                //singularity at t4 = 0 so this assigns t3 to a set value
                if(sin(t4) == 0){
                    t3 = 0;
                }
                else {
                    c =(pz*cos(t2) - l0*cos(t2) - px*cos(t1)*sin(t2) - py*sin(t1)*sin(t2))/(l4*sin(t4));
                    s = (px*sin(t1) - py*cos(t1))/(l4*sin(t4));
                    t3 = atan2(s,c);
                    }
                solSet[solIter][2] = t3;

                //t5 sol
                s = x1*cos(t3)*cos(t6)*sin(t1) - x2*cos(t1)*cos(t3)*cos(t6) - x3*cos(t2)*cos(t6)*sin(t3) + y2*cos(t1)*cos(t3)*sin(t6) - y_1*cos(t3)*sin(t1)*sin(t6) + y3*cos(t2)*sin(t3)*sin(t6) + x1*cos(t1)*cos(t6)*sin(t2)*sin(t3) + x2*cos(t6)*sin(t1)*sin(t2)*sin(t3) - y_1*cos(t1)*sin(t2)*sin(t3)*sin(t6) - y2*sin(t1)*sin(t2)*sin(t3)*sin(t6);
                c = z1*cos(t3)*sin(t1) - z2*cos(t1)*cos(t3) - z3*cos(t2)*sin(t3) + z1*cos(t1)*sin(t2)*sin(t3) + z2*sin(t1)*sin(t2)*sin(t3);
                
                t5 = atan2(s,c);
                solSet[solIter][4] = t5;

                //next iteration of solutions
                solIter++;


            }
        }
    }

    deltaT = 0.7;
    //deltaT = 0.5;
    //determine closest solution to current configeration 
    //also checks to make sure solution is real and in workspace
    NaNSol = 0;
    for(int i=0; i <=7; i++){
        jointChange[i] = 0;
        
        for(int j=0; j<=5; j++)
        {   
            
            if(solSet[i][j] != solSet[i][j]){continue; NaNSol++;} 
            if(solSet[i][j] > 2.01*M_PI || solSet[i][j] < -2.01*M_PI){continue; NaNSol++;}
            jointChange[i] = jointChange[i] + fabs((solSet[i][j] - jointFeedback.position[j]));
        }
        
        if(jointChange[i] < min_val){min_val = jointChange[i]; min_index = i;}

    }   

    if(NaNSol >= 6){return;}
    
    //updating message
    for(int i=0; i<6; i++){

    desiredCartData.position[i] = solSet[min_index][i];
    
    //std::cout << "requested vel and accel: " <<  (desiredCartData.position[i] - jointFeedback.position[i])/0.5 << ", " << (desiredCartData.velocity[i] - jointFeedback.velocity[i])/(deltaT/2) << std::endl;
    desiredCartData.velocity[i] = (desiredCartData.position[i] - jointFeedback.position[i])/0.7;
    desiredCartData.acceleration[i] = (desiredCartData.velocity[i] - jointFeedback.velocity[i])/(deltaT/2);
    /*
    //backwards difference for velocity
    if((desiredCartData.position[i] - jointFeedback.position[i])/deltaT > maxVel)
    {
        desiredCartData.velocity[i] = maxVel;
    }

    else if ((desiredCartData.position[i] - jointFeedback.position[i])/deltaT < -maxVel)
    {
        desiredCartData.velocity[i] = -maxVel;
    }

    else if ((desiredCartData.position[i] - jointFeedback.position[i])/deltaT > -minVel && (desiredCartData.position[i] - jointFeedback.position[i])/deltaT < 0 )
    {
        desiredCartData.velocity[i] = 0;
    }
    else if ((desiredCartData.position[i] - jointFeedback.position[i])/deltaT < minVel && (desiredCartData.position[i] - jointFeedback.position[i])/deltaT > 0 )
        {
            desiredCartData.velocity[i] = 0;
        }

    else{desiredCartData.velocity[i] = (desiredCartData.position[i] - jointFeedback.position[i])/deltaT;}

    //backwards difference for acceleration
    if((desiredCartData.velocity[i] - jointFeedback.velocity[i])/deltaT > maxAcc)
    {
        desiredCartData.acceleration[i] = maxAcc;
    }

    else if ((desiredCartData.velocity[i] - jointFeedback.velocity[i])/deltaT < -maxAcc)
    {
        desiredCartData.acceleration[i] = -maxAcc;
    }

    else if ((desiredCartData.velocity[i] - jointFeedback.velocity[i])/deltaT > -minVel && (desiredCartData.velocity[i] - jointFeedback.velocity[i])/deltaT < 0)
    {
        desiredCartData.acceleration[i] = 0;
    }


    else if ((desiredCartData.velocity[i] - jointFeedback.velocity[i])/deltaT > 0 && (desiredCartData.velocity[i] - jointFeedback.velocity[i])/deltaT < minVel)
    {
        desiredCartData.acceleration[i] = 0;
    }

    else{desiredCartData.acceleration[i] = (desiredCartData.velocity[i] - jointFeedback.velocity[i])/deltaT;}
    */
    }



}

int main( int argc, char **argv ){

    //ROS setup
    ros::init(argc, argv, "re2_ik_node" );
    ros::NodeHandle n;
    ros::Rate loop_rate( 300 ); // Hz, keep this above 150

    
    /*
    //this code is used for finding jacobian. Would be used for determing q_d desired but would need
    //sudo-inverse jacobian and am hesitant to use with RE2
    KDL::Tree  re2_tree;
    if(!kdl_parser::treeFromFile("/home/ngs/RE2_ws/src/re2_flexiforce/urdf/JD_changes_arna.urdf", re2_tree)){
        ROS_ERROR("URDF could not be parsed");
    }

    
    KDL::Chain re2_chain;
    re2_tree.getChain("base_link", "link_e2if", re2_chain); 
    KDL::ChainFkSolverPos_recursive fk_solver(re2_chain);
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    */

    //Subs and Pubs
    ros::Subscriber sub = n.subscribe("pose_d",1,IK_callback);
    ros::Subscriber feedback_subscriber = n.subscribe("/hebiros/" + group_name + "/feedback/joint_state", 1, joint_callback);
    ros::Publisher nac_pub = n.advertise<re2_flexiforce::NAC_input>("nac_input", 1);
    
    //initialize values of variables
    t1=t2=t3=t4=t5=t6=0;
    c=s=a=b=d=f=P=Q=px=py=pz=x1=x2=x3=y_1=y2=y3=z1=z2=z3=0;
    
    desiredCartData.name.push_back( "ARNA/Shoulder_Yaw" );   // [0]
    desiredCartData.name.push_back( "ARNA/Shoulder_Pitch" ); // [1]
    desiredCartData.name.push_back( "ARNA/Upper_Arm_Roll" ); // [2]
    desiredCartData.name.push_back( "ARNA/Elbow_Pitch" );    // [3]
    desiredCartData.name.push_back( "ARNA/Wrist_Roll" );     // [4]
    desiredCartData.name.push_back( "ARNA/Wrist_Pitch" );    // [5]

    desiredCartData.position.resize( 6 );
    desiredCartData.velocity.resize( 6 );
    desiredCartData.acceleration.resize( 6 );

    //initial configuration of arm. This is the config that will first be sent to nac until sensor is pressed
    desiredCartData.position[0] = 0;
    desiredCartData.position[1] = 0;
    desiredCartData.position[2] = 0;
    desiredCartData.position[3] = -1.01;
    desiredCartData.position[4] = 0;
    desiredCartData.position[5] = 1;

    desiredCartData.velocity[0] = 0.03;
    desiredCartData.velocity[1] = 0.03;
    desiredCartData.velocity[2] = 0.03;
    desiredCartData.velocity[3] = 0.03;
    desiredCartData.velocity[4] = 0.03;
    desiredCartData.velocity[5] = 0.03;

    desiredCartData.acceleration[0] = 0.03;
    desiredCartData.acceleration[1] = 0.03;
    desiredCartData.acceleration[2] = 0.03;
    desiredCartData.acceleration[3] = 0.03;
    desiredCartData.acceleration[4] = 0.03;
    desiredCartData.acceleration[5] = 0.03;

    while(ros::ok()){

        ros::spinOnce();
        desiredCartData.header.stamp;
        nac_pub.publish(desiredCartData);
        lastTime = ros::Time::now();
        loop_rate.sleep();

    }

}
