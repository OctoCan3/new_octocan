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
#include <kdl_conversions/kdl_msg.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include "kdl/frames.hpp"
#include <tf_conversions/tf_kdl.h>
#include "hebiros/AddGroupFromNamesSrv.h"
#include "actionlib/client/simple_action_client.h"
#include "hebiros/TrajectoryAction.h"

//custom message
#include <re2_flexiforce/NAC_input.h>

using namespace hebiros;


static bool receivedjointFeedback_g = false;
const int NUM_OF_JOINTS {6};
std::string group_name = "arna_manipulator";

re2_flexiforce::NAC_input desiredTrajData;

int main(int argc, char **argv ){

    ros::init(argc, argv, "NAC_Traj_Test" );
    ros::NodeHandle n;
    ros::Rate loop_rate( 200 ); // Hz

    ROS_WARN("Sleeping for 3 seconds for HEBI node and joint_state_pub to fully boot up");
    ros::Duration(5.0).sleep();

    ros::ServiceClient add_group_client =
            n.serviceClient<AddGroupFromNamesSrv>( "/hebiros/add_group_from_names" );
    bool groupCreated = false;
    AddGroupFromNamesSrv add_group_srv;
    add_group_srv.request.group_name = group_name;
    add_group_srv.request.families = {"ARNA"};
    add_group_srv.request.names = {
            "Shoulder_Yaw",
            "Shoulder_Pitch",
            "Upper_Arm_Roll",
            "Elbow_Pitch",
            "Wrist_Roll",
            "Wrist_Pitch" };
    groupCreated = add_group_client.call( add_group_srv );
    if ( true == groupCreated )
    {
        // Specific topics and services will now be available under this group's namespace
        ROS_INFO( "Created group: %s", group_name.c_str() );
    }
    else
    {
        ROS_FATAL( "Failed to create group: %s", group_name.c_str() );
        return -1;
    }

    // Now that there's a group, it should be getting feedback.  We're going
    // to want that feedback for our starting point of our trajectory.
    


    desiredTrajData.name.push_back( "ARNA/Shoulder_Yaw" );   // [0]
    desiredTrajData.name.push_back( "ARNA/Shoulder_Pitch" ); // [1]
    desiredTrajData.name.push_back( "ARNA/Upper_Arm_Roll" ); // [2]
    desiredTrajData.name.push_back( "ARNA/Elbow_Pitch" );    // [3]
    desiredTrajData.name.push_back( "ARNA/Wrist_Roll" );     // [4]
    desiredTrajData.name.push_back( "ARNA/Wrist_Pitch" );    // [5]

    desiredTrajData.position.resize( 6 );
    desiredTrajData.velocity.resize( 6 );
    desiredTrajData.acceleration.resize( 6 );

    desiredTrajData.position[0] = 0;
    desiredTrajData.position[1] = 0;
    desiredTrajData.position[2] = 0;
    desiredTrajData.position[3] = 0;
    desiredTrajData.position[4] = 0;
    desiredTrajData.position[5] = 0;

    desiredTrajData.velocity[0] = 0;
    desiredTrajData.velocity[1] = 0;
    desiredTrajData.velocity[2] = 0;
    desiredTrajData.velocity[3] = 0;
    desiredTrajData.velocity[4] = 0;
    desiredTrajData.velocity[5] = 0;

    desiredTrajData.acceleration[0] = 0;
    desiredTrajData.acceleration[1] = 0;
    desiredTrajData.acceleration[2] = 0;
    desiredTrajData.acceleration[3] = 0;
    desiredTrajData.acceleration[4] = 0;
    desiredTrajData.acceleration[5] = 0;

    
    ros::Publisher nac_pub = n.advertise<re2_flexiforce::NAC_input>("/nac_input", 1);

    float sampT = 0;
    float x = 0;
    float x_d = 0;
    float x_dd = 0;

    desiredTrajData.header.stamp;
    nac_pub.publish(desiredTrajData);
    ros::Duration(5.0).sleep();

    while(ros::ok()){
        
        x = (M_PI/8)*sin((M_PI/8)*sampT);
        x_d = (M_PI/8)*(M_PI/8)*cos((M_PI/8)*sampT);
        x_dd = -pow((M_PI/8),2)*(M_PI/8)*sin((M_PI/8)*sampT);
        sampT+=0.0025;
        if (sampT >= 16.0025){sampT=0;}

        desiredTrajData.header.stamp;
        
        desiredTrajData.position[0] = x;
        desiredTrajData.velocity[0] = x_d;
        desiredTrajData.acceleration[0] = x_dd;
        std::cout << "vel: " << x_d << " accel: " << x_dd << std::endl;
        nac_pub.publish(desiredTrajData);
        loop_rate.sleep();
    }
    


    return 0;
}
