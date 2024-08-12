//
// Created by Jordan Dowdy on 8/29/22.
//
//

#include <string>
#include <iostream>

//ROS stuff
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "hebiros/AddGroupFromNamesSrv.h"
#include "actionlib/client/simple_action_client.h"
#include "hebiros/TrajectoryAction.h"



sensor_msgs::JointState cur_jnt_pos;

void joint_callback(sensor_msgs::JointState data){


    cur_jnt_pos = data;
    

}



int main(int argc, char **argv){

    ros::init( argc, argv, "joint_state_node" );
    ros::NodeHandle n;
    ros::Rate loop_rate( 100 );
    std::string group_name = "arna_manipulator";
    
    ros::Duration(4.5).sleep();
    ros::Subscriber feedback_subscriber = n.subscribe("/hebiros/" + group_name + "/feedback/joint_state", 100, joint_callback);

    
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);


    
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
	
	if (cur_jnt_pos.name.empty()){
	    continue;
	}
        cur_jnt_pos.name[0] = "joint_1";
        cur_jnt_pos.name[1] = "joint_2";
        cur_jnt_pos.name[2] = "joint_3";
        cur_jnt_pos.name[3] = "joint_4";
        cur_jnt_pos.name[4] = "joint_5";
        cur_jnt_pos.name[5] = "joint_6";
	
        joint_pub.publish(cur_jnt_pos);

	

    	}

    return 0;
}


