/*
 * Author: Jordan Dowdy
 * Date: 17/08/22
 * Version: 2.0
 * 
 * 
 * OctoCan ROS Publisher
 * 
 *
 */

//ros libs
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
//octocan libs
extern "C" {
#include "octo_lib/skintalk.h"
#include "octo_lib/profile.h"
#include "octo_lib/util.h"
#include "octo_lib/layout.h"
}

#include "re2_flexiforce/OctoData.h"
//----OctoCan------
struct skin skin;
struct skin_pressure pressure = {};

void fullstop(int signum)
{
	// Tell the reader thread to stop
	skin_stop(&skin);
}

int main(int argc, char **argv ){
    
    if ( signal(SIGINT, fullstop) == SIG_ERR ) {
		return EXIT_FAILURE;
	}

    // ROS Setup -----------------------------------------------------------------
    ros::init(argc, argv, "octocan_pub");
    ros::NodeHandle n;
    ros::Rate loop_rate(100); // Hz

    // First initialize octocan device. It will assume a symlink
    skin_from_layout(&skin, "/dev/ttyUSB0", "/home/ngs/skinvis/octocan2.layout");
  
    skin_set_pressure_alpha(&skin, 0.8);
    //skin_set_pressure_alpha(&skin, 0.2);
    skin_read_profile(&skin, "/home/ngs/skinvis/octocan2.calib");
    skin_start(&skin);
    //This is baseline calibration
    std::cout << "Calibrating for 8 seconds... DO NOT TOUCH!" << std::endl;
    skin_calibrate_start(&skin);
    sleep(8);
    skin_calibrate_stop(&skin);
    std::cout << "Calibration Complete!" << std::endl;
    //publisher
    ros::Publisher octo_pub = n.advertise<re2_flexiforce::OctoData>("octocan",1);

    //octocan array
    re2_flexiforce::OctoData patchData;
    patchData.mag.resize(4);
    patchData.mean.resize(4);
    struct skin_pressure pressure = {};
    int sensNum = 5;
    float highestValMag[4] = {2000,2000,2000,2000};
    float highestValMean[4] = {200,200,200,200};
    float baseLineTrigger[4] = {};
    float mvgAvgMean[4][5];
    float mvgAvgMag[4][5];
    int j = 0;
    for(int i=3; i<=7; i+=2){
        for (int k = 0; k<5; k++){
            
            mvgAvgMean[j][k] = (float)skin_get_patch_mean(&skin,i);
            skin_get_patch_pressure(&skin, i, &pressure);
            mvgAvgMag[j][k] = (float)pressure.magnitude;
        
        }
        j++;
    }
    
    j = 0;
    for (int i=3; i<=7; i+=2){
            
            baseLineTrigger[j] = (float)skin_get_patch_mean(&skin,i);
            j++;
        } 
    j = 0;
    
    while(ros::ok()){

        //for(int i=1; i<=7; i=i+2){ patchData.sensorReading[i-1] = (float)skin_get_patch_mean(&skin,i);}
        
        //moving average...just shifting values down 1 index
        
        for (int i=0; i<4; i++){for (int k = 1; k<5; k++){
            mvgAvgMean[i][k-1] = mvgAvgMean[i][k]; mvgAvgMag[i][k-1] = mvgAvgMag[i][k];}}
        
        
        j = 0;
        for(int i=3; i<=7; i+=2){
            //adding newest value to moving avg
            mvgAvgMean[j][4] = (float)skin_get_patch_mean(&skin,i);
            skin_get_patch_pressure(&skin, i, &pressure);
            mvgAvgMag[j][4] = (float)pressure.magnitude;
            //averaging and adding to msg array
            patchData.mag[j] = (mvgAvgMag[j][0]+mvgAvgMag[j][1]+mvgAvgMag[j][2]+mvgAvgMag[j][3]+mvgAvgMag[j][4])/5;
            patchData.mean[j] = (mvgAvgMean[j][0]+mvgAvgMean[j][1]+mvgAvgMean[j][2]+mvgAvgMean[j][3]+mvgAvgMean[j][4])/5;
            j++;
        }

        //did this because I was too lazy to rewrite code elsewhere
        //TLDR this adds patch 2's (technically patch 1) data to the last index of the array 
        j = 3;
        mvgAvgMean[3][4] = (float)skin_get_patch_mean(&skin,2);
        skin_get_patch_pressure(&skin, 2, &pressure);
        mvgAvgMag[3][4] = (float)pressure.magnitude;
        patchData.mag[3] = (mvgAvgMag[j][0]+mvgAvgMag[j][1]+mvgAvgMag[j][2]+mvgAvgMag[j][3]+mvgAvgMag[j][4])/5;
        patchData.mean[3] = (mvgAvgMean[j][0]+mvgAvgMean[j][1]+mvgAvgMean[j][2]+mvgAvgMean[j][3]+mvgAvgMean[j][4])/5;
    
        /*
            //was testing a way to more accuratly determine sensor presses when they drift
            for(int i=0; i<sensNum; i++){
            //checks the derivative for a change of at least 400
            //based on testing this works pretty well and 
            if(patchData.mag[i]>highestValMag[i]){highestValMag[i] = patchData.mag[i];}
            if(patchData.mean[i]>highestValMean[i]){highestValMean[i] = patchData.mean[i];}
            }
            for(int i=0; i<sensNum; i++){
                if ((highestValMag[i] - 1000) > patchData.mag[i]){patchData.mag[i] = 0;}
                if ((highestValMean[i]-100) > patchData.mean[i]){patchData.mean[i] = 0;}
            }
        */
            octo_pub.publish(patchData);
            loop_rate.sleep();
    }


    return 0;
}
