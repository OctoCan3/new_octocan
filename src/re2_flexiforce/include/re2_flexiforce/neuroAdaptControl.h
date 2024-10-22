#include <iostream>
#include <string>
#include <fstream>
#include <ctime>          // Updated from <time.h>
#include <cstdlib>        // Updated from <stdlib.h>
#include <cstdio>         // Updated from <stdio.h>
#include <csignal>        // Updated from <signal.h>
#include <unistd.h>

// ROS2 libraries
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
// #include "hebiros/AddGroupFromNamesSrv.h"  // Uncomment and update when migrating this service to ROS2
#include "rclcpp_action/rclcpp_action.hpp"  // Consider replacing with ROS2 action client
// #include "hebiros/TrajectoryAction.h"  // Uncomment and update for ROS2 action interface
#include "sensor_msgs/msg/joy.hpp"

// tf2 and related libraries
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // Replaces tf/transform_datatypes.h in ROS2

// KDL libraries
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>

// Consider replacing boost::scoped_ptr with std::unique_ptr or std::shared_ptr
#include <memory> // For std::unique_ptr, std::shared_pt

/*
#include "hebi_cpp_api/lookup.hpp"
#include "hebi_cpp_api/group_command.hpp"
#include "hebi_cpp_api/group_feedback.hpp"
*/

//matrix math libs
#include <Eigen/Core>
#include <Eigen/LU>

#include "nac_controller/nac_nn_two.h"

//using namespace hebiros; //hebiros no longer needed
const int NUM_OF_JOINTS {6};
const int NUMBER_OF_JOINTS {6};
sensor_msgs::JointState jointFeedback;
sensor_msgs::JointState joint_command_msg;
sensor_msgs::JointState desiredCartData;
std::string group_name = "arna_manipulator";

//#############################
//# NN controller             #
//#############################
/*
double                                                      num_Inputs                      =    30                                     ; // n Size of the inputs
double                                                      num_Outputs                     =    NUMBER_OF_JOINTS                       ; // m Size of the outputs
double                                                      num_Hidden                      =    20                                     ; // l Size of the hidden layer
double                                                      num_Error                       =    NUMBER_OF_JOINTS                       ; // filtered error
double                                                      num_Joints                      =    NUMBER_OF_JOINTS                       ; // number of joints.

double                                                      kappa                           =    0.1                                    ;
double                                                      Kv                              =    0.5                                    ;
double                                                      lambda                          =    2.5                                    ;
double                                                      Kz                              =    0.0008                                   ;
double                                                      Zb                              =    100                                    ;
double                                                      nnF                             =    2                                     ;
double                                                      nnG                             =    50                                     ;
double                                                      nn_ON                           =     1                                     ;

*/
double                                                      num_Inputs                      =    30                                     ; // n Size of the inputs
double                                                      num_Outputs                     =    NUMBER_OF_JOINTS                       ; // m Size of the outputs
double                                                      num_Hidden                      =    20                                     ; // l Size of the hidden layer
double                                                      num_Error                       =    NUMBER_OF_JOINTS                       ; // filtered error
double                                                      num_Joints                      =    NUMBER_OF_JOINTS                       ; // number of joints.

double                                                      kappa                           =    0.08                                    ;
double                                                      Kv                              =    0.25                                    ;
double                                                      lambda                          =    1.1                                   ;
double                                                      Kz                              =    0.0008                                   ;
double                                                      Zb                              =    100                                    ;
double                                                      nnF                             =    2                                     ;
double                                                      nnG                             =    50                                     ;
double                                                      nn_ON                           =    1                                     ;

long double 								                xm[6]						    =	{0} 		                            ;
long double 								                xd[6]						    =	{0} 		                            ;
long double 								                xdd[6]						    =	{0} 		                            ;

Eigen::VectorXd 											jointTorqueValue															;
Eigen::VectorXd 											dxlCurrentPos																;
Eigen::VectorXd 											dxlCurrentVel																;

Eigen::VectorXd 											dxlFiltPos																	;
Eigen::VectorXd 											dxlPastFiltPos																;
Eigen::VectorXd 											dxlFiltVel																	;
Eigen::VectorXd 											dxlPastFiltVel																;


Eigen::VectorXd 							                controlForce									                            ;
Eigen::VectorXd 							                baseTwist										                            ;
Eigen::VectorXd 							                userForce										                            ;

Eigen::VectorXd 							                X_m												                            ;
Eigen::VectorXd 							                Xd_m											                            ;
Eigen::VectorXd 							                Xdd_m											                            ;

TwoLayerNeuralNetworkController 							nnController																;
double 														errorTraining					=	0										;

//#############################
//# Moving Window             #
//#############################
int 											window						=	10										;
int 											vectorCntr 					=	0										;
Eigen::MatrixXd 							    avgVelocity;	
Eigen::MatrixXd nnInpData;
Eigen::MatrixXd nnOutData;
Eigen::MatrixXd nnTwistData;
											                ;
class NACNode : public rclcpp::Node {
public:
    NACNode();

private:
    void setupMatrices();
    void cart_callback(const re2_flexiforce::msg::NACInput::SharedPtr msg);
    void joint_callback(const sensor_msgs::JointState msg);

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_publisher;
    rclcpp::Subscription<re2_flexiforce::msg::NACInput>::SharedPtr cart_subscriber;
    rclcpp::TimerBase::SharedPtr timer_;
};