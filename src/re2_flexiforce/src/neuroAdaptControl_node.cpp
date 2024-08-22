/*
 * Author: Jordan Dowdy
 * Date: 17/08/22
 * Version: 2.0
 * 
 * 
 * NAC stuffs.
 * 
 * 
 */


#include "re2_flexiforce/neuroAdaptControl.h" 

// Custom Message for NAC
#include <re2_flexiforce/NAC_input.h>

void joint_callback(sensor_msgs::JointState data){

    dxlCurrentPos << data.position[0], data.position[1], data.position[2], 
                     data.position[3], data.position[4], data.position[5]; 

    dxlCurrentVel << data.velocity[0], data.velocity[1], data.velocity[2],
                     data.velocity[3], data.velocity[4], data.velocity[5];

}


void cart_callback(re2_flexiforce::NAC_input data){
    
    X_m << data.position[0], data.position[1], data.position[2], 
           data.position[3], data.position[4], data.position[5];
    
	Xd_m << data.velocity[0], data.velocity[1], data.velocity[2], 
            data.velocity[3], data.velocity[4], data.velocity[5];
    

    Xdd_m << data.acceleration[0], data.acceleration[1], 
             data.acceleration[2], data.acceleration[3], 
             data.acceleration[4], data.acceleration[5];
			 
	/*
    Xd_m << data.velocity[0], data.velocity[1], data.velocity[2], 
            data.velocity[3], data.velocity[4], data.velocity[5];
    

    Xdd_m << data.acceleration[0], data.acceleration[1], 
             data.acceleration[2], data.acceleration[3], 
             data.acceleration[4], data.acceleration[5];
    */
    
}

void setupMatrices(){
	jointTorqueValue.resize(NUMBER_OF_JOINTS, 1)			;
	dxlCurrentPos.resize(NUMBER_OF_JOINTS, 1)				;
	dxlCurrentVel.resize(NUMBER_OF_JOINTS, 1)				;

	controlForce.resize(NUMBER_OF_JOINTS, 1)		        ;
	userForce.resize(NUMBER_OF_JOINTS, 1)			        ;
 
	X_m.resize(NUMBER_OF_JOINTS, 1)		                    ;
	Xd_m.resize(NUMBER_OF_JOINTS, 1)	                    ;
	Xdd_m.resize(NUMBER_OF_JOINTS, 1)	                    ;

	avgVelocity.resize(NUMBER_OF_JOINTS, window)			;

	baseTwist.resize(NUMBER_OF_JOINTS,1)					;

	nnController.changeNNstructure( num_Inputs  ,   // num_Inputs
									num_Outputs ,   // num_Outputs
									num_Hidden  ,   // num_Hidden
									num_Error   ,   // num_Error
									num_Joints );  	// num_Joints = num_Outputs for cart space

	Eigen::MatrixXd p_Kv     ;
	Eigen::MatrixXd p_lambda ;

	p_Kv     .resize( num_Outputs, 1 ) ;
	p_lambda .resize( num_Outputs, 1 ) ;

	avgVelocity << Eigen::MatrixXd::Zero(NUMBER_OF_JOINTS, window);

	p_Kv << Kv ,
			Kv ,
			Kv ,
			Kv ,
			Kv ,
			Kv;

	p_lambda << lambda ,
				lambda ,
				lambda ,
				lambda ,
				lambda ,
				lambda ;

	jointTorqueValue << 0.1,0.1,0.1,0.1,0.1,0.1;
	dxlCurrentPos<<0,0,0,0,0,0;
	dxlCurrentVel<<0.0,0.0,0.0,0.0,0.0,0.0;
	userForce << 0,0,0,0,0,0;
	controlForce << 0,0,0,0,0,0;
	nnController.init( kappa  ,
					   p_Kv     ,
					   p_lambda ,
					   Kz     ,
					   Zb     ,
					   1,
					   nnF    ,
					   nnG    ,
					   nn_ON   );
    X_m << 0, 0, 0, 0, 0, 0; 
    Xd_m << 0.0,0.0,0.0,0.0,0.0,0.0;
    Xdd_m << 0,0,0,0,0,0;
}

NACNode::NACNode() : Node("nac_node") {
    setupMatrices();

    // Publishers
    joint_command_publisher = this->create_publisher<sensor_msgs::msg::JointState>("/command/joint_state", 10);

    // Subscribers
    cart_subscriber = this->create_subscription<re2_flexiforce::msg::NACInput>(
        "/nac_input", 10, std::bind(&NACNode::cart_callback, this, std::placeholders::_1));

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(3), std::bind(&NACNode::update_control, this));
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "NAC_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(300);
    setupMatrices();
    ros::Duration(5.0).sleep();
    //ros::Publisher joint_command_publisher = n.advertise<sensor_msgs::JointState>("/hebiros/" + group_name + "/command/joint_state",1);
    
    //ros::Subscriber feedback_subscriber = n.subscribe("/hebiros/" + group_name + "/feedback/joint_state", 1, joint_callback);
    ros::Subscriber cart_subscriber = n.subscribe("/nac_input", 1, cart_callback);
    
    //joint_command_msg.name.push_back( "ARNA/Shoulder_Yaw" );   // [0]
    //joint_command_msg.name.push_back( "ARNA/Shoulder_Pitch" ); // [1]
    //joint_command_msg.name.push_back( "ARNA/Upper_Arm_Roll" ); // [2]
    //joint_command_msg.name.push_back( "ARNA/Elbow_Pitch" );    // [3]
    //joint_command_msg.name.push_back( "ARNA/Wrist_Roll" );     // [4]
    //joint_command_msg.name.push_back( "ARNA/Wrist_Pitch" );    // [5]
    
    //joint_command_msg.position.resize( NUMBER_OF_JOINTS ); // Use position commands
    joint_command_msg.effort.resize( NUMBER_OF_JOINTS );
    
    ros::AsyncSpinner spinner(2);
    spinner.start();
	sleep(1);
    while(ros::ok()){
    

	ros::spinOnce();
    
	
	
        
       
        nnController.UpdateJoint( dxlCurrentPos     ,
							    dxlCurrentVel    ,
							    X_m   ,
                                Xd_m  ,
                                Xdd_m ,
                                userForce   ,			// Human force = 0
                                jointTorqueValue  );		// Output
   
        
        joint_command_msg.effort[0] = jointTorqueValue(0);
        joint_command_msg.effort[1] = jointTorqueValue(1);
        joint_command_msg.effort[2] = jointTorqueValue(2);
        joint_command_msg.effort[3] = jointTorqueValue(3);
        joint_command_msg.effort[4] = jointTorqueValue(4);
        joint_command_msg.effort[5] = jointTorqueValue(5);
        

        

        joint_command_publisher.publish(joint_command_msg);
        
        loop_rate.sleep();

    }

    return 0;
}
