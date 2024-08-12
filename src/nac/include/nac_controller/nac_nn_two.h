/*
 * nac_nn_two.h
 *
 *  Created on: Jul 30, 2017
 *      Author: Sumit Kumar Das
 */

#ifndef NAC_CONTROLLERS_INCLUDE_NAC_CONTROLLERS_NAC_NN_TWO_H_
#define NAC_CONTROLLERS_INCLUDE_NAC_CONTROLLERS_NAC_NN_TWO_H_

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <iostream>
#include <math.h>


class TwoLayerNeuralNetworkController
{
private:

	bool updateWeights;
	bool updateInnerWeights;

	int num_Inputs  ; // n Size of the inputs
	int num_Outputs ; // m Size of the outputs
	int num_Hidden  ; // l Size of the hidden layer
	int num_Error   ; // filtered error
	int num_Joints  ; // number of joints.

	Eigen::MatrixXd V_;
	Eigen::MatrixXd W_;
	Eigen::MatrixXd V_next_;
	Eigen::MatrixXd W_next_;

	Eigen::MatrixXd V_trans;
	Eigen::MatrixXd W_trans;
	Eigen::MatrixXd G;
	Eigen::MatrixXd F;
	Eigen::MatrixXd L;
	Eigen::MatrixXd Z;

	Eigen::MatrixXd x;
	Eigen::MatrixXd y;
	Eigen::MatrixXd hiddenLayer_out;
	Eigen::MatrixXd hiddenLayerIdentity;
	Eigen::MatrixXd hiddenLayer_in;
	Eigen::MatrixXd outputLayer_out;
	Eigen::MatrixXd sigmaPrime;
	Eigen::MatrixXd r;
	Eigen::MatrixXd r_tran;
	Eigen::MatrixXd vRobust;
	Eigen::MatrixXd sigmaPrimeTrans_W_r;

	double kappa;
	Eigen::MatrixXd Kv;
	Eigen::MatrixXd lambda;
	double Kz;
	double Zb;
	double nnF;
	double nnG;
	double nn_ON;

	double feedForwardForce;

	double delT; // Time step

public:

	TwoLayerNeuralNetworkController();

	void changeNNstructure( 	int para_num_Inputs  ,
								int para_num_Outputs ,
								int para_num_Hidden  ,
								int para_num_Error   ,
								int para_num_Joints   );

	void init( double p_kappa             ,
	           Eigen::MatrixXd & p_Kv     ,
	           Eigen::MatrixXd & p_lambda ,
			   double p_Kz                ,
			   double p_Zb                ,
			   double p_ffForce           ,
			   double p_nnF               ,
			   double p_nnG               ,
			   double p_nn_ON              );

	void updateDelT( double p_delT );

	void UpdateCart(	 Eigen::VectorXd & X     ,
	                 	 	 Eigen::VectorXd & Xd    ,
							 Eigen::VectorXd & X_m   ,
							 Eigen::VectorXd & Xd_m  ,
							 Eigen::VectorXd & Xdd_m ,
							 Eigen::VectorXd & q     ,
							 Eigen::VectorXd & qd    ,
							 Eigen::VectorXd & t_r   ,
							 Eigen::VectorXd & tau    );

	void UpdateJoint(  Eigen::VectorXd & q     ,
                          	  Eigen::VectorXd & qd    ,
							  Eigen::VectorXd & q_m   ,
							  Eigen::VectorXd & qd_m  ,
							  Eigen::VectorXd & qdd_m ,
							  Eigen::VectorXd & t_r   ,
							  Eigen::VectorXd & tau    );

	void Update(  Eigen::VectorXd & q    ,
                     	 Eigen::VectorXd & qd   ,
						 Eigen::VectorXd & q_m  ,
						 Eigen::VectorXd & qd_m ,
						 Eigen::VectorXd & t_r  ,
						 Eigen::VectorXd & tau   );

	Eigen::MatrixXd sigmoid( Eigen::MatrixXd & z ) const					;

	double getInnerWeightsNorm()							;
	double getOuterWeightsNorm()							;
	Eigen::MatrixXd	getInnerWeights()						;   // TODO return V_ instead
	Eigen::MatrixXd	getOuterWeights()						;   // TODO return W_ instead
	void setInnerWeights(Eigen::MatrixXd V_trans_)			;	// TODO check size
	void setOuterWeights(Eigen::MatrixXd W_trans_)			;
	void setUpdateWeights(bool p_updateWeights)				;
	void setUpdateInnerWeights(bool p_updateInnerWeights)	;


	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};







#endif /* NAC_CONTROLLERS_INCLUDE_NAC_CONTROLLERS_NAC_NN_TWO_H_ */
