/*
 * nac_nn_two.cpp
 *
 *  Created on: Jul 30, 2017
 *      Author: Sumit Kumar Das
 */

#include <nac_controller/nac_nn_two.h>

TwoLayerNeuralNetworkController::TwoLayerNeuralNetworkController()
{
	updateWeights = true;
	updateInnerWeights = true;

	  changeNNstructure( 14 ,   // num_Inputs
						 2  ,   // num_Outputs
						 10 ,   // num_Hidden
						 2  ,   // num_Error
						 2   ); // num_Joints

	delT = 0.001; /// 1000 Hz by default

	Eigen::MatrixXd p_Kv     ;
	Eigen::MatrixXd p_lambda ;

	p_Kv                  .resize( 2, 1 ) ;
	p_lambda              .resize( 2, 1 ) ;

	  p_Kv << 2 ,
			  2 ;

	  p_lambda << 0.2 ,
				  0.2 ;

	  init( 0.07     ,
			p_Kv     ,
			p_lambda ,
			0        ,
			100      ,
			1        ,
			100      ,
			20       ,
			1         );
}

void TwoLayerNeuralNetworkController::changeNNstructure( int para_num_Inputs  ,
		int para_num_Outputs ,
		int para_num_Hidden  ,
		int para_num_Error   ,
		int para_num_Joints   )
{
	num_Inputs  = para_num_Inputs  ;
	num_Outputs = para_num_Outputs ;
	num_Hidden  = para_num_Hidden  ;
	num_Error   = para_num_Error   ;
	num_Joints  = para_num_Joints  ;

	V_        .resize( num_Inputs + 1              , num_Hidden               ) ;
	W_        .resize( num_Hidden                  , num_Outputs              ) ;
	V_next_   .resize( num_Inputs + 1              , num_Hidden               ) ;
	W_next_   .resize( num_Hidden                  , num_Outputs              ) ;

	V_trans   .resize( num_Hidden                  , num_Inputs + 1           ) ;
	W_trans   .resize( num_Outputs                 , num_Hidden               ) ;
	G         .resize( num_Inputs + 1              , num_Inputs + 1           ) ;
	F         .resize( num_Hidden                  , num_Hidden               ) ;
	L         .resize( num_Outputs                 , num_Outputs              ) ;
	Z         .resize( num_Hidden + num_Inputs + 1 , num_Hidden + num_Outputs ) ;

	x                   .resize( num_Inputs + 1, 1          ) ;
	y                   .resize( num_Outputs   , 1          ) ;
	hiddenLayer_out     .resize( num_Hidden    , 1          ) ;
	hiddenLayerIdentity .resize( num_Hidden    , num_Hidden ) ;
	hiddenLayer_in      .resize( num_Hidden    , 1          ) ;
	outputLayer_out     .resize( num_Outputs   , 1          ) ;
	sigmaPrime          .resize( num_Hidden    , num_Hidden ) ;
	r                   .resize( num_Error     , 1          ) ;
	r_tran              .resize( 1             , num_Error  ) ;
	vRobust             .resize( num_Outputs   , 1          ) ;
	sigmaPrimeTrans_W_r .resize( num_Hidden    , 1          ) ;

	hiddenLayerIdentity.setIdentity();
	F.setIdentity();
	G.setIdentity();
	L.setIdentity();

	W_.setZero();
	W_next_.setZero();
	V_.setZero();
	V_next_.setZero();

	// Very important
	Z.setZero();

}

void TwoLayerNeuralNetworkController::init( double p_kappa             ,
		   Eigen::MatrixXd & p_Kv     ,
		   Eigen::MatrixXd & p_lambda ,
	   double p_Kz                ,
	   double p_Zb                ,
	   double p_ffForce           ,
	   double p_nnF               ,
	   double p_nnG               ,
	   double p_nn_ON              )
{
	// Init Kv
	if(p_Kv.rows() == num_Error && p_Kv.cols() == 1 )
	{
		Kv = p_Kv.asDiagonal();
	}
	else if (p_Kv.rows() == num_Error && p_Kv.cols() == num_Error )
	{
		Kv = p_Kv;
	}
	else
	{
		std::cerr<<"Error in TwoLayerNeuralNetworkController::init";
		Kv.setZero();
	}

	// Init Lambda
	if(p_lambda.rows() == num_Error && p_lambda.cols() == 1 )
	{
		lambda = p_lambda.asDiagonal();
	}
	else if (p_lambda.rows() == num_Error && p_lambda.cols() == num_Error )
	{
		lambda = p_lambda;
	}
	else
	{
		std::cerr<<"Error in TwoLayerNeuralNetworkController::init";
		lambda.setZero();
	}

	std::cout<<"Kv:\n"<<Kv<<"\n---\n";
	std::cout<<"Kv*lambda:\n"<<Kv*lambda<<"\n---\n";

	kappa            = p_kappa   ;
	Kz               = p_Kz      ;
	Zb               = p_Zb      ;
	feedForwardForce = p_ffForce ;
	nnF              = p_nnF     ;
	nnG              = p_nnG     ;
	nn_ON            = p_nn_ON   ;

	F = nnF*F;
	G = nnG*G;

}

void TwoLayerNeuralNetworkController::updateDelT( double p_delT )
{
	delT = p_delT;
}


double TwoLayerNeuralNetworkController::getInnerWeightsNorm()
{
	return V_.norm();
}
double TwoLayerNeuralNetworkController::getOuterWeightsNorm()
{
	return W_.norm();
}
Eigen::MatrixXd	TwoLayerNeuralNetworkController::getInnerWeights()   // TODO return V_ instead
{
	return V_trans;
}
Eigen::MatrixXd	TwoLayerNeuralNetworkController::getOuterWeights()   // TODO return W_ instead
{
	return W_trans;
}
void TwoLayerNeuralNetworkController::setInnerWeights(Eigen::MatrixXd V_trans_)	// TODO check size
{
	V_next_ = V_trans_.transpose();
	V_ = V_trans_.transpose();
}
void TwoLayerNeuralNetworkController::setOuterWeights(Eigen::MatrixXd W_trans_)
{
	W_next_ = W_trans_.transpose();
	W_ = W_trans_.transpose();
}
void TwoLayerNeuralNetworkController::setUpdateWeights(bool p_updateWeights)
{
	updateWeights = p_updateWeights;
}
void TwoLayerNeuralNetworkController::setUpdateInnerWeights(bool p_updateInnerWeights)
{
	updateInnerWeights = p_updateInnerWeights;
}

void TwoLayerNeuralNetworkController::UpdateCart( Eigen::VectorXd & X     ,
                                                  Eigen::VectorXd & Xd    ,
                                                  Eigen::VectorXd & X_m   ,
                                                  Eigen::VectorXd & Xd_m  ,
                                                  Eigen::VectorXd & Xdd_m ,
                                                  Eigen::VectorXd & q     ,
                                                  Eigen::VectorXd & qd    ,
                                                  Eigen::VectorXd & t_r   ,
                                                  Eigen::VectorXd & tau    )
{
        // NN Input Vector
        x <<           1 ,
            (  X_m -  X) , //   q( 0 ) ;
            ( Xd_m - Xd) , //  qd( 0 ) ;
                    X_m  ,
                   Xd_m  ,
                  Xdd_m  ,
                      q  ,
                     qd  ;

        // x is global so not passed
        Update( X    ,
                Xd   ,
                X_m  ,
                Xd_m ,
                t_r  ,
                tau   );
}

void TwoLayerNeuralNetworkController::UpdateJoint( Eigen::VectorXd & q     ,
					           Eigen::VectorXd & qd    ,
					           Eigen::VectorXd & q_m   ,
                               Eigen::VectorXd & qd_m  ,
					           Eigen::VectorXd & qdd_m ,
					           Eigen::VectorXd & t_r   ,
					           Eigen::VectorXd & tau    )
{
	// NN Input Vector
        x <<           1 ,
            (  q_m -  q) , //   q( 0 ) ;
            ( qd_m - qd) , //  qd( 0 ) ;
                    q_m  ,
                   qd_m  ,
                  qdd_m  ;

        Update( q    ,
                qd   ,
                q_m  ,
                qd_m ,
                t_r  ,
                tau   );
}

void TwoLayerNeuralNetworkController::Update( Eigen::VectorXd & q    ,
                                              Eigen::VectorXd & qd   ,
                                              Eigen::VectorXd & q_m  ,
                                              Eigen::VectorXd & qd_m ,
                                              Eigen::VectorXd & t_r  ,
                                              Eigen::VectorXd & tau   )
{
	W_ = W_next_;
	V_ = V_next_;

	W_trans = W_.transpose();
	V_trans = V_.transpose();

	// Filtered error
	r = (qd_m - qd) + lambda*(q_m - q);								//TODO: This is only for Dynamixel. Have to use this in case of change of motors

//	///////////////////FIX for Dynamixel////////////////////////////////
//	Eigen::VectorXd errorCalc;
//	Eigen::VectorXd errorDCalc;
//
//	errorCalc.resize(num_Joints,1);
//	errorDCalc.resize(num_Joints,1);
//
//	for (int i = 0; i < num_Joints ;  i++)
//	{
//		if (abs(q_m(i) - q(i)) > (M_PI))
//		{
//			errorCalc(i) = (q(i) - q_m(i));
//			errorDCalc(i) = (qd(i) - qd_m(i));
//		}
//		else
//		{
//			errorCalc(i) = (q_m(i) - q(i));
//			errorDCalc(i) = (qd_m(i) - qd(i));
//		}
//	}
//
////	errorCalc = (q_m - q);
////	errorDCalc = (qd_m - qd);
//
//	r = errorDCalc + lambda*errorCalc;
//	////////////////////////////////////////////////

	r_tran = r.transpose();

	// Robust term
	Z.block(0,0,num_Hidden,num_Outputs) = W_;
	Z.block(num_Hidden,num_Outputs,num_Inputs+1,num_Hidden) = V_;
	vRobust = - Kz*(Z.norm() + Zb)*r;

	hiddenLayer_in = V_trans*x;
	hiddenLayer_out = sigmoid(hiddenLayer_in);
	outputLayer_out = W_trans*hiddenLayer_out;

	y = outputLayer_out;

	// control torques
	tau = Kv*r + nn_ON*( y - vRobust ) - feedForwardForce*t_r ;
	//	tau = (qd_m - qd) + 100*(q_m - q);

	if(!updateWeights)
		return;

	//
	sigmaPrime = hiddenLayer_out.asDiagonal()*( hiddenLayerIdentity - hiddenLayerIdentity*hiddenLayer_out.asDiagonal() );

	// Wk+1                  = Wk                  +  Wkdot                                                                                                          * dt
	W_next_ = W_ + (F*hiddenLayer_out*r_tran - F*sigmaPrime*V_trans*x*r_tran - kappa*F*r.norm()*W_) * delT;

	if(!updateInnerWeights)
		return;

	sigmaPrimeTrans_W_r = sigmaPrime.transpose()*W_*r;      // make sigmaPrimeTrans_W_r_tran = r_tran*sigmaPrime*W_trans

	// Vk+1                  = Vk                  +  Vkdot                                                                                      			 * dt
	V_next_ = V_ + (G*x*sigmaPrimeTrans_W_r.transpose() - kappa*G*r.norm()*V_) * delT;

}

Eigen::MatrixXd TwoLayerNeuralNetworkController::sigmoid( Eigen::MatrixXd & z ) const
{
//	// FIXME improve this
	for(uint i=0;i<z.size();i++)
	{
		z(i) = 1.0/(1.0 + exp(-(double)z(i)));
	}
	return z;


//	z = -z;
//	z = z.array().exp();
//	z = Eigen::MatrixXd::Ones(z.size(),1) + z;
//	z = z.cwiseInverse();
	return z;
}




















