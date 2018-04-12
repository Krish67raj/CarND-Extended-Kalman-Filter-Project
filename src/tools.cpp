#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
	
  */
	VectorXd rmse(4);
	rmse << 0,0,0,0;

  	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	
	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd dif = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		dif = dif.array()*dif.array();
		rmse += dif;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
	
	
  */
  
  MatrixXd Hj(3,4);
	//recover state parameters
	
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);
	
    double px2 = pow(px,2);
    double py2 = pow(py,2);
    double sum = px2 + py2;
	
	//TODO: YOUR CODE HERE 

	//check division by zero
	
	//compute the Jacobian matrix
	
	Hj <<  px/sqrt(sum), py/sqrt(sum), 0, 0,
	        -py/sum, px/sum,0,0,
	        py*(vx*py-vy*px)/pow(sum,1.5), px*(vy*px-vx*py)/pow(sum,1.5),px/sqrt(sum),py/sqrt(sum);

	return Hj;
}
