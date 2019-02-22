#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0){
      std::cout << "ERROR" << std::endl;
      return rmse;
}
  for(int i=0; i < estimations.size(); ++i){
    VectorXd error = estimations[i] - ground_truth[i];
    // Coefficient-wise multiplication
    error = error.array()*error.array();
    rmse += error;
  }

  // Calculate the mean
  rmse = rmse / estimations.size();
  rmse = rmse.array().sqrt();
  //averpy+=rmse(1);
return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  double px=x_state(0);
  double py=x_state(1);
  double vx=x_state(2);
  double vy=x_state(3);
  
  double sum2=px*px+py*py;
  double sum12=sqrt(sum2);
  //////double sum32=sum2*sum12;
  long double sum32=pow((long double)sum2,(long double)1.50);
  if(fabs(sum2) < 0.0000001){
	std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
    return Hj;
      }
  
  Hj<<px/sum12,py/sum12,0,0,
    -1*py/sum2,px/sum2,0,0,
     py*(x_state(2)*py-x_state(3)*px)/sum32,px*(x_state(3)*px-x_state(2)*py)/sum32,px/sum12,py/sum12;
  return Hj;
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
}
