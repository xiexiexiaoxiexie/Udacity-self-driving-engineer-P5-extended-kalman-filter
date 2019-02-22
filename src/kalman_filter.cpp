#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_=F_*x_;
  MatrixXd Ft = F_.transpose();
  P_=F_*P_*Ft+Q_;
  /**
   * TODO: predict the state
   */
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y=z-H_*x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt=P_ * Ht;
  MatrixXd S = H_ *PHt + R_;
  MatrixXd K = PHt * (S.inverse());
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  VectorXd hx(3);
  hx(0)=sqrt(x_(0)*x_(0)+x_(1)*x_(1));
  if(fabs(x_(0))<0.0001||fabs(x_(1))<0.0001){
    hx(1)=0;
    //hx(1)=atan2(x_(1),x_(0));
    hx(2)=0;
    }
  else{
    hx(1)=atan2(x_(1),x_(0));
    hx(2)=(x_(0)*x_(2)+x_(1)*x_(3))/hx(0);
  }
  VectorXd y=z-hx;
  //while(hx(1)-z(1)>M_PI/2)
    //hx(1)=hx(1)-M_PI;
  //while(z(1)-hx(1)>M_PI/2)
    //hx(1)=hx(1)+M_PI;
  //y(1)=atan2(tan(y(1)),1);
  while(y(1)>M_PI)
    y(1)=y(1)-2*M_PI;
  while(y(1)<-M_PI)
    y(1)=y(1)+2*M_PI;
  MatrixXd S = H_ * P_ * (H_.transpose())+ R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + (K * y);
  int x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
