#include "kalman_filter.h"
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

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
  /**
  TODO:
    * predict the state
  */
	// From Lesson 5.13
	std::cout << "Entering KalmanFilter::Predict()\n";
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
	std::cout << "Exiting KalmanFilter::Predict()\n";
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
	// From Lesson 5.13
	std::cout << "Entering KalmanFilter::Update()\n";
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
	std::cout << "Exiting KalmanFilter::Update()\n";
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // Modifications from Lesson 5.20
	std::cout << "Entering KalmanFilter::UpdateEKF()\n";
	Tools tools;
	MatrixXd Hj_ = tools.CalculateJacobian(x_);
	std::cout << "KalmanFilter::UpdateEKF() - Hj Assignment Successful!\n";
	VectorXd z_pred = Hj_ * x_;
	std::cout << "KalmanFilter::UpdateEKF() - z_pred Assignment Successful!\n";
	std::cout << "Z size = " << z.size() << " , Z_Pred size = " << z_pred.size() << std::endl;
	// OUTPUT: Z size = 4 , Z_Pred size = 3
	VectorXd y = z - z_pred;			// Error occurs here
	std::cout << "KalmanFilter::UpdateEKF() - y Assignment Successful!\n";
	MatrixXd Ht = Hj_.transpose();
	std::cout << "KalmanFilter::UpdateEKF() - Ht Assignment Successful!\n";
	MatrixXd S = Hj_ * P_ * Ht + R_;
	std::cout << "KalmanFilter::UpdateEKF() - S Assignment Successful!\n";
	MatrixXd Si = S.inverse();
	std::cout << "KalmanFilter::UpdateEKF() - Si Assignment Successful!\n";
	MatrixXd PHt = P_ * Ht;
	std::cout << "KalmanFilter::UpdateEKF() - PHt Assignment Successful!\n";
	MatrixXd K = PHt * Si;
	std::cout << "KalmanFilter::UpdateEKF() - K Assignment Successful!\n";

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * Hj_) * P_;
	std::cout << "Exiting KalmanFilter::UpdateEKF()\n";
}
