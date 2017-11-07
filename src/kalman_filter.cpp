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
	std::cout << "KalmanFilter::Update() - z_pred Assignment Successful!\n";
	std::cout << "z size = " << z.size() << " , z_pred size = " << z_pred.size() << std::endl;
	// OUTPUT: z size = 4 , z_pred size = 2
	VectorXd y = z - z_pred;				// Error here
	std::cout << "KalmanFilter::Update() - y Assignment Successful!\n";
	MatrixXd Ht = H_.transpose();
	std::cout << "KalmanFilter::Update() - Ht Assignment Successful!\n";
	MatrixXd S = H_ * P_ * Ht + R_;
	std::cout << "KalmanFilter::Update() - S Assignment Successful!\n";
	MatrixXd Si = S.inverse();
	std::cout << "KalmanFilter::Update() - Si Assignment Successful!\n";
	MatrixXd PHt = P_ * Ht;
	std::cout << "KalmanFilter::Update() - PHt Assignment Successful!\n";
	MatrixXd K = PHt * Si;
	std::cout << "KalmanFilter::Update() - K Assignment Successful!\n";

	//new estimate
	x_ = x_ + (K * y);
	std::cout << "KalmanFilter::Update() - X Assignment Successful!\n";
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	std::cout << "KalmanFilter::Update() - I Assignment Successful!\n";
	P_ = (I - K * H_) * P_;
	std::cout << "KalmanFilter::Update() - P Assignment Successful!\n";
	std::cout << "Exiting KalmanFilter::Update()\n";
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // Modifications from Lesson 5.20
	std::cout << "Entering KalmanFilter::UpdateEKF()\n";

	// Convert current state back into polar coordinates
	float rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
	float phi = 0.0;
	float rhodot = 0.0;
	
	if (fabs(x_(0)) > 0.0001) {
		phi = atan2(x_(1), x_(0));
	}
	if (fabs(rho) > 0.0001) {
		rhodot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
	}
	VectorXd hx(3);
	hx << rho, phi, rhodot;

	Tools tools;
	MatrixXd Hj_ = tools.CalculateJacobian(z);
	std::cout << "KalmanFilter::UpdateEKF() - Hj Assignment Successful!\n";
	VectorXd z_pred = Hj_ * x_;
	std::cout << "KalmanFilter::UpdateEKF() - z_pred Assignment Successful!\n";
	std::cout << "z_pred size = " << z_pred.size() << " , hx size = " << hx.size() << std::endl;
	// OUTPUT: z size = 4 , hx size = 3
	VectorXd y = z_pred - hx;
	// Renormalize y pi/-pi
	std::cout << "KalmanFilter::UpdateEKF() - y Assignment Successful!\n";
	MatrixXd Ht = H_.transpose();
	std::cout << "KalmanFilter::UpdateEKF() - Ht Assignment Successful!\n";
	std::cout << "H size = " << H_.size() << " , P size = " << P_.size();
	std::cout << " , Ht size = " << Ht.size() << " , R size = " << R_.size() << ::endl;
	// OUTPUT: H size = 8 , P size = 16 , Ht size = 8 , R size = 4
	MatrixXd S = H_ * P_ * Ht + R_;	
	std::cout << "KalmanFilter::UpdateEKF() - S Assignment Successful!\n";
	MatrixXd Si = S.inverse();
	std::cout << "KalmanFilter::UpdateEKF() - Si Assignment Successful!\n";
	MatrixXd PHt = P_ * Ht;
	std::cout << "KalmanFilter::UpdateEKF() - PHt Assignment Successful!\n";
	MatrixXd K = PHt * Si;
	std::cout << "KalmanFilter::UpdateEKF() - K Assignment Successful!\n";

	//new estimate
	std::cout << "Hx size = " << hx.size() << " , K size = " << K.size();
	std::cout << " , Y size = " << y.size() << ::endl;
	// OUTPUT: Hx size = 3, K size = 8, Y size = 3
	x_ = hx + (K * y);				// Error occurs here
	std::cout << "KalmanFilter::UpdateEKF() - X Assignment Successful!\n";
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	std::cout << "KalmanFilter::UpdateEKF() - I Assignment Successful!\n";
	P_ = (I - K * Hj_) * P_;
	std::cout << "KalmanFilter::UpdateEKF() - P Assignment Successful!\n";
	std::cout << "Exiting KalmanFilter::UpdateEKF()\n";
}
