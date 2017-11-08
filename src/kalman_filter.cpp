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
	  * predict the state
	*/
	// From Lesson 5.13
	//std::cout << "Q = " << Q_ << "\n";
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	/**
	  * update the state by using Kalman Filter equations
	*/
	// From Lesson 5.13
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
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	  * update the state by using Extended Kalman Filter equations
	*/
	// Modifications from Lesson 5.20
	// Convert current state back into polar coordinates
	double rho = sqrt((x_[0] * x_[0]) + (x_[1] * x_[1]));
	double phi = 0.0;
	double rhodot = 0.0;

	// Check for 0 or near 0 values to prevent /0
	if (fabs(x_[0]) > 0.0001) {
		phi = atan2(x_[1], x_[0]);
	}
	else {
		phi = atan2(x_[1], 0.0001);
	}
	if (fabs(rho) > 0.0001) {
		rhodot = (((x_[0] * x_[2]) + (x_[1] * x_[3])) / rho);
	}
	else {
		rho = 0.0001;
		rhodot = (((x_[0] * x_[2]) + (x_[1] * x_[3])) / rho);
	}

	// Normalize Phi between -pi/pi
	phi = atan2(sin(phi), cos(phi));
	VectorXd hx(3);
	hx << rho, phi, rhodot;


	//VectorXd z_pred = Hj * x_;
	VectorXd y = z - hx;
	// Normalize y to be between -pi/pi
	y[1] = atan2(sin(y[1]), cos(y[1]));

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
}
