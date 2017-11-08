#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_matrix = MatrixXd(3, 4);

  // Initialize FusionEKF members
  P_matrix = MatrixXd(4, 4);
  F_matrix = MatrixXd(4, 4);

  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  // Measurement Matrix - Laser
  H_laser_ << 1, 0, 0, 0,		
	  0, 1, 0, 0;

  // Jacobian Measurement Matrix - Radar
  Hj_matrix << 0, 0, 0, 0,
	  0, 0, 0, 0,
	  0, 0, 0, 0;
  		
  // State Covariance Matrix
  P_matrix << 1, 0, 0, 0,
	  0, 1, 0, 0,
	  0, 0, 1000, 0,
	  0, 0, 0, 1000;

  // Initial Transition Matrix
  F_matrix << 1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
	  //previous_timestamp_ = 0;
	  ekf_.x_ = VectorXd(4);
	  ekf_.x_ << 0, 0, 0, 0;
	  ekf_.H_ = H_laser_;
	  ekf_.R_ = R_laser_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
		/*float rho = measurement_pack.raw_measurements_[0];
		float phi = measurement_pack.raw_measurements_[1];
		float rhodot = measurement_pack.raw_measurements_[2];

		// Normalize Phi to be between -pi, pi
		phi = atan2(sin(phi), cos(phi));

		// Convert polar to cartesian here
		float px = rho * cos(phi);
		float py = rho * sin(phi);
		float vx = rhodot * cos(phi);
		float vy = rhodot * sin(phi);
		ekf_.x_ << px, py, vx, vy;

		ekf_.H_ = tools.CalculateJacobian(ekf_.x_);

		ekf_.UpdateEKF(measurement_pack.raw_measurements_);*/
		ekf_.x_ << measurement_pack.raw_measurements_[1], measurement_pack.raw_measurements_[1], 0, 0;
		//ekf_.R_ = R_radar_;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
		/*VectorXd measurements(2);
		measurements << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1];
		
		ekf_.x_ << measurements(0), measurements(1), 0, 0;
		ekf_.H_ = H_laser_;
		ekf_.R_ = R_laser_;

		ekf_.Update(measurements);
		cout << "EKF Laser X: " << ekf_.x_ << endl;*/
		ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
		//ekf_.R_ = R_laser_;
		//ekf_.H_ = H_laser_;
    }
	previous_timestamp_ = measurement_pack.timestamp_;

	ekf_.F_ = F_matrix;
	ekf_.P_ = P_matrix;

    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
   //compute the time elapsed between the current and previous measurements
   //dt - expressed in seconds
	double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	previous_timestamp_ = measurement_pack.timestamp_;

	// time delta powers
	double dt_2 = dt * dt;
	double dt_3 = dt_2 * dt;
	double dt_4 = dt_3 * dt;

	//set the acceleration noise components
	float noise_ax = 9.0;
	float noise_ay = 9.0;

	// Incorporate time into F Matrix
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

	// Set the process covariance matrix Q
	// Don't ask me why but setting Q with in-line 
	// equations forces Q to always remain all 0s
	ekf_.Q_ = MatrixXd(4, 4);
	double Q1 = (dt_4 / 4 * noise_ax);
	double Q3 = (dt_3 / 2 * noise_ax);
	double Q6 = (dt_4 / 4 * noise_ay);
	double Q8 = (dt_3 / 2 * noise_ay);
	double Q9 = (dt_3 / 2 * noise_ax);
	double Q11 = (dt_2 * noise_ax);
	double Q14 = (dt_3 / 2 * noise_ay);
	double Q16 = (dt_2 * noise_ay);
	ekf_.Q_ << Q1, 0, Q3, 0, 0, Q6, 0, Q8, Q9, 0, Q11, 0, 0, Q14, 0, Q16;

	std::cout << "EKF_.Q_ = " << ekf_.Q_ << "\n";

	ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, ekf_.H_, ekf_.R_, ekf_.Q_);
	ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	  Hj_matrix = tools.CalculateJacobian(ekf_.x_);
	  ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, Hj_matrix, R_radar_, ekf_.Q_);
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
	  ekf_.Init(ekf_.x_, ekf_.P_, ekf_.F_, H_laser_, R_laser_, ekf_.Q_);
	  ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  std::cout << "x_ = " << ekf_.x_ << endl;
  std::cout << "P_ = " << ekf_.P_ << endl;
}
