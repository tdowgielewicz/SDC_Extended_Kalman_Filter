#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


#include "tools.h"  

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
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ <<	1, 0, 0, 0,
				0, 1, 0, 0;

  noise_ax = 9;
  noise_ay = 9;





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

	ekf_.P_  = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;



    ekf_.x_ = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

		// Get polar values from sensors
		float rho = measurement_pack.raw_measurements_[0];
		float phi = measurement_pack.raw_measurements_[1];
		float rho_dot = measurement_pack.raw_measurements_[2];

		//Convert to cartesian
		float x = rho * cos(phi);
		float y = rho * sin(phi);
		float vx = rho_dot * cos(phi);
		float vy = rho_dot * sin(phi);
				
		ekf_.x_ << x, y, vx, vy; // Full state knowledge but with high R
    }

    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {

		float x = measurement_pack.raw_measurements_[0];
		float y = measurement_pack.raw_measurements_[1];

		ekf_.x_ << x, y, 0, 0; // Laser gives only position (But with low R)

    }

    // done initializing, no need to predict or update

	is_initialized_ = true;
	cout << "Extended Kalman Filter initialised " << endl;
	cout << "EKF: " << ekf_.x_ <<  endl;
	
	previous_timestamp_ = measurement_pack.timestamp_;
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
	 //Lesson 5 Laser Measurements Part 3
   */
  
  cout << "dt>";

  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ << 1, 0, dt, 0,
	  0, 1, 0, dt,
	  0, 0, 1, 0,
	  0, 0, 0, 1;

  cout << "timestamp" << measurement_pack.timestamp_ << endl;

  float dt2 = dt*dt;
  float dt3 = dt2 * dt;
  float dt4 = dt3 * dt;



  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt4 / 4 * noise_ax, 0, dt3 / 2 * noise_ax, 0,
	  0, dt4 / 4 * noise_ay, 0, dt3 / 2 * noise_ay,
	  dt3 / 2 * noise_ax, 0, dt2*noise_ax, 0,
	  0, dt3 / 2 * noise_ay, 0, dt2*noise_ay;


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
	  ekf_.R_ = R_radar_;
	  //Radar is non linear we need to calculate jackobian
	  ekf_.H_ = tools.CalculateRadarJacobian(measurement_pack.raw_measurements_);
	  ekf_.UpdateRadar(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
	  ekf_.R_ = R_laser_;
	  ekf_.H_ = H_laser_;
	  ekf_.UpdateLaser(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
