#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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

	cout << "Program started TD!" << endl;

	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;


}

void KalmanFilter::UpdateLaser(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
	From 
  */

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

void KalmanFilter::UpdateRadar(const VectorXd &z) {
	
	// From kartesian to polar
	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

	double rho = sqrt(px*px + py*py);
	double rho_dot = (px*vx + py*vy) / rho;
	double theta = atan2(py , px);

	if (fabs(px) < 0.1) {
		theta = 0;
	}
	
	if (fabs(rho) < 0.1) {
		rho_dot = 0;
	}
	VectorXd h(3);
	h << rho, theta, rho_dot;



	cout << "Radar Update " << h << endl;

	VectorXd y = z - h; //error
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	////new estimate


	cout << "Radar Update H:" << h << endl;
	cout << "K: " << K << endl;
	cout << "x: " << x_ << endl;
	cout << "z: " << z << endl;
	cout << "y: " << y << endl;

	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;



}
