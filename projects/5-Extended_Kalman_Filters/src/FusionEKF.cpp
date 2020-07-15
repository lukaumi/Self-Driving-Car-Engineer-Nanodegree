#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

/**
 * Constructor.
 */
FusionEKF::FusionEKF()
{
  is_initialized_ = false;

  previous_timestamp_ = 0;

  //measurement covariance matrix - laser
  R_laser_ = Eigen::MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
      0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ = Eigen::MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;

  // measurement matrix - laser
  H_laser_ = Eigen::MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
      0, 1, 0, 0;

  // create Jacobian matrix
  Hj_ = Eigen::MatrixXd(3, 4);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
  /*
   ******************************
   * Initialization
   ******************************
   */

  if (!is_initialized_)
  {

    // initialize a 4D state vector ekf_.x_ with the first measurement
    ekf_.x_ = Eigen::VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
      // convert radar from polar to cartesian coordinates and initialize state
      double rho = measurement_pack.raw_measurements_(0);
      double theta = measurement_pack.raw_measurements_(1);
      double rho_dot = measurement_pack.raw_measurements_(2);

      ekf_.x_(0) = rho * cos(theta);
      ekf_.x_(1) = rho * sin(theta);
      ekf_.x_(2) = rho_dot * cos(theta);
      ekf_.x_(3) = rho_dot * sin(theta);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      // initialize state
      ekf_.x_(0) = measurement_pack.raw_measurements_[0];
      ekf_.x_(1) = measurement_pack.raw_measurements_[1];
      ekf_.x_(2) = 0;
      ekf_.x_(3) = 0;
    }

    // initialize state covariance matrix P
    ekf_.P_ = Eigen::MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

    // initialize state transition matrix F
    ekf_.F_ = Eigen::MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // create process covariance matrix Q
    ekf_.Q_ = Eigen::MatrixXd::Zero(4, 4);

    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*
   ******************************
   * Prediction
   ******************************
   */

  // compute the time elapsed between the current and previous measurements
  // time is measured in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // update the state transition matrix F according to the new elapsed time
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // update the process noise covariance matrix Q
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  float noise_ax = 9;
  float noise_ay = 9;

  ekf_.Q_(0, 1) = dt_4 / 4 * noise_ax;
  ekf_.Q_(0, 2) = dt_3 / 2 * noise_ax;
  ekf_.Q_(1, 1) = dt_4 / 4 * noise_ay;
  ekf_.Q_(1, 3) = dt_3 / 2 * noise_ay;
  ekf_.Q_(2, 0) = dt_3 / 2 * noise_ax;
  ekf_.Q_(2, 2) = dt_2 * noise_ax;
  ekf_.Q_(3, 1) = dt_3 / 2 * noise_ay;
  ekf_.Q_(3, 3) = dt_2 * noise_ay;

  // predict
  ekf_.Predict();

  /*
   ******************************
   * Update
   ******************************
   */

  // update the state and covariance matrices
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
  {
    // radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }
  else
  {
    // laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  std::cout << "x_ = " << ekf_.x_ << std::endl;
  std::cout << "P_ = " << ekf_.P_ << std::endl;
}
