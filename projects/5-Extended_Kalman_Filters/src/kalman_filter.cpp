#include "kalman_filter.h"

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in,
                        Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in)
{
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict()
{
  x_ = F_ * x_;

  Eigen::MatrixXd F_t = F_.transpose();
  P_ = F_ * P_ * F_t + Q_;
}

void KalmanFilter::Update(const Eigen::VectorXd &z)
{
  // update the state by using Kalman Filter equations

  // error
  Eigen::MatrixXd y = z - (H_ * x_);

  Eigen::MatrixXd H_t = H_.transpose();
  // matrix which is obtained by projecting the system uncertainty into the measurement space using the measurement function projection
  Eigen::MatrixXd S = H_ * P_ * H_t + R_;

  Eigen::MatrixXd Si = S.inverse();
  // Kalman gain
  Eigen::MatrixXd K = P_ * H_t * Si;

  const auto x_size = x_.size();
  // identity matrix
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);

  // new state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z)
{
  // update the state by using Extended Kalman Filter equations

  // map the predicted location x_ from Cartesian coordinates to polar coordinates
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);
  double rho = sqrt(px * px + py * py);
  double theta = atan2(py, px);
  double rho_dot = (px * vx + py * vy) / rho;
  Eigen::VectorXd hx(3);
  hx << rho, theta, rho_dot;

  // error
  Eigen::VectorXd y = z - hx;

  // normalize angles between -pi and pi
  while (y(1) > M_PI)
  {
    y(1) -= 2 * M_PI;
  }
  while (y(1) < -M_PI)
  {
    y(1) += 2 * M_PI;
  }

  Eigen::MatrixXd H_t = H_.transpose();
  // matrix which is obtained by projecting the system uncertainty into the measurement space using the measurement function projection
  Eigen::MatrixXd S = H_ * P_ * H_t + R_;

  Eigen::MatrixXd Si = S.inverse();
  // Kalman gain
  Eigen::MatrixXd K = P_ * H_t * Si;

  const auto x_size = x_.size();
  // identity matrix
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_size, x_size);

  // new state
  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}
