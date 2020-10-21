#include "tools.h"
#include <iostream>

Tools::Tools() {}

Tools::~Tools() {}

Eigen::VectorXd Tools::CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations,
                                     const std::vector<Eigen::VectorXd> &ground_truth)
{
   Eigen::VectorXd rmse(4);
   rmse << 0, 0, 0, 0;

   // check the validity of the following inputs:
   //  * the estimation vector size should not be zero
   //  * the estimation vector size should equal ground truth vector size
   if (estimations.size() == 0 || estimations.size() != ground_truth.size())
   {
      std::cout << "Invalid estimation or ground_truth data" << std::endl;
      return rmse;
   }

   // accumulate squared residuals
   for (int i = 0; i < estimations.size(); ++i)
   {
      Eigen::VectorXd residual = estimations[i] - ground_truth[i];

      // coefficient-wise multiplication
      residual = residual.array() * residual.array();
      rmse += residual;
   }

   // calculate the mean
   rmse = rmse / estimations.size();

   // calculate the squared root
   rmse = rmse.array().sqrt();

   return rmse;
}

Eigen::MatrixXd Tools::CalculateJacobian(const Eigen::VectorXd &x_state)
{
   Eigen::MatrixXd Hj(3, 4);

   // recover state parameters
   double px = x_state(0);
   double py = x_state(1);
   double vx = x_state(2);
   double vy = x_state(3);

   // pre-compute a set of terms to avoid repeated calculation
   double c1 = px * px + py * py;
   double c2 = sqrt(c1);
   double c3 = (c1 * c2);

   // check division by zero
   if (fabs(c1) < 0.0001)
   {
      std::cout << "CalculateJacobian() Error: Division by Zero" << std::endl;
      return Hj;
   }

   // compute the Jacobian matrix
   Hj << (px / c2), (py / c2), 0, 0,
       -(py / c1), (px / c1), 0, 0,
       py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

   return Hj;
}
