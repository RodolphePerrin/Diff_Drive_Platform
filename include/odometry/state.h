/**
 * (C)  2020
 *
 * @author Rodolphe Perrin
 */

#ifndef DIFF_DRIVE_ODOMETRY_STATE_H
#define DIFF_DRIVE_ODOMETRY_STATE_H


#include <eigen3/Eigen/Core>
#include <vector>
#include <iostream>

namespace diff_drive
{

namespace odometry
{

/**
 * @brief State Struct for the differential drive vehicle.
 */
struct State
{
  /** @brief Dimension of process state. */
  static const int dimension = 4;
  /** @brief State covariance matrix type. */
  typedef Eigen::Matrix<double, dimension, dimension> Covariance;
    
  /** @brief State transition matrix type. */
  typedef Eigen::Matrix<double, dimension, dimension> TransitionMatrix;
    
  /** @brief uncertainties matrix type. */
  typedef Eigen::Matrix<double, dimension, dimension> Uncertainties;
    
  /** State vector.*/
  Eigen::Matrix<double, dimension, 1> state_vector_;

  /** @brief Position x. */
  double x_;

  /** @brief Position y */
  double y_;

  /** @brief Linear horiztonal speed. */
  double vx_;

  /** @brief Linear vertical speed. */
  double vy_;
    
  /** @brief acceleration noise x. */
  double acc_noise_x = 9.;

  /** @brief acceleration noise y. */
  double acc_noise_y = 9.;
    
  /** @brief State covariance */
  Covariance P_;

  /** @brief Transition Matrix. */
  TransitionMatrix F_;
    
  /** @brief Uncertainties Matrix. */
  Uncertainties Q_;

  /**
   * @brief Default constructor.
   */
  State();

  /**
   * @brief Compute an initial mean state based on initial values for position and speed.
   */
  State(double x_0, double y_0, double vx_0, double vy_0);
};

} // namespace odometry

} // namespace diff_drive


#endif
