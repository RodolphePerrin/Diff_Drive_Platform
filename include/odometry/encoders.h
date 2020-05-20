/**
 * (C) 2020
 *
 * @author Rodolphe Perrin
 */

#ifndef DIFF_DRIVE_ODOMETRY_ENCODERS_H
#define DIFF_DRIVE_ODOMETRY_ENCODERS_H

#include <diff_drive/odometry/state.h>

namespace diff_drive
{

namespace odometry
{

/**
 * @brief Model for a pair of encoders estimating linear velocity on a differential
 * drive platform.
 */
struct Encoders
{
  /**
   * @brief Single encoders reading.
   */
  struct Reading
  {
    /** @brief Reading dimension. */
    static const int dimension = 4;

    /** @brief Current horizontal coordinate on the global reference frame. */
    double x;

    /** @brief Current vertical coordinate on the global reference frame. */
    double y;

    /** @brief Current linear velocity along x axis. */
    double vx;
      
    /** @brief Current linear velocity along y axis. */
    double vy;

    /** @brief Default constructor. */
    Reading();

    /** @brief Create a new reading from given linear velocity. */
    Reading(double v);

  };

  /** @brief Covariance matrix type. */
  typedef Eigen::Matrix<double, Reading::dimension, Reading::dimension> Covariance;

  /**
   * @brief Create a new encoders model with given parameters.
   */
  Encoders(double s2_x, double s2_y, double s2_vx, double s2_vy);

  /**
   * @brief Estimate a set of readings from a set of state predictions.
   */
  std::vector<Reading> estimate(const std::vector<State> predictions);

  /**
   * @brief Estimate a CTRV state from the latest reading.
   */
  State estimate() const;

  /**
   * @brief Retrieve the latest sensor reading.
   */
  const Reading &read() const;

  /**
   * @brief Update the internal reading.
   */
  void update(double x, double y, double vx, double vy);

  /**
   * @brief Return a reference to the noise matrix.
   */
  inline const Covariance &getNoiseMatrix() const
  {
    return noise_;
  }

private:
    
  /** Latest sensor reading. */
  Reading reading_;

  /** @brief Noise matrix. */
  Covariance noise_;

};

} // namespace odometry

} // namespace diff_drive

#endif
