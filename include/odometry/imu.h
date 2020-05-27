/**
 * (C) 2020
 *
 * @author Rodolphe Perrin
 */

#ifndef DIFF_DRIVE_ODOMETRY_IMU_H
#define DIFF_DRIVE_ODOMETRY_IMU_H

#include <odometry/state.h>

namespace diff_drive
{

namespace odometry
{

/**
 * @brief Model for a pair of encoders estimating linear velocity on a differential
 * drive platform.
 */
struct IMU
{
  /**
   * @brief Single encoders reading.
   */
  struct Reading
  {
    /** @brief Reading dimension. */
    static const int dimension = 6;
    
      /** Measurement vector.*/
    Eigen::Matrix<double, dimension, 1> measurement_;

    /** @brief Current horizontal coordinate on the global reference frame. */
    double x_;

    /** @brief Current vertical coordinate on the global reference frame. */
    double y_;

    /** @brief Current linear velocity along x axis. */
    double vx_;
      
    /** @brief Current linear velocity along y axis. */
    double vy_;
      
    /** @brief yaw rotation in radians.. */
    double yaw_;
        
    /** @brief Angular zl speed (turning speed). */
    double wz_;
    
    /** @brief timestamp */
    double timestamp_;
    

    /** @brief Default constructor. */
    Reading();

    /** @brief Create a new reading from given linear velocity. */
    Reading(const State &state);

  };

  /** @brief Covariance matrix type. */
  typedef Eigen::Matrix<double, Reading::dimension, Reading::dimension> Covariance;

  /**
   * @brief Create a new encoders model with given parameters.
   */
  Encoders(double s2_yaw,  double s2_wz);

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
    void update(double dt, double ax, double ay, double yaw, double wz);

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
