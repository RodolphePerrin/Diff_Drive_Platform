/**
 * (C) 2020
 *
 * @author Rodolphe Perrin
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <diff_drive/Encoders.h>
#include <ekf_ros/ekf.h>
#include <diff_drive/node.h>

#include <diff_drive/odometry/state.h>
#include <diff_drive/odometry/encoders.h>

namespace diff_drive
{

namespace odometry
{

/**
 * @brief Retrieve a value vector from the given parameter name at the given node.
 */
inline std::vector<double> getValues(ros::NodeHandle node, const std::string &name)
{
  std::vector<double> values;
  node.getParam(name, values);
  return values;
}


class Odometer
{
  /** @brief Extended Kalman Filter (EKF) used to estimate odometry readings. */
  ekf_ros::EKF ekf_;

  /** @brief Model for the wheel encoders. */
  odometry::Encoders encoders_;

  /** @brief Current odometry state. */
  nav_msgs::Odometry odometry_;

  /** @brief Subscriber to encoders messages. */
  ros::Subscriber encoders_sub_;

  /** @brief Publisher for odometry messages. */
  ros::Publisher publisher_;

  /**
   * @brief Update and publish he current odometry state.
   */
  void callbackEncoders(const diff_drive::EncodersConstPtr &encoders)
  {
    double dt = encoders->dt;

    odometry_.header.stamp = ros::Time::now();

    encoders_.update(encoders->x, encoders->y, encoders->vx, encoders-> vy);
    ekf_(odometry_.header.stamp.toSec(), encoders_);

    auto state = ukf_.getState();
    auto covariance = ukf_.getCovariance();

    double x = state.x;
    double y = state.y;
    double vx = state.vx;
    double vy = state.vy;

    odometry_.pose.pose.position.x = x;
    odometry_.pose.pose.position.y = y;
    
    odometry_.twist.twist.linear.x = vx;
    odometry_.twist.twist.linear.y = vy;

    // Set odometry covariances.
    odometry_.pose.covariance.at(0)  = covariance(0, 0); // x/x
    odometry_.pose.covariance.at(7)  = covariance(1, 1); // y/y
    odometry_.twist.covariance.at(0)  = covariance(2, 2); // vx / vx
    odometry_.twist.covariance.at(7)  = covariance(3, 3); // vy / vy


    publisher_.publish(odometry_);
  }

public:
  /**
   * @brief Create new odometry estimator.
   * @TODO Transform the ekf constructor to add acceleration noise x and y, which will imply addind  a constructor for state with the noise parameters.
   */
  Odometer(ros::NodeHandle node):
    ekf_(),
    encoders_(
     node.param("odometry/variances/x", 0.1),
     node.param("odometry/variances/y", 0.01),
     node.param("odometry/variances/vx", 0.01),
     node.param("odometry/variances/vy", 0.01)
    )
  {
    odometry_.header.frame_id = node.param<std::string>("frame/odometry", "odom");
    odometry_.child_frame_id = node.param<std::string>("frame/robot", "base_link");

    odometry_.pose.covariance.assign(0.0);
    odometry_.twist.covariance.assign(0.0);

    odometry_.pose.covariance.at(0) = node.param("odometry/variances/x", 0.01);
    odometry_.pose.covariance.at(7) = node.param("odometry/variances/y", 0.01);

    encoders_sub_ = node.subscribe("encoders", 1, &Odometer::callbackEncoders, this);
    publisher_ = node.advertise<nav_msgs::Odometry>("odometry", 1);
  }

  /**
   * @brief Return the current odometry state.
   */
  const nav_msgs::Odometry &getOdometry() const
  {
    return odometry_;
  }
};


} // namespace odometry

} // namespace diff_drive

DIFF_DRIVE_NODE("odometry", diff_drive::odometry::Odometer)
