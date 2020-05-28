/**
 * (C) 2020
 *
 * @author Rodolphe Perrin
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <diff_drive/Encoders.h>
#include <sensor_msgs/Imu.h>
#include <ekf.h>
#include <node.h>

#include <odometry/state.h>
#include <odometry/encoders.h>
#include <odometry/imu.h>

#include <tf/LinearMath/Quaternion.h>


//writing into files for debugging purposes
#include <fstream>

using namespace std; 

ofstream odometry_file;

namespace diff_drive
{

namespace odometry
{
/** @brief Get yaw angle from quaternion.*/
double getYaw(const geometry_msgs::Quaternion &orientation)
{
  tf::Quaternion q(
    orientation.x,
    orientation.y,
    orientation.z,
    orientation.w
  );

  return tf::getYaw(q);
}

class Odometer
{
  /** @brief Extended Kalman Filter (EKF) used to estimate odometry readings. */
  diff_drive::odometry::EKF ekf_;

  /** @brief Model for the wheel encoders. */
  odometry::Encoders encoders_;
    
  /** @brief Model for the wheel encoders. */
  odometry::Imu imu_;

  /** @brief Current odometry state. */
  nav_msgs::Odometry odometry_;

  /** @brief Subscriber to encoders messages. */
  ros::Subscriber encoders_sub_;
    
  /** @brief Subscriber to imu messages. */
  ros::Subscriber imu_sub_;

  /** @brief Publisher for odometry messages. */
  ros::Publisher publisher_;

  /**
   * @brief Update and publish the current odometry state.
   */
  void callbackEncoders(const diff_drive::EncodersConstPtr &encoders)
  {
    double dt = encoders->header.stamp.toSec();

    odometry_.header.stamp = ros::Time::now();

      encoders_.update(encoders->pose.pose.position.x, encoders->pose.pose.position.y, encoders->twist.twist.linear.x, encoders-> twist.twist.linear.y, getYaw(encoders->pose.pose.orientation));
    ekf_(odometry_.header.stamp.toSec(), encoders_);

    auto state = ekf_.getState();
    auto covariance = ekf_.getCovariance();
    double x = state.x_;
    double y = state.y_;
    double vx = state.vx_;
    double vy = state.vy_;
    double yaw = state.yaw_;
    double wz = state.wz_;
      
    geometry_msgs::Quaternion orientation_msg;
    tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(q, orientation_msg);

      
    odometry_.pose.pose.position.x = x;
    odometry_.pose.pose.position.y = y;
    odometry_.pose.pose.orientation = orientation_msg;
    
    odometry_.twist.twist.linear.x = vx;
    odometry_.twist.twist.linear.y = vy;
    odometry_.twist.twist.angular.z = wz;

    // Set odometry covariances.
    odometry_.pose.covariance.at(0)  = covariance(0, 0); // x/x
    odometry_.pose.covariance.at(7)  = covariance(1, 1); // y/y
    odometry_.twist.covariance.at(0)  = covariance(2, 2); // vx / vx
    odometry_.twist.covariance.at(7)  = covariance(3, 3); // vy / vy
    
    odometry_file.open("/home/user/personal_ws/src/Diff_Drive_Platform/ressources/odometry_filtered.txt", std::ios::app);
    odometry_file <<odometry_.header.stamp<<" "<<odometry_.pose.pose.position.x<<" "<<odometry_.pose.pose.position.y<<" "<<odometry_.twist.twist.linear.x<<" "<<odometry_.twist.twist.linear.y<<endl;
    odometry_file.close();


    publisher_.publish(odometry_);
  }
    
  /**
   * @brief Update the state with Imu measurements..
   */
  void callbackIMU(const sensor_msgs::ImuConstPtr &imu)
  {
    double dt = imu->header.stamp.toSec();
    //Extract yaw from quaternion
      double yaw = getYaw(imu->orientation);

      imu_.update(dt,imu->linear_acceleration.x, imu->linear_acceleration.y , yaw, imu->angular_velocity.z);
      ekf_(dt, imu_);
      
      auto state = ekf_.getState();
      auto covariance = ekf_.getCovariance();
      double x = state.x_;
      double y = state.y_;
      double vx = state.vx_;
      double vy = state.vy_;
      double yaw = state.yaw_;
      double wz = state.wz_;
      
      geometry_msgs::Quaternion orientation_msg;
      tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, yaw);
      tf::quaternionTFToMsg(q, orientation_msg);

      odometry_.pose.pose.position.x = x;
      odometry_.pose.pose.position.y = y;
      odometry_.pose.pose.orientation = orientation_msg;
      
      odometry_.twist.twist.linear.x = vx;
      odometry_.twist.twist.linear.y = vy;
      odometry_.twist.twist.angular.z = wz;

      // Set odometry covariances.
      odometry_.pose.covariance.at(0)  = covariance(0, 0); // x/x
      odometry_.pose.covariance.at(7)  = covariance(1, 1); // y/y
      odometry_.twist.covariance.at(0)  = covariance(2, 2); // vx / vx
      odometry_.twist.covariance.at(7)  = covariance(3, 3); // vy / vy
      
      odometry_file.open("/home/user/personal_ws/src/Diff_Drive_Platform/ressources/odometry_filtered.txt", std::ios::app);
      odometry_file <<odometry_.header.stamp<<" "<<odometry_.pose.pose.position.x<<" "<<odometry_.pose.pose.position.y<<" "<<odometry_.twist.twist.linear.x<<" "<<odometry_.twist.twist.linear.y<<endl;
      odometry_file.close();


      publisher_.publish(odometry_);
  }

public:
  /**
   * @brief Create new odometry estimator.
   * @TODO Transform the ekf constructor to add acceleration noise x and y, which will imply addind  a constructor for state with the noise parameters.
   */
  Odometer(ros::NodeHandle node):
    encoders_(
     node.param("encoders/variances/x", 100000),
     node.param("encoders/variances/y", 0.1),
     node.param("encoders/variances/vx", 0.1),
     node.param("encoders/variances/vy", 0.1)
    ),
    imu_(
     node.param("imu/variances/yaw", 0.1),
     node.param("imu/variances/angular_velocity", 0.1),
    )
  {
    odometry_.header.frame_id = node.param<std::string>("frame/odometry", "odom");
    odometry_.child_frame_id = node.param<std::string>("frame/robot", "base_link");

    odometry_.pose.covariance.assign(0.0);
    odometry_.twist.covariance.assign(0.0);

    odometry_.pose.covariance.at(0) = node.param("odometry/variances/x", 0.1);
    odometry_.pose.covariance.at(7) = node.param("odometry/variances/y", 0.1);

    //encoders_sub_ = node.subscribe("encoders", 1, &Odometer::callbackEncoders, this);
    imu_sub_ = node.subscribe("imu/data", 1, &Odometer::callbackIMU, this);
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
