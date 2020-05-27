/**
 * (C) 2020
 *
 * @author Rodolphe Perrin
 */

#include <odometry/imu.h>

 /** @brief Boolean to check that the encoders have been initialized */
    bool initialized = false;

namespace diff_drive
{

namespace odometry
{

IMU::Reading::Reading():
  x_(0.0),
  y_(0.0),
  vx_(0.0),
  vy_(0.0),
  yaw_(0.0),
  wz(0.0)
  {
    // Nothing to do.
  }

IMU::Reading::Reading(const State &state):
  x_(state.x_),
  y_(state.y_),
vx_(state.vx_),
vy_(state.vy_)
yaw_(state.yaw_),
wz_(state.wz_)
{
  // Nothing to do.
}

IMU::Encoders(double s2_yaw, double s2_wz)
{
    noise_(0,0) = s2_yaw;
    noise_(1,1) = s2_yaw;
    noise_(2,2) = 0.05;
    noise_(3,3) = 0.05;
    noise_(4,4) = s2_yaw;
    noise(5,5) = s2_wz;
}


State IMU::estimate() const
{
  State state;
  //Initializes all values at 0
  if(!initialized)
  {
  //Initializing at 0 as the first displayed sometimes by the plugin is sometimes erronous. 
  state.x_ = 0.0;
  state.y_ = 0.0;
  state.vx_ = 0.0;
  state.vy_ = 0.0;
  state.yaw_ = 0.0;
  state.wz_ = 0.0;
      state.state_vector_ << state.x_, state.y_, state.vx_, state.vy_,state.yaw_, state.wz_;
  initialized = true;
  return state;
  }
  
  state.x_ = reading_.x_;
  state.y_ = reading_.y_;
  state.vx_ = reading_.vx_;
  state.vy_ = reading_.vy_;
  state.yaw_ = reading_.yaw_;
  state.wz_ = reading_.wz_;
  state.state_vector_ << state.x_, state.y_, state.vx_, state.vy_, state.yaw_, state.wz_;

  return state;
}



const IMU::Reading &Encoders::read() const
{
  return reading_;
}

void IMU::update(double t, double ax, double ay, double yaw, double wz)
{
  reading_.timestamp_ = t -  reading_.timestamp_;
  reading_.x_ = reading_.x_ +reading_.vx*reading_.timestamp_ + 0.5*ax*reading_.timestamp_*reading_.timestamp_ ;
  reading_.y_ = reading_.y_ +reading_.vy*reading_.timestamp_ + 0.5*ay*reading_.timestamp_*reading_.timestamp_ ;
  reading_.vx_ = reading_.vx_ + ax*t;
  reading_.vy_ = reading_.vy_ + ay*t;
  reading_.yaw_ = reading_.yaw_;
  reading_.wz_ = reading_.wz_;
    reading_.measurement_ <<reading_.x_,reading_.y_,reading_.vx_,reading_.vy_,reading_.yaw_,reading_.wz_;
}


} // namespace odometry

} // namespace diff_drive
