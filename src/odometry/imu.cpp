/**
 * (C) 2020
 *
 * @author Rodolphe Perrin
 */

#include <odometry/imu.h>

 /** @brief Boolean to check that the encoders have been initialized */
    bool imu_initialized = false;
    bool timestamp_initialized = false;

namespace diff_drive
{

namespace odometry
{

Imu::Reading::Reading():
  x_(0.0),
  y_(0.0),
  vx_(0.0),
  vy_(0.0),
  yaw_(0.0),
  wz_(0.0)
  {
    // Nothing to do.
  }

Imu::Reading::Reading(const State &state):
  x_(state.x_),
  y_(state.y_),
vx_(state.vx_),
vy_(state.vy_),
yaw_(state.yaw_),
wz_(state.wz_)
{
  // Nothing to do.
}

Imu::Imu(double s2_ax, double s2_ay, double s2_yaw, double s2_wz)
{
    noise_(0,0) = s2_ax;
    noise_(1,1) = s2_ay;
    noise_(2,2) = s2_ax;
    noise_(3,3) = s2_ay;
    noise_(4,4) = s2_yaw;
    noise_(5,5) = s2_wz;
}


State Imu::estimate() const
{
  State state;
  //Initializes all values at 0
  if(!imu_initialized)
  {
  //Initializing at 0 as the first displayed sometimes by the plugin is sometimes erronous. 
  state.x_ = 0.0;
  state.y_ = 0.0;
  state.vx_ = 0.0;
  state.vy_ = 0.0;
  state.yaw_ = 0.0;
  state.wz_ = 0.0;
      state.state_vector_ << state.x_, state.y_, state.vx_, state.vy_,state.yaw_, state.wz_;
  imu_initialized = true;
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



const Imu::Reading &Imu::read() const
{
  return reading_;
}

void Imu::update(double t, double ax, double ay, double yaw, double wz)
{
  if (!timestamp_initialized)
  {
      reading_.timestamp_ = t;
      timestamp_initialized = true;
      reading_.x_ = 0 ;
      reading_.y_ = 0 ;
      reading_.vx_ = 0;
      reading_.vy_ = 0;
      reading_.yaw_ = 0;
      reading_.wz_ = 0;
      reading_.measurement_ <<reading_.x_,reading_.y_,reading_.vx_,reading_.vy_,reading_.yaw_,reading_.wz_;
      return;
      
  }
  double dt = t - reading_.timestamp_;
  reading_.timestamp_ = t;
  reading_.x_ = 0.0;//reading_.x_ +reading_.vx_*dt + 0.5*ax*dt*dt;
  reading_.y_ = 0.0; //reading_.y_ +reading_.vy_*dt + 0.5*ay*dt*dt;
  reading_.vx_ = 0.0; //reading_.vx_ + ax*dt;
  reading_.vy_ = 0.0; //reading_.vy_ + ay*dt;
  reading_.yaw_ = yaw;
  reading_.wz_ = wz;
  reading_.measurement_ <<reading_.x_,reading_.y_,reading_.vx_,reading_.vy_,reading_.yaw_,reading_.wz_;
    
}


} // namespace odometry

} // namespace diff_drive
