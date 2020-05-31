/**
 * (C) 2020
 *
 * @author Rodolphe Perrin
 */

#include <odometry/encoders.h>

 /** @brief Boolean to check that the encoders have been initialized */
    bool initialized = false;

namespace diff_drive
{

namespace odometry
{

Encoders::Reading::Reading():
  x_(0.0),
  y_(0.0),
  vx_(0.0),
  vy_(0.0),
  yaw_(0.0),
  wz_(0.0)
  {
    // Nothing to do.
  }
  
Encoders::Reading::Reading(const State &state):
  x_(state.x_),
  y_(state.y_),
vx_(state.vx_),
vy_(state.vy_),
yaw_(state.yaw_),
wz_(state.wz_)
{
  // Nothing to do.
}

Encoders::Encoders(double s2_x, double s2_y, double s2_vx, double s2_vy)
{
    noise_(0,0) = s2_x;
    noise_(1,1) = s2_y;
    noise_(2,2) = s2_vx;
    noise_(3,3) = s2_vy;
    noise_(4,4) = s2_x;
    noise_(5,5) = s2_vx;
}


State Encoders::estimate() const
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
  state.state_vector_ << state.x_, state.y_, state.vx_, state.vy_, state.yaw_, state.wz_;
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



const Encoders::Reading &Encoders::read() const
{
  return reading_;
}

void Encoders::update(double x, double y, double vx, double vy, double yaw, double wz)
{
  //Disregarding wz value here as not given by encoders.
  reading_.x_ = x;
  reading_.y_ = y;
  reading_.vx_ = vx;
  reading_.vy_ = vy;
  reading_.yaw_ = yaw;
  reading_.wz_ = wz;
  reading_.measurement_ << x,y,vx,vy, yaw, wz;
    
}


} // namespace odometry

} // namespace diff_drive
