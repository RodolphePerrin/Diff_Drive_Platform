/**
 * (C) 2020
 *
 * @author Rodolphe Perrin
 */

#include <machina/differential/odometry/encoders.h>

namespace diff_drive
{

namespace odometry
{

Encoders::Reading::Reading():
  x(0.0),
  y(0.0),
  vx(0.0),
  vy(0.0)
  {
    // Nothing to do.
  }

Encoders::Reading::Reading(const State &state):
  x(state.x),
  y(state.y),
vx(state.vx),
vy(state.vy)
{
  // Nothing to do.
}

Encoders::Encoders(double s2_x, double s2_y, double s2_vx, double s2_vy)
{
    noise_(0,0) = s2_x;
    noise_(1,1) = s2_y;
    noise_(2,2) = s2_vx;
    noise_(3,3) = s2_vy;


State Encoders::estimate() const
{
  State state;

  state.x = reading_.x;
  state.y = reading_.y;
  state.vx = reading_.vx;
  state.vy = reading_.vy;

  return state;
}



const Encoders::Reading &Encoders::read() const
{
  return reading_;
}

void Encoders::update(double x, double y, double vx, double vy)
{
  reading_.x = x;
  reading_.y = y;
  reading_.vx = vx;
  reading_.vy = vy;
    
}


} // namespace odometry

} // namespace diff_drive
