/**
 * (C) 2019 Cyberworks Robotics, inc.
 *
 * @author Helio Perroni Filho
 */

#include <machina/differential/odometry/state.h>

namespace machina
{

namespace differential
{

namespace odometry
{

State::State():
  yaw(1.0, 0.0),
  v(0.0),
  w(0.0)
{
  // Nothing to do.
}

State::State(double weight_0, double weight_j, const std::vector<State> &samples)
{
  const auto &first = samples[0];

  yaw = weight_0 * first.yaw;
  v = weight_0 * first.v;
  w = weight_0 * first.w;

  for (int j = 1, n = samples.size(); j < n; ++j)
  {
    const auto &sample = samples[j];
    yaw += weight_j * sample.yaw;
    v += weight_j * sample.v;
    w += weight_j * sample.w;
  }

  // In theory this would be unnecessary, as the magnitude of the added numbers
  // should add to 1; in practice however, resolution limitations in the double
  // type cause the actual magnitude to be other than 1, which if left unchecked
  // would cause compounded computation errors later on.
  yaw = botn::normalizeAngle(yaw);
}

State &State::operator += (const Difference &difference)
{
  yaw *= botn::angleToComplex(difference(0));
  v += difference(1);
  w += difference(2);

  // It's debatable whether normalization is required here (no sizeable errors
  // were ever detected in practice), but better safe than sorry.
  yaw = botn::normalizeAngle(yaw);
}

State::Difference State::operator - (const State &that) const
{
  Difference d;

  d(0) = botn::differenceAngles(yaw, that.yaw);
  d(1) = v - that.v;
  d(2) = w - that.w;

  return d;
}

} // namespace odometry

} // namespace differential

} // namespace machina

std::ostream &operator << (std::ostream &out, const machina::differential::odometry::State &state)
{
  return (out << "(" << state.yaw << ", " << botn::angleToRadians(state.yaw) << ", " << state.v << ", " << state.w << ")");
}
