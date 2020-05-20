/**
 * (C) 2019 Cyberworks Robotics, inc.
 *
 * @author Helio Perroni Filho
 */

#include <machina/differential/odometry/encoders.h>

namespace machina
{

namespace differential
{

namespace odometry
{

Encoders::Reading::Reading():
  v(0.0)
{
  // Nothing to do.
}

Encoders::Reading::Reading(double v):
  v(v)
{
  // Nothing to do.
}

Encoders::Reading::Reading(double weight_0, double weight_j, const std::vector<Encoders::Reading> &samples)
{
  v = weight_0 * samples[0].v;
  for (int i = 1, n = samples.size(); i < n; ++i)
    v += weight_j * samples[i].v;
}

Encoders::Reading::Difference Encoders::Reading::operator - (const Reading &that) const
{
  Difference d;

  d(0) = v - that.v;

  return d;
}

Encoders::Encoders(double coef_left, double coef_right, double s2_v):
  coef_left_(coef_left),
  coef_right_(coef_right),
  noise_(Covariance::Constant(0.0))
{
  noise_(0, 0) = s2_v;
}

State Encoders::estimate() const
{
  State state;

  state.v = reading_.v;

  return state;
}

std::vector<Encoders::Reading> Encoders::estimate(const std::vector<State> predictions)
{
  std::vector<Reading> readings;
  readings.reserve(predictions.size());

  for (const State &prediction: predictions)
    readings.emplace_back(prediction.v);

  return std::move(readings);
}

const Encoders::Reading &Encoders::read() const
{
  return reading_;
}

void Encoders::update(double dt, double ticks_left, double ticks_right)
{
  reading_.v = 0.5 * (ticks_left * coef_left_ + ticks_right * coef_right_) / dt;
}

} // namespace odometry

} // namespace differential

} // namespace machina
