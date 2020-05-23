/**
 * (C) 2020
 *
 * @author Rodolphe Perrin
 */

#include <odometry/state.h>

namespace diff_drive
{
namespace odometry
{
State::State(){
    //Nothing to do.
}

State::State(double x_0, double y_0, double vx_0, double vy_0)
{
    state_vector_ << x_0, y_0, vx_0, vy_0;
}

} // namespace odometry
} // namespace diff_drive


