/**
 * (C)2020
 *
 * @author Rodolphe Perrin
 */

#ifndef DIFF_DRIVE_NODELET_H
#define DIFF_DRIVE_NODELET_H

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <memory>

namespace diff_drive
{

/**
 * @brief Simple nodelet template.
 *
 * The `Server` class is expected to have a constructor accepting three arguments:
 * a name `string` and two `ros::NodeHandle` values, one pointing to the underlying
 * node, the other to the node's private namespace (so it can be used to retrieve
 * parameter values). The constructor is expected to set up any ROS facilities
 * required by the server (subscribers, publisher, timers etc) before returning.
 * Any cleanup code required to run at termination time should be included in the
 * class destructor.
 */
template<class Server>
class Nodelet: public nodelet::Nodelet
{
  /** @brief Server encapsulated by this nodelet. */
  std::shared_ptr<Server> server_;

public:
  // See nodelet::Nodelet::onInit() documentation.
  virtual void onInit()
  {
    server_.reset(new Server(getPrivateNodeHandle()));
  }
};

} // namespace diff_drive

#define DIFF_DRIVE_DEFINE_NODELET(SERVER) class SERVER ## Nodelet: public diff_drive::Nodelet<SERVER> {};

#define DIFF_DRIVE_EXPORT_NODELET(SERVER) PLUGINLIB_EXPORT_CLASS(SERVER ## Nodelet, nodelet::Nodelet)

#endif
