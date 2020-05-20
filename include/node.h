/**
 * 2020
 *
 * @author Rodolphe Perrin
 */

#ifndef DIFF_DRIVE_NODE_H
#define DIFF_DRIVE_NODE_H

#include <ros/ros.h>

#include <signal.h>

namespace diff_drive
{

/**
 * @brief Simple node template.
 *
 * The `Server` class is expected to have a constructor accepting a
 * `ros::NodeHandle` pointing to the underlying ROS node's private ("~") namespace
 * (so it can be used to retrieve parameter values). The constructor is expected to
 * set up any ROS facilities required by the server (subscribers, publisher, timers
 * etc) and then return. The constructor **must** return: if active looping is
 * required, use a ROS timer or other similar mechanism to repeatedly execute a
 * callback function.
 *
 * When the node receives an external termination request (e.g. by the user
 * pressing `Ctrl+C` on the terminal), the server object is deleted before the ROS
 * context is brought down. Therefore any required cleanup actions can be safely
 * performed in the destructor of the `Server` class.
 */
template<class Server>
class Node
{
  /** @brief Pointer to the running node server. */
  static boost::shared_ptr<Server> server_;

  /**
   * @brief Process termination signal handler.
   */
  static void signalHandler(int signal);

public:
  /**
   * @brief Class constructor.
   */
  Node(const std::string &name, int argc, char** argv);

  /**
   * @brief Spin the node thread, deleting the server object on termination.
   */
  void spin();
};

/** @brief Definition of static server pointer member field. */
template<class Server> boost::shared_ptr<Server> Node<Server>::server_;

template<class Server>
Node<Server>::Node(const std::string &name, int argc, char** argv)
{
  ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
  signal(SIGINT, &Node::signalHandler);

  ros::NodeHandle node("~");
  server_.reset(new Server(node));
}

template<class Server>
void Node<Server>::signalHandler(int signal)
{
  server_.reset();

  ros::shutdown();
}

template<class Server>
void Node<Server>::spin()
{
  ros::spin();
  server_.reset();
}

template<class Server>
int init(const std::string &name, int argc, char** argv)
{
  Node<Server> node(name, argc, argv);

  node.spin();

  return 0;
}

} // namespace diff_drive

#define DIFF_DRIVE_NODE(NAME, SERVER) int main(int argc, char** argv) {return diff_drive::init<SERVER>(NAME, argc, argv);}

#endif
