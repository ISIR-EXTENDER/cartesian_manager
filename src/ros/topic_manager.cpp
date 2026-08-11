#include "cartesian_manager/ros/topic_manager.hpp"

namespace ros_cartesian_manager
{
  TopicManager::TopicManager(rclcpp::Node &node) : node_(node)
  {
  }

  void TopicManager::removePublisher(const std::string &name)
  {
    publishers_.erase(name);
  }

  void TopicManager::removeSubscriber(const std::string &name)
  {
    subscribers_.erase(name);
  }

  bool TopicManager::hasPublisher(const std::string &name) const
  {
    return publishers_.find(name) != publishers_.end();
  }

  bool TopicManager::hasSubscriber(const std::string &name) const
  {
    return subscribers_.find(name) != subscribers_.end();
  }

  double TopicManager::nowSec() const
  {
    return node_.now().seconds();
  }
} // namespace ros_cartesian_manager
