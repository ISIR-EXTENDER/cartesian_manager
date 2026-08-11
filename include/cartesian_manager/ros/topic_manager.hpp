#pragma once

#include <any>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "rclcpp/rclcpp.hpp"

namespace ros_cartesian_manager
{
  /**
   * @brief Lightweight owner and lookup table for ROS publishers and subscribers.
   *
   * TopicManager centralizes the repetitive ROS boilerplate needed to create publishers and
   * subscribers, while keeping Cartesian-specific logic outside of this class. Callers register
   * each publisher or subscriber with a stable string key, then use that key later to publish,
   * remove, or check whether the ROS handle exists.
   *
   * The class is intentionally generic but minimal:
   * - Message types are selected with templates, for example addPublisher<TwistStamped>().
   * - Subscribers receive const message references and decide themselves where to store data.
   * - Publishers and subscribers are stored type-erased in std::any so different message types can
   *   share the same maps.
   * - No message conversion, service handling, or controller-specific state lives here.
   *
   * The rclcpp::Node passed to the constructor must outlive TopicManager. The stored publisher and
   * subscription shared pointers keep ROS communication alive for as long as they remain in the
   * maps.
   *
   * Example:
   * @code
   * topic_manager_.addSubscriber<geometry_msgs::msg::PoseStamped>(
   *     "ee_pose", "/ee_pose",
   *     [this](const geometry_msgs::msg::PoseStamped &msg) {
   *       robot_context_.ee_pose.frame_id = msg.header.frame_id;
   *     });
   *
   * topic_manager_.addPublisher<geometry_msgs::msg::TwistStamped>(
   *     "cartesian_command", "/cartesian_command");
   * topic_manager_.publish("cartesian_command", command_msg);
   * @endcode
   */
  class TopicManager
  {
  public:
    /**
     * @brief Construct a topic manager bound to an existing ROS node.
     *
     * The node reference must outlive this manager because publishers/subscribers are created from
     * it and logging/time access uses it.
     *
     * @param node ROS node used to create publishers and subscribers.
     */
    explicit TopicManager(rclcpp::Node &node);
    ~TopicManager() = default;

    /**
     * @brief Create and store a named publisher.
     *
     * If a publisher with the same name already exists, it is replaced. The name is only an
     * internal lookup key; it does not have to match the ROS topic name.
     *
     * @tparam MessageT ROS message type published by this publisher.
     * @param name Internal publisher key.
     * @param topic_name ROS topic name.
     * @param qos_history_depth QoS history depth passed to rclcpp.
     */
    template <typename MessageT>
    void addPublisher(const std::string &name, const std::string &topic_name,
                      std::size_t qos_history_depth = 10);

    /**
     * @brief Create and store a named subscriber.
     *
     * The callback receives a const reference to the ROS message, so callers can directly convert
     * or store data into their own state. If a subscriber with the same name already exists, it is
     * replaced.
     *
     * @tparam MessageT ROS message type received by this subscriber.
     * @param name Internal subscriber key.
     * @param topic_name ROS topic name.
     * @param callback Function called for each received message.
     * @param qos_history_depth QoS history depth passed to rclcpp.
     */
    template <typename MessageT>
    void addSubscriber(const std::string &name, const std::string &topic_name,
                       std::function<void(const MessageT &)> callback,
                       std::size_t qos_history_depth = 10);

    /**
     * @brief Remove a named publisher, if present.
     *
     * Removing the stored shared pointer destroys the ROS publisher unless another owner exists.
     */
    void removePublisher(const std::string &name);

    /**
     * @brief Remove a named subscriber, if present.
     *
     * Removing the stored shared pointer unsubscribes unless another owner exists.
     */
    void removeSubscriber(const std::string &name);

    /**
     * @brief Check whether a publisher key is registered.
     */
    bool hasPublisher(const std::string &name) const;

    /**
     * @brief Check whether a subscriber key is registered.
     */
    bool hasSubscriber(const std::string &name) const;

    /**
     * @brief Publish a message with a named publisher.
     *
     * The template message type must match the type used in addPublisher for this name. If the name
     * is missing or the stored publisher has a different message type, a warning is logged and
     * nothing is published.
     *
     * @tparam MessageT ROS message type to publish.
     * @param name Internal publisher key.
     * @param msg Message to publish.
     */
    template <typename MessageT> void publish(const std::string &name, const MessageT &msg);

    /**
     * @brief Return the current node time in seconds.
     */
    double nowSec() const;

  private:
    /**
     * @brief Return a typed publisher from the type-erased publisher map.
     *
     * Returns nullptr when the name is missing or the requested MessageT does not match the stored
     * publisher type.
     */
    template <typename MessageT>
    typename rclcpp::Publisher<MessageT>::SharedPtr getPublisher(const std::string &name) const;

    rclcpp::Node &node_;

    // std::any stores rclcpp::Publisher<MessageT>::SharedPtr and
    // rclcpp::Subscription<MessageT>::SharedPtr without forcing a common message type.
    std::unordered_map<std::string, std::any> publishers_;
    std::unordered_map<std::string, std::any> subscribers_;
  };

  template <typename MessageT>
  void TopicManager::addPublisher(const std::string &name, const std::string &topic_name,
                                  std::size_t qos_history_depth)
  {
    if (topic_name.empty())
    {
      RCLCPP_WARN(node_.get_logger(), "Skipping publisher '%s' with an empty topic name",
                  name.c_str());
      return;
    }

    publishers_[name] = node_.create_publisher<MessageT>(topic_name, qos_history_depth);
  }

  template <typename MessageT>
  void TopicManager::addSubscriber(const std::string &name, const std::string &topic_name,
                                   std::function<void(const MessageT &)> callback,
                                   std::size_t qos_history_depth)
  {
    if (topic_name.empty())
    {
      RCLCPP_WARN(node_.get_logger(), "Skipping subscriber '%s' with an empty topic name",
                  name.c_str());
      return;
    }

    std::function<void(std::shared_ptr<MessageT>)> ros_callback =
        [callback = std::move(callback)](std::shared_ptr<MessageT> msg) { callback(*msg); };

    subscribers_[name] =
        node_.create_subscription<MessageT>(topic_name, qos_history_depth, std::move(ros_callback));
  }

  template <typename MessageT>
  void TopicManager::publish(const std::string &name, const MessageT &msg)
  {
    const auto publisher = getPublisher<MessageT>(name);
    if (!publisher)
    {
      RCLCPP_WARN(node_.get_logger(), "No publisher named '%s' for requested message type",
                  name.c_str());
      return;
    }

    publisher->publish(msg);
  }

  template <typename MessageT>
  typename rclcpp::Publisher<MessageT>::SharedPtr TopicManager::getPublisher(
      const std::string &name) const
  {
    const auto publisher = publishers_.find(name);
    if (publisher == publishers_.end())
    {
      return nullptr;
    }

    try
    {
      return std::any_cast<typename rclcpp::Publisher<MessageT>::SharedPtr>(publisher->second);
    }
    catch (const std::bad_any_cast &)
    {
      return nullptr;
    }
  }
} // namespace ros_cartesian_manager
