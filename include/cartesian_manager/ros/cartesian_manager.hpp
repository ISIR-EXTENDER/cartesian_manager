#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/joint_state.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "cartesian_manager/cartesian_manager_parameters.hpp"
#include "cartesian_manager/core/manager.hpp"
#include "cartesian_manager/ros/parameter_parsing.hpp"
#include "cartesian_manager/ros/topic_manager.hpp"

namespace ros_cartesian_manager
{

  class CartesianManagerROS : public rclcpp::Node
  {
  public:
    explicit CartesianManagerROS(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    void modeRequestCallback(const std_msgs::msg::String &mode_request);
    void poseTargetCallback(const geometry_msgs::msg::PoseStamped &pose_target);

    void jointStatesSubscriberCallback(const sensor_msgs::msg::JointState &msg);
    void eePoseSubscriberCallback(const geometry_msgs::msg::PoseStamped &msg);
    void eeVelSubscriberCallback(const geometry_msgs::msg::TwistStamped &msg);
    void eeJacobianSubscriberCallback(const std_msgs::msg::Float64MultiArray &msg);
    void joystickSubscriberCallback(const geometry_msgs::msg::TwistStamped &msg);
    void tabletSubscriberCallback(const geometry_msgs::msg::TwistStamped &msg);
    void visualServoingSubscriberCallback(const geometry_msgs::msg::TwistStamped &msg);

  private:
    void setupSubscribers();
    void setupPublishers();
    void readParameters();
    void applyConfig(const ManagerConfig &config, bool initial);
    void refreshParameters();
    void updateVelocity();

    rcl_interfaces::msg::SetParametersResult validateParameterUpdate(
        const std::vector<rclcpp::Parameter> &parameters) const;
    void publishJointTargetCommand(const std::optional<manager_core::JointTargetCommand> &command);

    TopicManager topic_manager_;
    manager_core::Manager manager_;
    manager_core::RobotContext robot_context_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<cartesian_manager::ParamListener> param_listener_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_validator_handle_;
    cartesian_manager::Params params_;
    ManagerConfig config_;
  };
} // namespace ros_cartesian_manager
