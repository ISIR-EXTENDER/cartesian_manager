#include "cartesian_manager/ros/cartesian_manager.hpp"

#include <chrono>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace ros_cartesian_manager
{
  namespace
  {
    constexpr const char *kOutputCommandPublisher = "output_command";
    constexpr const char *kJointTargetCommandPublisher = "joint_target_command";
    constexpr const char *kBehaviourPassthroughMode = "behaviour/passthrough";
    constexpr const char *kJointTargetModePrefix = "behaviour/joint_target/";
    constexpr const char *kPoseTargetModePrefix = "behaviour/pose_target/";

    std::chrono::nanoseconds timerPeriod(double update_rate_hz)
    {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / update_rate_hz));
    }

    bool inputConfigsEqual(const std::vector<manager_core::InputConfig> &lhs,
                           const std::vector<manager_core::InputConfig> &rhs)
    {
      if (lhs.size() != rhs.size())
      {
        return false;
      }
      for (std::size_t index = 0; index < lhs.size(); ++index)
      {
        if (lhs[index].source != rhs[index].source ||
            lhs[index].timeout_sec != rhs[index].timeout_sec ||
            lhs[index].enabled != rhs[index].enabled)
        {
          return false;
        }
      }
      return true;
    }

    bool tuningConfigsEqual(const manager_core::ManagerConfig &lhs,
                            const manager_core::ManagerConfig &rhs)
    {
      const auto &a = lhs.pose_targets;
      const auto &b = rhs.pose_targets;
      return lhs.jaco.min_radius == rhs.jaco.min_radius &&
             lhs.jaco.max_angular_velocity == rhs.jaco.max_angular_velocity &&
             lhs.snake.gain == rhs.snake.gain &&
             lhs.rate_limiter.max_linear_acceleration == rhs.rate_limiter.max_linear_acceleration &&
             lhs.rate_limiter.max_angular_acceleration ==
                 rhs.rate_limiter.max_angular_acceleration &&
             a.linear_kp == b.linear_kp && a.angular_kp == b.angular_kp &&
             a.max_linear_velocity == b.max_linear_velocity &&
             a.max_angular_velocity == b.max_angular_velocity &&
             a.position_tolerance == b.position_tolerance &&
             a.orientation_tolerance == b.orientation_tolerance;
    }

    double stampSec(const builtin_interfaces::msg::Time &stamp, const double fallback_sec)
    {
      if (stamp.sec == 0 && stamp.nanosec == 0)
      {
        return fallback_sec;
      }

      return rclcpp::Time(stamp).seconds();
    }

    std::string frameOrDefault(const std::string &frame_id, const std::string &default_frame_id)
    {
      return frame_id.empty() ? default_frame_id : frame_id;
    }

    manager_core::CartesianVelocity twistToCommand(const geometry_msgs::msg::TwistStamped &msg,
                                                   const std::string &default_frame_id)
    {
      manager_core::CartesianVelocity command;
      command.linear = Eigen::Vector3d(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
      command.angular =
          Eigen::Vector3d(msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z);
      command.frame_id = frameOrDefault(msg.header.frame_id, default_frame_id);
      return command;
    }

    geometry_msgs::msg::TwistStamped commandToMsg(const manager_core::CartesianVelocity &command,
                                                  const rclcpp::Time &stamp,
                                                  const std::string &output_frame_id)
    {
      geometry_msgs::msg::TwistStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = frameOrDefault(command.frame_id, output_frame_id);
      msg.twist.linear.x = command.linear.x();
      msg.twist.linear.y = command.linear.y();
      msg.twist.linear.z = command.linear.z();
      msg.twist.angular.x = command.angular.x();
      msg.twist.angular.y = command.angular.y();
      msg.twist.angular.z = command.angular.z();
      return msg;
    }

    sensor_msgs::msg::JointState jointTargetToMsg(const manager_core::JointTargetCommand &command,
                                                  const rclcpp::Time &stamp)
    {
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = stamp;
      msg.name = command.joint_names;
      msg.position = command.positions;
      return msg;
    }

    std::optional<Eigen::MatrixXd> jacobianFromMsg(const std_msgs::msg::Float64MultiArray &msg,
                                                   const rclcpp::Logger &logger)
    {
      if (msg.data.empty())
      {
        return Eigen::MatrixXd{};
      }

      std::size_t rows = 0;
      std::size_t cols = 0;
      std::size_t data_offset = 0;
      std::size_t row_stride = 0;
      if (msg.layout.dim.size() == 2)
      {
        rows = msg.layout.dim[0].size;
        cols = msg.layout.dim[1].size;
        data_offset = msg.layout.data_offset;
        row_stride = msg.layout.dim[1].stride == 0 ? cols : msg.layout.dim[1].stride;
      }
      else if (msg.data.size() % 6 == 0)
      {
        rows = 6;
        cols = msg.data.size() / rows;
        row_stride = cols;
      }
      else
      {
        RCLCPP_WARN(logger, "Ignoring ee_jac message without a 2D layout and non-6xN data");
        return std::nullopt;
      }

      if (rows == 0 || cols == 0 || row_stride < cols || data_offset >= msg.data.size())
      {
        RCLCPP_WARN(logger, "Ignoring ee_jac message with inconsistent dimensions");
        return std::nullopt;
      }

      const auto last_index = data_offset + (rows - 1) * row_stride + (cols - 1);
      if (last_index >= msg.data.size())
      {
        RCLCPP_WARN(logger, "Ignoring ee_jac message with inconsistent dimensions");
        return std::nullopt;
      }

      Eigen::MatrixXd jacobian(rows, cols);
      for (std::size_t row = 0; row < rows; ++row)
      {
        for (std::size_t col = 0; col < cols; ++col)
        {
          jacobian(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
              msg.data[data_offset + row * row_stride + col];
        }
      }

      return jacobian;
    }
  } // namespace

  CartesianManagerROS::CartesianManagerROS(const rclcpp::NodeOptions &options)
      : rclcpp::Node("cartesian_manager", options), topic_manager_(*this)
  {
    readParameters();
    applyConfig(config_, true);
    timer_ = create_wall_timer(timerPeriod(config_.update_rate_hz), [this]() { updateVelocity(); });
  }

  void CartesianManagerROS::readParameters()
  {
    param_listener_ = std::make_shared<cartesian_manager::ParamListener>(this);
    params_ = param_listener_->get_params();
    config_ = parseManagerConfig(params_);
    parameter_validator_handle_ =
        add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &parameters) {
          return validateParameterUpdate(parameters);
        });
  }

  void CartesianManagerROS::applyConfig(const ManagerConfig &config, bool initial)
  {
    const bool tuning_changed = initial || !tuningConfigsEqual(config_.manager, config.manager);
    const bool input_changed =
        initial || !inputConfigsEqual(config_.manager.inputs, config.manager.inputs);
    config_ = config;

    if (initial)
      manager_.configure(config_.manager);
    else if (tuning_changed)
      manager_.updateTuning(config_.manager);

    if (input_changed && !initial)
      manager_.configureInputChannels(config_.manager.inputs);

    if (initial)
    {
      setupPublishers();
      setupSubscribers();
    }
  }

  void CartesianManagerROS::refreshParameters()
  {
    auto updated_params = params_;
    if (!param_listener_ || !param_listener_->try_update_params(updated_params))
      return;

    ManagerConfig updated_config;
    try
    {
      updated_config = parseManagerConfig(updated_params);
    }
    catch (const std::invalid_argument &error)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Ignoring invalid runtime parameter update: %s", error.what());
      return;
    }

    params_ = updated_params;
    applyConfig(updated_config, false);

    RCLCPP_INFO(get_logger(), "Applied updated cartesian_manager parameters");
  }

  rcl_interfaces::msg::SetParametersResult CartesianManagerROS::validateParameterUpdate(
      const std::vector<rclcpp::Parameter> &parameters) const
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    try
    {
      parseManagerConfig(updatedParamsForRequest(params_, parameters));
    }
    catch (const std::exception &error)
    {
      result.successful = false;
      result.reason = error.what();
    }

    return result;
  }

  void CartesianManagerROS::setupPublishers()
  {
    topic_manager_.addPublisher<geometry_msgs::msg::TwistStamped>(kOutputCommandPublisher,
                                                                  config_.topics.output_command);
    topic_manager_.addPublisher<sensor_msgs::msg::JointState>(kJointTargetCommandPublisher,
                                                              config_.topics.joint_target_command);
  }

  void CartesianManagerROS::setupSubscribers()
  {
    topic_manager_.addSubscriber<std_msgs::msg::String>(
        "mode_request", config_.topics.mode_request,
        std::bind(&CartesianManagerROS::modeRequestCallback, this, std::placeholders::_1));

    topic_manager_.addSubscriber<geometry_msgs::msg::PoseStamped>(
        "pose_target", config_.topics.pose_target,
        std::bind(&CartesianManagerROS::poseTargetCallback, this, std::placeholders::_1));

    topic_manager_.addSubscriber<geometry_msgs::msg::PoseStamped>(
        "ee_pose", config_.topics.ee_pose,
        std::bind(&CartesianManagerROS::eePoseSubscriberCallback, this, std::placeholders::_1));

    topic_manager_.addSubscriber<geometry_msgs::msg::TwistStamped>(
        "ee_vel", config_.topics.ee_vel,
        std::bind(&CartesianManagerROS::eeVelSubscriberCallback, this, std::placeholders::_1));

    topic_manager_.addSubscriber<std_msgs::msg::Float64MultiArray>(
        "ee_jac", config_.topics.ee_jac,
        std::bind(&CartesianManagerROS::eeJacobianSubscriberCallback, this, std::placeholders::_1));

    topic_manager_.addSubscriber<sensor_msgs::msg::JointState>(
        "joint_states", config_.topics.joint_states,
        std::bind(&CartesianManagerROS::jointStatesSubscriberCallback, this,
                  std::placeholders::_1));

    for (const auto &input : config_.manager.inputs)
    {
      switch (input.source)
      {
      case manager_core::InputSource::JOYSTICK:
        topic_manager_.addSubscriber<geometry_msgs::msg::TwistStamped>(
            "joystick_command", config_.topics.joystick_command,
            std::bind(&CartesianManagerROS::joystickSubscriberCallback, this, std::placeholders::_1));
        break;
      case manager_core::InputSource::TABLET:
        topic_manager_.addSubscriber<geometry_msgs::msg::TwistStamped>(
            "tablet_command", config_.topics.tablet_command,
            std::bind(&CartesianManagerROS::tabletSubscriberCallback, this, std::placeholders::_1));
        break;
      case manager_core::InputSource::VISUAL_SERVOING:
        topic_manager_.addSubscriber<geometry_msgs::msg::TwistStamped>(
            "visual_servoing_command", config_.topics.visual_servoing_command,
            std::bind(&CartesianManagerROS::visualServoingSubscriberCallback, this,
                      std::placeholders::_1));
        break;
      }
    }
  }

  void CartesianManagerROS::joystickSubscriberCallback(const geometry_msgs::msg::TwistStamped &msg)
  {
    const auto now_sec = topic_manager_.nowSec();
    const auto input_frame_id = frameOrDefault(msg.header.frame_id, config_.default_input_frame_id);
    const manager_core::FramesConfig temp_frames = config_.manager.frames;
    const auto command = twistToCommand(msg, config_.default_input_frame_id);
    if (input_frame_id == temp_frames.hybrid_frame)
    {
      robot_context_.updateHybridPose(command.angular);
    }

    if (!manager_.setInputCommand(manager_core::InputSource::JOYSTICK, command,
                                  stampSec(msg.header.stamp, now_sec)))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring joystick command because the source is not configured");
    }
  }

  void CartesianManagerROS::tabletSubscriberCallback(const geometry_msgs::msg::TwistStamped &msg)
  {
    const auto now_sec = topic_manager_.nowSec();
    const auto input_frame_id = frameOrDefault(msg.header.frame_id, config_.default_input_frame_id);
    const manager_core::FramesConfig temp_frames = config_.manager.frames;
    const auto command = twistToCommand(msg, config_.default_input_frame_id);
    if (input_frame_id == temp_frames.hybrid_frame)
    {
      robot_context_.updateHybridPose(command.angular);
    }

    if (!manager_.setInputCommand(manager_core::InputSource::TABLET, command,
                                  stampSec(msg.header.stamp, now_sec)))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ignoring tablet command because the source is not configured");
    }
  }

  void CartesianManagerROS::visualServoingSubscriberCallback(
      const geometry_msgs::msg::TwistStamped &msg)
  {
    const auto now_sec = topic_manager_.nowSec();
    const auto command = twistToCommand(msg, config_.default_input_frame_id);
    if (!manager_.setInputCommand(manager_core::InputSource::VISUAL_SERVOING, command,
                                  stampSec(msg.header.stamp, now_sec)))
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring visual-servoing command in frame '%s'; expected input frame '%s'",
          command.frame_id.c_str(), config_.default_input_frame_id.c_str());
    }
  }

  void CartesianManagerROS::jointStatesSubscriberCallback(const sensor_msgs::msg::JointState &msg)
  {
    robot_context_.joint_names = msg.name;
    if (msg.position.empty())
    {
      robot_context_.joint_positions = Eigen::VectorXd{};
      return;
    }

    robot_context_.joint_positions = Eigen::Map<const Eigen::VectorXd>(
        msg.position.data(), static_cast<Eigen::Index>(msg.position.size()));
  }

  void CartesianManagerROS::eePoseSubscriberCallback(const geometry_msgs::msg::PoseStamped &msg)
  {
    robot_context_.ee_pose.position =
        Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    robot_context_.ee_pose.orientation =
        Eigen::Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y,
                           msg.pose.orientation.z);
    robot_context_.ee_pose.frame_id =
        frameOrDefault(msg.header.frame_id, config_.manager.frames.base_frame);
  }

  void CartesianManagerROS::eeVelSubscriberCallback(const geometry_msgs::msg::TwistStamped &msg)
  {
    robot_context_.ee_vel = twistToCommand(msg, config_.default_input_frame_id);
  }

  void CartesianManagerROS::eeJacobianSubscriberCallback(
      const std_msgs::msg::Float64MultiArray &msg)
  {
    auto jacobian = jacobianFromMsg(msg, get_logger());
    if (jacobian)
    {
      robot_context_.ee_jac = std::move(*jacobian);
    }
  }

  void CartesianManagerROS::modeRequestCallback(const std_msgs::msg::String &mode_request)
  {
    const auto normalized_mode_request = normalizeParameterName(mode_request.data);
    const bool joint_target_request = normalized_mode_request.rfind(kJointTargetModePrefix, 0) == 0;
    const bool passthrough_request = normalized_mode_request == kBehaviourPassthroughMode;
    const bool pose_target_request = normalized_mode_request.rfind(kPoseTargetModePrefix, 0) == 0;

    if (!manager_.setMode(normalized_mode_request))
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid mode request '%s'", mode_request.data.c_str());
      return;
    }

    if (joint_target_request)
    {
      publishJointTargetCommand(manager_.activeJointTargetCommand());
      manager_.setMode(kBehaviourPassthroughMode);
      return;
    }

    if (passthrough_request || pose_target_request)
      publishJointTargetCommand(std::nullopt);
  }

  void CartesianManagerROS::poseTargetCallback(const geometry_msgs::msg::PoseStamped &pose_target)
  {
    if (pose_target.header.frame_id != config_.manager.frames.base_frame)
    {
      RCLCPP_WARN(get_logger(),
                  "Ignoring pose target in frame '%s'; expected configured base frame '%s'",
                  pose_target.header.frame_id.c_str(), config_.manager.frames.base_frame.c_str());
      return;
    }

    manager_core::CartesianPose target;
    target.frame_id = pose_target.header.frame_id;
    target.position = Eigen::Vector3d(pose_target.pose.position.x, pose_target.pose.position.y,
                                      pose_target.pose.position.z);
    target.orientation = Eigen::Quaterniond(
        pose_target.pose.orientation.w, pose_target.pose.orientation.x,
        pose_target.pose.orientation.y, pose_target.pose.orientation.z);

    std::string error;
    if (!manager_.setPoseTarget(target, &error))
    {
      RCLCPP_WARN(get_logger(), "Ignoring invalid pose target: %s", error.c_str());
      return;
    }

    publishJointTargetCommand(std::nullopt);
  }

  void CartesianManagerROS::publishJointTargetCommand(
      const std::optional<manager_core::JointTargetCommand> &command)
  {
    if (command)
    {
      topic_manager_.publish(kJointTargetCommandPublisher, jointTargetToMsg(*command, now()));
      return;
    }

    sensor_msgs::msg::JointState cancel_msg;
    cancel_msg.header.stamp = now();
    topic_manager_.publish(kJointTargetCommandPublisher, cancel_msg);
  }

  void CartesianManagerROS::updateVelocity()
  {
    refreshParameters();

    const auto now = this->now();
    const auto command =
        manager_.update(now.seconds(), 1.0 / config_.update_rate_hz, robot_context_)
            .value_or(manager_core::CartesianVelocity{});
    topic_manager_.publish(kOutputCommandPublisher,
                           commandToMsg(command, now, config_.output_frame_id));
  }
} // namespace ros_cartesian_manager
