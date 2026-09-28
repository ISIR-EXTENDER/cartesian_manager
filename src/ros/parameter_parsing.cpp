#include "cartesian_manager/ros/parameter_parsing.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace ros_cartesian_manager
{
  namespace
  {
    constexpr double kMaxUpdateRateHz = 1000.0;

    bool isFinite(double value)
    {
      return std::isfinite(value);
    }

    void requireNonEmpty(const std::string &value, const std::string &name)
    {
      if (value.empty())
        throw std::invalid_argument(name + " must not be empty");
    }

    void requirePositive(double value, const std::string &name)
    {
      if (!isFinite(value) || value <= 0.0)
        throw std::invalid_argument(name + " must be finite and > 0.0");
    }

    void requireAtMost(double value, double max_value, const std::string &name)
    {
      if (!isFinite(value) || value > max_value)
        throw std::invalid_argument(name + " must be finite and <= " + std::to_string(max_value));
    }

    void requireNonNegative(double value, const std::string &name)
    {
      if (!isFinite(value) || value < 0.0)
        throw std::invalid_argument(name + " must be finite and >= 0.0");
    }

    void requireFinite(double value, const std::string &name)
    {
      if (!isFinite(value))
        throw std::invalid_argument(name + " must be finite");
    }

    bool isKnownCommandFrame(const manager_core::FramesConfig &frames, const std::string &frame_id)
    {
      return frame_id == frames.base_frame || frame_id == frames.ee_frame ||
             frame_id == frames.hybrid_frame;
    }

    void requireKnownCommandFrame(const manager_core::FramesConfig &frames,
                                  const std::string &frame_id, const std::string &name)
    {
      if (!isKnownCommandFrame(frames, frame_id))
        throw std::invalid_argument(name + " must match frames.base_frame, frames.ee_frame, or "
                                           "frames.hybrid_frame");
    }

    std::vector<std::string> normalizedNonEmptyNames(const std::vector<std::string> &names)
    {
      std::vector<std::string> normalized_names;
      normalized_names.reserve(names.size());

      for (const auto &name : names)
      {
        auto normalized = normalizeParameterName(name);
        if (!normalized.empty())
          normalized_names.push_back(std::move(normalized));
      }

      return normalized_names;
    }

    void requireUniqueNames(const std::vector<std::string> &names, const std::string &name)
    {
      std::unordered_set<std::string> seen;
      for (const auto &value : names)
      {
        if (!seen.insert(value).second)
          throw std::invalid_argument(name + " contains duplicate entry '" + value + "'");
      }
    }

    manager_core::InputSource inputSourceFromName(const std::string &name)
    {
      if (name == "joystick")
        return manager_core::InputSource::JOYSTICK;

      if (name == "tablet")
        return manager_core::InputSource::TABLET;

      if (name == "visual_servoing")
        return manager_core::InputSource::VISUAL_SERVOING;

      throw std::invalid_argument("inputs.sources contains unsupported source '" + name + "'");
    }

    manager_core::InputConfig makeInputConfig(manager_core::InputSource source,
                                              const cartesian_manager::Params &params)
    {
      switch (source)
      {
      case manager_core::InputSource::JOYSTICK:
        requirePositive(params.inputs.joystick.timeout_sec, "inputs.joystick.timeout_sec");
        return manager_core::InputConfig{source, params.inputs.joystick.timeout_sec,
                                         params.inputs.joystick.enabled};

      case manager_core::InputSource::TABLET:
        requirePositive(params.inputs.tablet.timeout_sec, "inputs.tablet.timeout_sec");
        return manager_core::InputConfig{source, params.inputs.tablet.timeout_sec,
                                         params.inputs.tablet.enabled};

      case manager_core::InputSource::VISUAL_SERVOING:
        requirePositive(params.inputs.visual_servoing.timeout_sec,
                        "inputs.visual_servoing.timeout_sec");
        return manager_core::InputConfig{source, params.inputs.visual_servoing.timeout_sec,
                                         params.inputs.visual_servoing.enabled};
      }

      throw std::invalid_argument("inputs.sources contains an unknown source enum value");
    }

    std::vector<manager_core::JointTarget> makeJointTargets(
        const std::vector<std::string> &target_names, const std::vector<double> &positions,
        std::size_t joint_count)
    {
      std::vector<manager_core::JointTarget> targets;
      if (joint_count == 0 || target_names.empty())
        return targets;

      const auto expected_position_count = target_names.size() * joint_count;
      if (positions.size() != expected_position_count)
      {
        throw std::invalid_argument("behaviours.joint_targets.positions has " +
                                    std::to_string(positions.size()) + " values, expected " +
                                    std::to_string(expected_position_count));
      }

      targets.reserve(target_names.size());
      for (std::size_t target_index = 0; target_index < target_names.size(); ++target_index)
      {
        const auto first_position = target_index * joint_count;
        const auto first = positions.begin() + static_cast<std::ptrdiff_t>(first_position);
        const auto last = first + static_cast<std::ptrdiff_t>(joint_count);
        if (!std::all_of(first, last, isFinite))
        {
          throw std::invalid_argument(
              "behaviours.joint_targets.positions contains a non-finite value");
        }

        manager_core::JointTarget target;
        target.name = target_names[target_index];
        target.positions.assign(first, last);
        targets.push_back(std::move(target));
      }

      return targets;
    }

    manager_core::PoseTargetConfig makePoseTargetConfig(const cartesian_manager::Params &params,
                                                        const manager_core::FramesConfig &frames)
    {
      const auto &source = params.behaviours.pose_targets;
      manager_core::PoseTargetConfig config;
      config.target_names = normalizedNonEmptyNames(source.target_names);
      if (config.target_names.size() != source.target_names.size() &&
          !(source.target_names.size() == 1 && source.target_names.front().empty()))
      {
        throw std::invalid_argument("behaviours.pose_targets.target_names contains an empty name");
      }
      requireUniqueNames(config.target_names, "behaviours.pose_targets.target_names");

      config.linear_kp = source.linear_kp;
      config.angular_kp = source.angular_kp;
      config.max_linear_velocity = source.max_linear_velocity;
      config.max_angular_velocity = source.max_angular_velocity;
      config.position_tolerance = source.position_tolerance;
      config.orientation_tolerance = source.orientation_tolerance;
      requireNonNegative(config.linear_kp, "behaviours.pose_targets.linear_kp");
      requireNonNegative(config.angular_kp, "behaviours.pose_targets.angular_kp");
      requirePositive(config.max_linear_velocity, "behaviours.pose_targets.max_linear_velocity");
      requirePositive(config.max_angular_velocity, "behaviours.pose_targets.max_angular_velocity");
      requireNonNegative(config.position_tolerance, "behaviours.pose_targets.position_tolerance");
      requireNonNegative(config.orientation_tolerance,
                         "behaviours.pose_targets.orientation_tolerance");

      const auto count = config.target_names.size();
      if (count == 0)
        return config;

      if (source.frame_ids.size() != count || source.positions.size() != count * 3 ||
          source.orientations.size() != count * 4)
      {
        throw std::invalid_argument(
            "behaviours.pose_targets arrays must contain one frame, "
            "three position values, and four orientation values per target");
      }

      config.targets.reserve(count);
      for (std::size_t index = 0; index < count; ++index)
      {
        manager_core::CartesianPose pose;
        pose.frame_id = source.frame_ids[index];
        if (pose.frame_id != frames.base_frame)
        {
          throw std::invalid_argument(
              "behaviours.pose_targets.frame_ids must match frames.base_frame");
        }
        pose.position =
            Eigen::Vector3d(source.positions[index * 3], source.positions[index * 3 + 1],
                            source.positions[index * 3 + 2]);
        pose.orientation = Eigen::Quaterniond(
            source.orientations[index * 4 + 3], source.orientations[index * 4],
            source.orientations[index * 4 + 1], source.orientations[index * 4 + 2]);
        if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite() ||
            !std::isfinite(pose.orientation.norm()) || pose.orientation.norm() <= 1.0e-9)
        {
          throw std::invalid_argument("behaviours.pose_targets contains a non-finite pose "
                                      "or zero quaternion");
        }
        pose.orientation.normalize();
        config.targets.push_back(std::move(pose));
      }
      return config;
    }

    manager_core::JointTargetBehaviourConfig makeJointTargetConfig(
        const cartesian_manager::Params &params)
    {
      const auto &joint_target_params = params.behaviours.joint_targets;

      manager_core::JointTargetBehaviourConfig config;
      config.joint_names = normalizedNonEmptyNames(joint_target_params.joint_names);
      requireUniqueNames(config.joint_names, "behaviours.joint_targets.joint_names");

      auto target_names = normalizedNonEmptyNames(joint_target_params.target_names);
      requireUniqueNames(target_names, "behaviours.joint_targets.target_names");

      config.targets =
          makeJointTargets(target_names, joint_target_params.positions, config.joint_names.size());
      return config;
    }
  } // namespace

  std::string normalizeParameterName(std::string name)
  {
    std::transform(name.begin(), name.end(), name.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    std::replace(name.begin(), name.end(), '-', '_');
    return name;
  }

  cartesian_manager::Params updatedParamsForRequest(
      cartesian_manager::Params params, const std::vector<rclcpp::Parameter> &parameters)
  {
    // Structural settings are read-only. Only tuning and input state can change at runtime.
    for (const auto &param : parameters)
    {
      const auto &name = param.get_name();
      if (name == "inputs.joystick.timeout_sec")
        params.inputs.joystick.timeout_sec = param.as_double();
      else if (name == "inputs.joystick.enabled")
        params.inputs.joystick.enabled = param.as_bool();
      else if (name == "inputs.tablet.timeout_sec")
        params.inputs.tablet.timeout_sec = param.as_double();
      else if (name == "inputs.tablet.enabled")
        params.inputs.tablet.enabled = param.as_bool();
      else if (name == "inputs.visual_servoing.timeout_sec")
        params.inputs.visual_servoing.timeout_sec = param.as_double();
      else if (name == "inputs.visual_servoing.enabled")
        params.inputs.visual_servoing.enabled = param.as_bool();
      else if (name == "shapers.jaco.min_radius")
        params.shapers.jaco.min_radius = param.as_double();
      else if (name == "shapers.jaco.max_angular_velocity")
        params.shapers.jaco.max_angular_velocity = param.as_double();
      else if (name == "shapers.snake.gain")
        params.shapers.snake.gain = param.as_double();
      else if (name == "rate_limiter.max_linear_acceleration")
        params.rate_limiter.max_linear_acceleration = param.as_double();
      else if (name == "rate_limiter.max_angular_acceleration")
        params.rate_limiter.max_angular_acceleration = param.as_double();
      else if (name == "behaviours.pose_targets.linear_kp")
        params.behaviours.pose_targets.linear_kp = param.as_double();
      else if (name == "behaviours.pose_targets.angular_kp")
        params.behaviours.pose_targets.angular_kp = param.as_double();
      else if (name == "behaviours.pose_targets.max_linear_velocity")
        params.behaviours.pose_targets.max_linear_velocity = param.as_double();
      else if (name == "behaviours.pose_targets.max_angular_velocity")
        params.behaviours.pose_targets.max_angular_velocity = param.as_double();
      else if (name == "behaviours.pose_targets.position_tolerance")
        params.behaviours.pose_targets.position_tolerance = param.as_double();
      else if (name == "behaviours.pose_targets.orientation_tolerance")
        params.behaviours.pose_targets.orientation_tolerance = param.as_double();
    }
    return params;
  }

  ManagerConfig parseManagerConfig(const cartesian_manager::Params &params)
  {
    ManagerConfig config;

    config.update_rate_hz = params.update_rate_hz;
    requirePositive(config.update_rate_hz, "update_rate_hz");
    requireAtMost(config.update_rate_hz, kMaxUpdateRateHz, "update_rate_hz");

    config.topics.joystick_command = params.topics.joystick_command;
    config.topics.tablet_command = params.topics.tablet_command;
    config.topics.visual_servoing_command = params.topics.visual_servoing_command;
    config.topics.mode_request = params.topics.mode_request;
    config.topics.pose_target = params.topics.pose_target;
    config.topics.ee_pose = params.topics.ee_pose;
    config.topics.ee_vel = params.topics.ee_vel;
    config.topics.ee_jac = params.topics.ee_jac;
    config.topics.joint_states = params.topics.joint_states;
    config.topics.joint_target_command = params.topics.joint_target_command;
    config.topics.output_command = params.topics.output_command;

    requireNonEmpty(config.topics.mode_request, "topics.mode_request");
    requireNonEmpty(config.topics.pose_target, "topics.pose_target");
    requireNonEmpty(config.topics.ee_pose, "topics.ee_pose");
    requireNonEmpty(config.topics.ee_vel, "topics.ee_vel");
    requireNonEmpty(config.topics.ee_jac, "topics.ee_jac");
    requireNonEmpty(config.topics.joint_states, "topics.joint_states");
    requireNonEmpty(config.topics.joint_target_command, "topics.joint_target_command");
    requireNonEmpty(config.topics.output_command, "topics.output_command");

    config.output_frame_id = params.frames.output_frame_id;
    config.default_input_frame_id = params.frames.default_input_frame_id;
    config.manager.frames.base_frame = params.frames.base_frame;
    config.manager.frames.ee_frame = params.frames.ee_frame;
    config.manager.frames.hybrid_frame = params.frames.hybrid_frame;
    requireNonEmpty(config.output_frame_id, "frames.output_frame_id");
    requireNonEmpty(config.default_input_frame_id, "frames.default_input_frame_id");
    requireNonEmpty(config.manager.frames.base_frame, "frames.base_frame");
    requireNonEmpty(config.manager.frames.ee_frame, "frames.ee_frame");
    requireNonEmpty(config.manager.frames.hybrid_frame, "frames.hybrid_frame");
    requireKnownCommandFrame(config.manager.frames, config.default_input_frame_id,
                             "frames.default_input_frame_id");

    const auto input_names = normalizedNonEmptyNames(params.inputs.sources);
    if (input_names.size() != params.inputs.sources.size())
      throw std::invalid_argument("inputs.sources must not contain empty source names");

    requireUniqueNames(input_names, "inputs.sources");

    config.manager.inputs.reserve(input_names.size());
    for (const auto &input_name : input_names)
    {
      const auto source = inputSourceFromName(input_name);
      config.manager.inputs.push_back(makeInputConfig(source, params));
      switch (source)
      {
      case manager_core::InputSource::JOYSTICK:
        requireNonEmpty(config.topics.joystick_command, "topics.joystick_command");
        break;
      case manager_core::InputSource::TABLET:
        requireNonEmpty(config.topics.tablet_command, "topics.tablet_command");
        break;
      case manager_core::InputSource::VISUAL_SERVOING:
        requireNonEmpty(config.topics.visual_servoing_command, "topics.visual_servoing_command");
        break;
      }
    }

    config.manager.jaco.min_radius = params.shapers.jaco.min_radius;
    config.manager.jaco.max_angular_velocity = params.shapers.jaco.max_angular_velocity;
    config.manager.snake.gain = params.shapers.snake.gain;
    config.manager.joint_targets = makeJointTargetConfig(params);
    config.manager.pose_targets = makePoseTargetConfig(params, config.manager.frames);
    config.manager.rate_limiter.max_linear_acceleration =
        params.rate_limiter.max_linear_acceleration;
    config.manager.rate_limiter.max_angular_acceleration =
        params.rate_limiter.max_angular_acceleration;

    requireFinite(config.manager.rate_limiter.max_linear_acceleration,
                  "rate_limiter.max_linear_acceleration");
    requireFinite(config.manager.rate_limiter.max_angular_acceleration,
                  "rate_limiter.max_angular_acceleration");
    requireNonNegative(config.manager.jaco.min_radius, "shapers.jaco.min_radius");
    requireNonNegative(config.manager.jaco.max_angular_velocity,
                       "shapers.jaco.max_angular_velocity");
    requirePositive(config.manager.snake.gain, "shapers.snake.gain");

    return config;
  }
} // namespace ros_cartesian_manager
