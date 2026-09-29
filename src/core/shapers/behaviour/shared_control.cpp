#include "cartesian_manager/core/shapers/behaviour/shared_control.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <utility>

namespace manager_core
{
  namespace
  {
    constexpr double kReleasedNorm = 1.0e-3;
    constexpr double kVerticalWeight = 0.2;   // W = diag(1, 1, 0.2) damps vertical intent
    constexpr double kMinIntentSpeedRatio = 0.2; // below this fraction of v_j_max confidences freeze
    constexpr const char *kAgnosticGoalId = "agnostic";

    std::pair<double, Eigen::Vector3d> distanceAndDirection(const Eigen::Vector3d &goal,
                                                            const Eigen::Vector3d &position)
    {
      const Eigen::Vector3d delta = goal - position;
      const double distance = delta.norm();
      return {distance, distance > 1.0e-9 ? Eigen::Vector3d(delta / distance)
                                          : Eigen::Vector3d::UnitX()};
    }

    // Shortest rotation from current to goal as angle in [0, pi] and unit axis in the base frame.
    std::pair<double, Eigen::Vector3d> rotationError(const Eigen::Quaterniond &goal,
                                                     const Eigen::Quaterniond &current)
    {
      Eigen::Quaterniond error = goal * current.conjugate();
      if (error.w() < 0.0)
        error.coeffs() *= -1.0;
      const Eigen::AngleAxisd angle_axis(error);
      if (angle_axis.angle() <= 0.0)
        return {0.0, Eigen::Vector3d::Zero()};
      return {angle_axis.angle(), angle_axis.axis()};
    }

    Eigen::Quaterniond normalized(Eigen::Quaterniond quaternion)
    {
      if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1.0e-9)
        return Eigen::Quaterniond::Identity();
      quaternion.normalize();
      return quaternion;
    }
  } // namespace

  template <typename Function> void SharedControl::forEachGoal(Function function)
  {
    for (auto &goal : static_goals_)
      function(goal);
    for (auto &goal : dynamic_goals_)
      function(goal);
  }

  template <typename Function> void SharedControl::forEachGoal(Function function) const
  {
    for (const auto &goal : static_goals_)
      function(goal);
    for (const auto &goal : dynamic_goals_)
      function(goal);
  }

  SharedControl::SharedControl(const SharedControlConfig &config)
  {
    configure(config);
  }

  void SharedControl::configure(const SharedControlConfig &config)
  {
    std::vector<SharedControlGoal> goals = config.goals;
    for (auto &goal : goals)
    {
      goal.orientation = normalized(goal.orientation);
      goal.confidence = 0.0;
      for (const auto &previous : static_goals_)
        if (previous.id == goal.id)
          goal.confidence = previous.confidence;
    }
    config_ = config;
    config_.goals.clear();
    static_goals_ = std::move(goals);
  }

  CartesianCommand SharedControl::update(const CartesianCommand &input,
                                         const RobotContext &context, double dt_sec)
  {
    const auto &scale = context.command_scale;
    if (context.ee_pose.frame_id.empty() || scale.linear <= 0.0 || scale.angular <= 0.0)
      return input;

    CartesianPose ee_pose = context.ee_pose;
    ee_pose.orientation = normalized(ee_pose.orientation);
    const Twist si{input.linear * config_.input_scale * scale.linear,
                   input.angular * config_.input_scale * scale.angular};

    Twist shaped;
    if (input.linear.norm() >= kReleasedNorm)
    {
      updateConfidences(si.linear, ee_pose.position, dt_sec);
      shaped = translationMode(si, softGoal(ee_pose), ee_pose);
    }
    else if (input.angular.norm() >= kReleasedNorm)
    {
      shaped = rotationMode(si, softGoal(ee_pose), ee_pose);
    }
    else
    {
      return input;
    }

    CartesianCommand output = input;
    output.linear = shaped.linear / scale.linear;
    output.angular = shaped.angular / scale.angular;
    return output;
  }

  void SharedControl::reset()
  {
    dynamic_goals_.clear();
    for (auto &goal : static_goals_)
      goal.confidence = 0.0;
  }

  std::string SharedControl::name() const
  {
    return "shared_control";
  }

  void SharedControl::setGoals(const std::vector<SharedControlGoal> &goals)
  {
    std::vector<SharedControlGoal> previous = std::move(dynamic_goals_);
    std::vector<bool> taken(previous.size(), false);
    dynamic_goals_.clear();
    for (const auto &incoming : goals)
    {
      if (!incoming.position.allFinite())
        continue;
      auto goal = incoming;
      goal.orientation = normalized(goal.orientation);
      goal.confidence = 0.0;

      double best_distance = config_.goal_match_distance;
      std::size_t best = previous.size();
      for (std::size_t index = 0; index < previous.size(); ++index)
      {
        const double distance = (previous[index].position - goal.position).norm();
        if (!taken[index] && distance <= best_distance)
        {
          best_distance = distance;
          best = index;
        }
      }
      if (best < previous.size())
      {
        taken[best] = true;
        goal.confidence = previous[best].confidence;
      }
      dynamic_goals_.push_back(std::move(goal));
    }
  }

  std::vector<SharedControlGoal> SharedControl::goals() const
  {
    std::vector<SharedControlGoal> goals = static_goals_;
    goals.insert(goals.end(), dynamic_goals_.begin(), dynamic_goals_.end());
    return goals;
  }

  SharedControlState SharedControl::state(const RobotContext &context) const
  {
    SharedControlState state;
    state.goal_ids.push_back(kAgnosticGoalId);
    state.confidences.push_back(agnosticConfidence());
    forEachGoal([&state](const SharedControlGoal &goal) {
      state.goal_ids.push_back(goal.id);
      state.confidences.push_back(goal.confidence);
    });
    CartesianPose ee_pose = context.ee_pose;
    ee_pose.orientation = normalized(ee_pose.orientation);
    state.soft_goal = softGoal(ee_pose);
    return state;
  }

  // Algorithm 1: integrate the confidence of each goal the weighted velocity points at.
  void SharedControl::updateConfidences(const Eigen::Vector3d &velocity,
                                        const Eigen::Vector3d &position, double dt_sec)
  {
    if (dt_sec <= 0.0 || config_.v_j_max <= 0.0)
      return;

    const double speed_scale = std::min(1.0, velocity.norm() / config_.v_j_max);
    if (speed_scale < kMinIntentSpeedRatio)
      return;

    const double r_freeze = std::min(config_.r1, config_.r2);
    bool frozen = false;
    forEachGoal([&](const SharedControlGoal &goal) {
      frozen = frozen || distanceAndDirection(goal.position, position).first < r_freeze;
    });
    if (frozen)
      return;

    const double cos_theta = std::cos(config_.theta_l);
    const double cone_span = std::max(1.0 - cos_theta, 1.0e-6);
    Eigen::Vector3d weighted_velocity = velocity;
    weighted_velocity.z() *= kVerticalWeight;

    forEachGoal([&](SharedControlGoal &goal) {
      Eigen::Vector3d weighted_direction = distanceAndDirection(goal.position, position).second;
      weighted_direction.z() *= kVerticalWeight;
      double cos_phi = 0.0;
      const double norms = weighted_direction.norm() * weighted_velocity.norm();
      if (norms > 0.0)
        cos_phi = std::clamp(weighted_direction.dot(weighted_velocity) / norms, -1.0, 1.0);

      const double confidence_rate =
          config_.alpha_conf * speed_scale * (cos_phi - cos_theta) / cone_span;
      goal.confidence = std::clamp(goal.confidence + confidence_rate * dt_sec, 0.0, 1.0);
    });
  }

  double SharedControl::agnosticConfidence() const
  {
    double sum = 0.0;
    forEachGoal([&sum](const SharedControlGoal &goal) { sum += goal.confidence; });
    return std::max(0.0, 1.0 - sum);
  }

  // Confidence-weighted position and Markley quaternion average, the agnostic goal being the current pose.
  CartesianPose SharedControl::softGoal(const CartesianPose &ee_pose) const
  {
    double weight_sum = agnosticConfidence();
    Eigen::Vector3d position_sum = weight_sum * ee_pose.position;
    const auto accumulate = [](Eigen::Matrix4d &sum, const Eigen::Quaterniond &q, double weight) {
      const Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());
      sum += weight * v * v.transpose();
    };
    Eigen::Matrix4d quaternion_sum = Eigen::Matrix4d::Zero();
    accumulate(quaternion_sum, ee_pose.orientation, weight_sum);

    forEachGoal([&](const SharedControlGoal &goal) {
      if (goal.confidence <= 0.0)
        return;
      weight_sum += goal.confidence;
      position_sum += goal.confidence * goal.position;
      accumulate(quaternion_sum, goal.orientation, goal.confidence);
    });

    CartesianPose soft_goal = ee_pose;
    if (weight_sum <= 0.0)
      return soft_goal;

    soft_goal.position = position_sum / weight_sum;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(quaternion_sum);
    if (solver.info() == Eigen::Success)
    {
      const Eigen::Vector4d q = solver.eigenvectors().col(3);
      soft_goal.orientation = normalized(Eigen::Quaterniond(q(0), q(1), q(2), q(3)));
    }
    return soft_goal;
  }

  double SharedControl::sigmaD(double distance) const
  {
    const double r_near = std::min(config_.r1, config_.r2);
    const double r_far = std::max(config_.r1, config_.r2);
    if (distance <= r_near)
      return 0.0;
    if (distance >= r_far)
      return 1.0;
    return (distance - r_near) / (r_far - r_near);
  }

  double SharedControl::sigmaR(double angle) const
  {
    if (config_.theta1 <= config_.theta2 || angle <= config_.theta2)
      return 0.0;
    if (angle >= config_.theta1)
      return 1.0;
    return (angle - config_.theta2) / (config_.theta1 - config_.theta2);
  }

  // Translation mode: amplify the goal-aligned velocity and turn the wrist towards the goal orientation
  // so it is reached with the position. The incoming angular command is the baseline (jaco's, or none).
  SharedControl::Twist SharedControl::translationMode(const Twist &input, const CartesianPose &goal,
                                                      const CartesianPose &ee_pose) const
  {
    const auto [distance, direction] = distanceAndDirection(goal.position, ee_pose.position);
    const double sigma = sigmaD(distance);

    Twist output;
    output.linear =
        input.linear + sigma * (config_.gamma - 1.0) * input.linear.dot(direction) * direction;
    output.angular = input.angular;
    if (sigma <= 0.0)
      return output;

    const auto [angle, axis] = rotationError(goal.orientation, ee_pose.orientation);
    const double r_near = std::min(config_.r1, config_.r2);
    const double rate = output.linear.norm() * angle / (distance - r_near);
    output.angular = (1.0 - sigma) * input.angular + sigma * rate * axis;
    return output;
  }

  // Rotation mode: amplify rotation towards the goal orientation and pivot about the goal position.
  SharedControl::Twist SharedControl::rotationMode(const Twist &input, const CartesianPose &goal,
                                                   const CartesianPose &ee_pose) const
  {
    const auto [angle, axis] = rotationError(goal.orientation, ee_pose.orientation);

    Twist output;
    output.angular =
        input.angular + config_.gamma * sigmaR(angle) * input.angular.dot(axis) * axis;
    output.linear = (goal.position - ee_pose.position).cross(output.angular);
    return output;
  }

} // namespace manager_core
