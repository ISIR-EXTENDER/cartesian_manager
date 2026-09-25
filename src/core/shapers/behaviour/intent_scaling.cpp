#include "cartesian_manager/core/shapers/behaviour/intent_scaling.hpp"

#include <algorithm>

namespace manager_core
{
  namespace
  {
    constexpr double kReleasedNorm = 1.0e-3;
  } // namespace

  IntentScaling::IntentScaling(const IntentScalingConfig &config)
  {
    configure(config);
  }

  void IntentScaling::configure(const IntentScalingConfig &config)
  {
    config_ = config;
    reset();
  }

  CartesianCommand IntentScaling::update(const CartesianCommand &input, const RobotContext &,
                                         double dt_sec)
  {
    CartesianCommand command = input;
    Eigen::Vector3d linear = input.linear;
    const double norm = linear.norm();
    if (norm > 1.0)
      linear /= norm;

    // A release or a turn back against the push starts slow again.
    if (norm < kReleasedNorm || window_sum_.dot(linear) < 0.0)
      reset();
    if (norm >= kReleasedNorm && dt_sec > 0.0)
    {
      window_.push_back({dt_sec, linear * dt_sec});
      window_sum_ += window_.back().displacement;
      window_duration_ += dt_sec;
      while (window_.size() > 1 && window_duration_ - window_.front().dt_sec >= config_.window_sec)
      {
        window_sum_ -= window_.front().displacement;
        window_duration_ -= window_.front().dt_sec;
        window_.pop_front();
      }

      // How far the last window pushed along today's direction: 1 after a full window at full scale.
      consistency_ = std::clamp(window_sum_.dot(linear.normalized()) / config_.window_sec, 0.0, 1.0);
      scale_ += config_.gain * (consistency_ - config_.consistency_threshold) * dt_sec;
      scale_ = std::clamp(scale_, config_.min_scale, 1.0);
    }

    command.linear = scale_ * linear;
    return command;
  }

  void IntentScaling::reset()
  {
    window_.clear();
    window_sum_.setZero();
    window_duration_ = 0.0;
    consistency_ = 0.0;
    scale_ = config_.min_scale;
  }

  std::string IntentScaling::name() const
  {
    return "intent_scaling";
  }

  double IntentScaling::scale() const
  {
    return scale_;
  }

  double IntentScaling::consistency() const
  {
    return consistency_;
  }

} // namespace manager_core
