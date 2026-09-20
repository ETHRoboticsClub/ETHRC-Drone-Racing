#include "sitl_gazebo_plugins/boreas_dynamics.hpp"

#include <algorithm>
#include <cmath>

namespace ethrc::sitl
{
RotorDynamics::RotorDynamics(const PropulsionConfig &_config)
  : config(_config)
{
}

void RotorDynamics::Reset()
{
  this->normalizedSpeed = 0.0;
}

void RotorDynamics::Step(const double _command, const double _dtSeconds)
{
  if (!std::isfinite(_command) || !std::isfinite(_dtSeconds) || _dtSeconds <= 0.0)
    return;

  const double target = std::clamp(
      _command, this->config.inputMin, this->config.inputMax);
  const double timeConstant = target >= this->normalizedSpeed ?
      this->config.timeConstantUpS : this->config.timeConstantDownS;

  if (timeConstant <= 0.0)
  {
    this->normalizedSpeed = target;
    return;
  }

  const double alpha = 1.0 - std::exp(-_dtSeconds / timeConstant);
  this->normalizedSpeed += alpha * (target - this->normalizedSpeed);
  this->normalizedSpeed = std::clamp(
      this->normalizedSpeed, this->config.inputMin, this->config.inputMax);
}

double RotorDynamics::NormalizedSpeed() const
{
  return this->normalizedSpeed;
}

double RotorDynamics::Rpm() const
{
  return this->normalizedSpeed * this->config.maximumRpm;
}

double RotorDynamics::ThrustN() const
{
  const double speed = this->normalizedSpeed;
  const double thrust = this->config.thrustCurveN[0] +
      this->config.thrustCurveN[1] * speed +
      this->config.thrustCurveN[2] * speed * speed;
  return std::max(0.0, thrust);
}

double RotorDynamics::ReactionTorqueNm(const double _turningDirection) const
{
  return _turningDirection * this->config.reactionMomentRatioM * this->ThrustN();
}

std::array<double, 3> QuadraticDrag(
    const std::array<double, 3> &_value,
    const std::array<double, 3> &_coefficient)
{
  std::array<double, 3> result{};
  for (std::size_t i = 0; i < result.size(); ++i)
    result[i] = -_coefficient[i] * _value[i] * std::abs(_value[i]);
  return result;
}
}  // namespace ethrc::sitl
