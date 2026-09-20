#pragma once

#include <array>

namespace ethrc::sitl
{
struct PropulsionConfig
{
  std::array<double, 3> thrustCurveN{0.0, 0.0, 1.0};
  double reactionMomentRatioM{0.0};
  double maximumRpm{0.0};
  double timeConstantUpS{0.0};
  double timeConstantDownS{0.0};
  double inputMin{0.0};
  double inputMax{1.0};
};

class RotorDynamics
{
  public: explicit RotorDynamics(const PropulsionConfig &_config = {});

  public: void Reset();

  /// Advance the normalized rotor-speed state using an exact first-order step.
  public: void Step(double _command, double _dtSeconds);

  public: double NormalizedSpeed() const;
  public: double Rpm() const;
  public: double ThrustN() const;
  /// Signed airframe yaw moment in the Betaflight mixer convention.
  public: double ReactionTorqueNm(double _turningDirection) const;

  private: PropulsionConfig config;
  private: double normalizedSpeed{0.0};
};

/// Component-wise quadratic drag: -coefficient * value * abs(value).
std::array<double, 3> QuadraticDrag(
    const std::array<double, 3> &_value,
    const std::array<double, 3> &_coefficient);
}  // namespace ethrc::sitl
