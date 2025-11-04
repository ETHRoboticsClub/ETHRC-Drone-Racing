#include "ekf/ekf.hpp"

namespace ekf
{
    Ekf::Ekf(const Params& params)
        : params_(params)
    {
      // Initialize state and covariance
      state_.setZero();
      P_.setZero();
    }
}