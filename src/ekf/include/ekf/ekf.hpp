#pragma once

#include <memory>

#include <ekf/ekf_params.hpp>

namespace ekf
{

  class Ekf
  {
  public:
    Ekf(const Params& params);
    ~Ekf();

  private:
    std::shared_ptr<Params> params_;
    Eigen::SparseMatrix<double> Q_;
    Eigen::SparseMatrix<double> P_;
  };

}