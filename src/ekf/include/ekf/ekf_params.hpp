#pragma once

#include <Eigen/Eigen>

namespace ekf
{
    struct Params
    {
        // Fixed aliases usable as Matrix<3,3>
        template <int rows = Eigen::Dynamic, int cols = Eigen::Dynamic>
        using Matrix = Eigen::Matrix<double, rows, cols>;

        /// Fixed aliases usable as Vector<3>
        template <int rows = Eigen::Dynamic>
        using Vector = Eigen::Matrix<rows, 1>;

        // Measurement covariances
        Vector<3> R_pos;
        Vector<3> R_att; // small-angle approx, that's why it's 3D
        Vector<3> R_acc;
        Vector<3> R_omega;

        // Process noise covariances
        Vector<3> Q_pos;
        Vector<4> Q_att;
        Vector<3> Q_vel;
        Vector<3> Q_omega;
        Vector<3> Q_acc;
        Vector<3> Q_bomega;
        Vector<3> Q_bacc;

        // Initial covariances
        Vector<3> Q_init_pos;
        Vector<4> Q_init_att;
        Vector<3> Q_init_vel;
        Vector<3> Q_init_omega;
        Vector<3> Q_init_acc;
        // Initial bias covariances
        Vector<3> Q_init_bomega;
        Vector<3> Q_init_bacc;
    };
}
