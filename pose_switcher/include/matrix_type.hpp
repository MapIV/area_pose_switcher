#ifndef MATRIX_TYPE_HPP
#define MATRIX_TYPE_HPP

#include <eigen3/Eigen/Core>

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using RowMatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

#endif  // MATRIX_TYPE_HPP