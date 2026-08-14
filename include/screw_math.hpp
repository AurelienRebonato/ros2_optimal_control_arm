#pragma once
#include <Eigen/Dense>

using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix4d = Eigen::Matrix<double, 4, 4>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;


namespace screw_math 
{
    Matrix3d vecToSkew(const Vector3d& omega);
    Matrix3d matrixExp3(const Vector3d& omega, double theta);
    Matrix4d seToSE3(const Vector6d& S, double theta);
    MatrixXd adjoint(const Matrix4d& T);
    MatrixXd getSpatialInertiaMatrix(double mass, double length, double radius);
    MatrixXd ad(const Vector6d& V);
};