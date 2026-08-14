#include <cmath>
#include <Eigen/Dense>

#include "screw_math.hpp"

using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix4d = Eigen::Matrix<double, 4, 4>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;

namespace screw_math
{
    Matrix3d vecToSkew(const Vector3d& omega)
    {
        Matrix3d M;

        M << 0, -omega(2), omega(1),
            omega(2), 0, -omega(0),
            -omega(1), omega(0), 0;
        return M;
    }

    Matrix3d matrixExp3(const Vector3d& omega, double theta)
    {
        Matrix3d I = Matrix3d::Identity();
        Matrix3d omg_mat = vecToSkew(omega);

        // Rodrigues' formula
        Matrix3d R = I + std::sin(theta)*omg_mat + (1 - std::cos(theta)) * (omg_mat*omg_mat);

        return R;

    }

    Matrix4d seToSE3(const Vector6d& S, double d_theta)
    {
        Vector3d omega;
        omega << S(0), S(1), S(2);
        Vector3d v;
        v << S(3), S(4), S(5);

        Matrix3d R = matrixExp3(omega, d_theta);
        Matrix3d omg_mat = vecToSkew(omega);

        
        Matrix3d I = Matrix3d::Identity();
        // According to equation 3.87 (Lynch & Park, 2019)
        Matrix3d G = I * d_theta + (1.0 - std::cos(d_theta)) * omg_mat + (d_theta - std::sin(d_theta)) * (omg_mat * omg_mat);
        Vector3d p = G * v;

        Matrix4d T = Matrix4d::Identity();
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = p;

        return T;
    }

    MatrixXd adjoint(const Matrix4d& T)
    {
        

        Matrix3d R = T.block<3, 3>(0, 0);
        Vector3d p = T.block<3, 1>(0, 3);

        Matrix3d p_mat = vecToSkew(p);

        MatrixXd ad = MatrixXd::Zero(6, 6);
        ad.block<3, 3>(0, 0) = R;
        ad.block<3, 3>(3, 0) = p_mat * R;
        ad.block<3, 3>(3, 3) = R;

        return ad;

    }

    MatrixXd getSpatialInertiaMatrix(double mass, double length, double radius)
    {
        // The robot's links are supposed to be cylinder
        double Ixx_yy = (mass / 12.0) * (3.0 * radius * radius + length * length);
        double Izz = (mass / 2.0) * (radius * radius);
        Matrix3d Ib;
            Ib << Ixx_yy, 0, 0,
                0, Ixx_yy, 0,
                0, 0, Izz;

        Matrix3d I = Matrix3d::Identity();
        
        MatrixXd G = MatrixXd::Zero(6, 6);
        G.block<3, 3> (0,0) = Ib;
        G.block<3, 3> (3,3) = mass*I;

        return G;

    }

    MatrixXd ad(const Vector6d& V)
    {
        Vector3d omega = V.block<3,1> (0,0);
        Vector3d v = V.block<3,1> (3,0);

        MatrixXd ad = MatrixXd::Zero(6, 6);
        ad.block<3,3>(0,0) = vecToSkew(omega);
        ad.block<3,3>(3,3) = vecToSkew(omega);
        ad.block<3,3>(3,0) = vecToSkew(v);
        return ad;
    }
}