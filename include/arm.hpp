#pragma once
#include <Eigen/Dense>
#include <vector>

using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix4d = Eigen::Matrix<double, 4, 4>;
using Vector3d = Eigen::Matrix<double, 3, 1>;


class Arm
{
    double link1_length;
    double link2_length;
    double link3_length;
    double link1_radius;
    double link2_radius;
    double link3_radius;
    double link1_mass;
    double link2_mass;
    double link3_mass;
    
    MatrixXd G1;
    MatrixXd G2;
    MatrixXd G3;
    std::vector<MatrixXd> G_list;

    Vector6d S1;
    Vector6d S2;
    Vector6d S3;

    std::vector<MatrixXd> S_list;

    Matrix4d M1;
    Matrix4d M2;
    Matrix4d M3;
    Matrix4d Me;

    std::vector<Matrix4d> M_list;
    public:
        Arm(
        double m_link1_length,
        double m_link1_radius,
        double m_link2_length, 
        double m_link2_radius,
        double m_link3_length,
        double m_link3_radius,
        double m_link1_mass,
        double m_link2_mass,
        double m_link3_mass);

        /**
        * @brief Get the Jacobian object
        * 
        * @param X State vector
        * @return MatrixXd 
        */
        MatrixXd getJacobian(const VectorXd& X);

        /**
        * @brief Compute the forward kinematics (the configuration of each joint)
        * 
        * @param X State vector
        * @return std::vector<Matrix4d> 
        */
        std::vector<Matrix4d> forwardKinematic(const VectorXd& X);

        /**
        * @brief Compute the inverse dynamics using the Newton-Euler Inverse Dynamics algorithm (Chap 8.3.2 Modern Robotics (Lynch & Park, 2019)) . Return the torque at each joint. 
        * 
        * @param X state vector
        * @param d_X first derivative of the state vector
        * @param dd_X second derivative of the state vector
        * @param F_tip Wrench applied at the end effector
        * @param g Vector that represents the gravity
        * @return VectorXd list of tau
        */
        VectorXd inverseDynamics(const VectorXd& X, 
                                    const VectorXd& d_X, 
                                    const VectorXd& dd_X, 
                                    const Vector6d& F_tip,
                                    const Vector3d& g);

        /**
        * @brief Get the Mass Matrix object
        * 
        * @param X State vector
        * @return MatrixXd 
        */
        MatrixXd getMassMatrix(const VectorXd& X);

        /**
        * @brief Compute the term h(theta, d_theta) of the forward dynamics equation by calling the inverse dynamics algorithm with dd_theta = 0 and F_tip = 0.
        * 
        * @param X State vector
        * @param d_X First derivative of the state vector
        * @return VectorXd 
        */
        VectorXd velQuadraticForces(const VectorXd& X, const VectorXd& d_X);

        /**
        * @brief Compute the term J*(theta).F_tip of the forward dynamics equation.
        * 
        * @param X State vector
        * @param F_tip Wrench applied to the end effector
        * @return VectorXd 
        */
        VectorXd endEffectorForces(const VectorXd& X, const Vector6d& F_tip);

        /**
        * @brief Compute the gravity part of the dynamics
        * 
        * @param X State Vector
        * @param g gravity force vector
        * @return VectorXd 
        */
        VectorXd gravityForces(const VectorXd& X, const Vector3d& g);

        /**
        * @brief Compute the forward dynamics. Return the acceleration (dd_theta) of the joints
        * 
        * @param X state vector
        * @param d_X first derivative of state vector
        * @param tau list of torques
        * @param F_tip Wrench applied at the end effector
        * @param g gravity force vector
        * @return VectorXd 
        */
        VectorXd forwardDynamics(const VectorXd& X, const VectorXd& d_X, const VectorXd& tau, const Vector6d& F_tip, const Vector3d& g);

        /**
        * @brief Get the State Derivative object. Helpers function used in the stepRK4 function
        * 
        * @param X state vector
        * @param d_X first derivative of the state vector
        * @param tau list of torques
        * @param F_tip wrench applied to the end effector
        * @param g gravity force vector
        * @return VectorXd 
        */
        VectorXd getStateDerivative(const VectorXd& X, const VectorXd& d_X, const VectorXd& tau, const Vector6d& F_tip, const Vector3d& g);

        /**
        * @brief Runge-Kutta 4 algorithm implementation used to update the state of the arm
        * 
        * @param X State Vector
        * @param d_X First derivative of state vector
        * @param tau List of torque
        * @param F_tip Wrench applied to the end effector
        * @param g Gravity force vector
        * @param dt time step
        */
        void stepRK4(VectorXd& X, VectorXd& d_X, const VectorXd& tau, const Vector6d& F_tip, const Vector3d& g, double dt);

    };