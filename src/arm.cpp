// Using the convention from Moder Robotics: Mechanics, Planning, and Control (Lynch & Park, 2019)
#include <cmath>
#include <vector>
#include <Eigen/Dense>

#include <arm.hpp>
#include <screw_math.hpp>

using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;

using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Vector3d = Eigen::Matrix<double, 3, 1>;

using Matrix4d = Eigen::Matrix<double, 4, 4>;

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6x3d = Eigen::Matrix<double, 6, 3>;


Arm::Arm(
    double m_link1_length,
    double m_link1_radius,
    double m_link2_length, 
    double m_link2_radius,
    double m_link3_length, // End effector
    double m_link3_radius,
    double m_link1_mass,
    double m_link2_mass,
    double m_link3_mass

){
    link1_length = m_link1_length;
    link2_length = m_link2_length;
    link3_length = m_link3_length;
    link1_radius = m_link1_radius;
    link2_radius = m_link2_radius;
    link3_radius = m_link3_radius;
    link1_mass = m_link1_mass;
    link2_mass = m_link2_mass;
    link3_mass = m_link3_mass;

    G1 = screw_math::getSpatialInertiaMatrix(m_link1_mass, m_link1_length, m_link1_radius);
    G2 = screw_math::getSpatialInertiaMatrix(m_link2_mass, m_link2_length, m_link2_radius);
    G3 = screw_math::getSpatialInertiaMatrix(m_link3_mass, m_link3_length, m_link3_radius);
    G_list = {G1, G2, G3};

    
    // Screw axis S = [ omega v]* expressed in the spatial frame.
    S1 << 0, 0, 1, 0, 0, 0;
    S2 << 0, 1, 0, -link1_length, 0, 0;
    S3 << 1, 0, 0, 0, link1_length + link2_length, 0;

    S_list = {S1, S2, S3}; // std::vector

    // Position of joints and end effector at home configuration
    M1 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, m_link1_length/2,
          0, 0, 0, 1;
    M2 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, m_link1_length + m_link2_length/2,
          0, 0, 0, 1;   
    M3 << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, m_link1_length + m_link2_length + m_link3_length/2,
          0, 0, 0, 1;

    Me << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, m_link1_length + m_link2_length + m_link3_length,
          0, 0, 0, 1;
    
    M_list = {M1, M2, M3, Me}; // std::vector   
}

MatrixXd Arm::getJacobian(const VectorXd& X)
{
    Matrix4d T1 = screw_math::seToSE3(S1, X(0));
    Matrix4d T12 = T1 * screw_math::seToSE3(S2, X(1)); 
    
    Vector6d J1 = S1;
    Vector6d J2 = screw_math::adjoint(T1) * S2;
    Vector6d J3 = screw_math::adjoint(T12) * S3;

    Matrix6x3d J = Matrix6x3d::Zero();
    J.col(0) = J1; 
    J.col(1) = J2;
    J.col(2) = J3;

    return J;
}

std::vector<Matrix4d> Arm::forwardKinematic(const VectorXd& X)
{
    double q1 {X(0)};
    double q2 {X(1)};
    double q3 {X(2)};

    
    Matrix4d Te = screw_math::seToSE3(S1, q1) * screw_math::seToSE3(S2, q2) * screw_math::seToSE3(S3, q3) * Me;
    
    Matrix4d T3 = screw_math::seToSE3(S1, q1) * screw_math::seToSE3(S2, q2) * screw_math::seToSE3(S3, q3) * M3;
    Matrix4d T2 = screw_math::seToSE3(S1, q1) * screw_math::seToSE3(S2, q2) * M2;
    Matrix4d T1 = screw_math::seToSE3(S1, q1) * M1;

    std::vector<Matrix4d> T {T1, T2, T3, Te};
    
    return T;
}

VectorXd Arm::inverseDynamics(const VectorXd& X, 
                         const VectorXd& d_X, 
                         const VectorXd& dd_X, 
                         const Vector6d& F_tip,
                         const Vector3d& g)
{
    // Newton-Euler inverse dynamics algorithm 
    int n_joints {3}; // Number of joints

    Matrix4d M0 = Matrix4d::Identity();
    std::vector<Matrix4d> M_list_full = {M0, M1, M2, M3, Me};

    std::vector<Matrix4d> T_list(n_joints + 1, Matrix4d::Identity());
    std::vector<Vector6d> V_list(n_joints + 1, Vector6d::Zero());
    std::vector<Vector6d> dV_list(n_joints + 1, Vector6d::Zero());
    std::vector<Vector6d> A_list(n_joints + 1, Vector6d::Zero());

    // initial conditions
    Vector6d V0 = Vector6d::Zero();
    V_list[0] = V0;

    Vector6d dV0 = Vector6d::Zero();
    dV0.tail<3>() = -g;
    dV_list[0] = dV0;

    std::vector<Vector6d> F_list(n_joints + 2, Vector6d::Zero());
    F_list[n_joints + 1] = F_tip;

    VectorXd tau_list = VectorXd::Zero(n_joints);

    
    for (int i = 1; i <= n_joints; ++i) {
        A_list[i] = screw_math::adjoint(M_list_full[i].inverse()) * S_list[i-1];
    }
    // Forward iterations
    for (int i {1}; i <= n_joints; ++i)
    {            
        Matrix4d M_i_iminus1 = M_list_full[i].inverse() * M_list_full[i-1];
        Matrix4d T_i_iminus1 = screw_math::seToSE3(-A_list[i], X(i-1)) * M_i_iminus1;
        T_list[i] = T_i_iminus1; 

        MatrixXd Ad_T = screw_math::adjoint(T_i_iminus1);

        V_list[i] = Ad_T * V_list[i-1] + A_list[i] * d_X(i-1);

        MatrixXd adv = screw_math::ad(V_list[i]);
        dV_list[i] = Ad_T * dV_list[i-1] + adv * (A_list[i] * d_X(i-1)) + A_list[i] * dd_X(i-1);
    }
    
    Matrix4d M_e_3 = Me.inverse() * M3;
    T_list[n_joints + 1] = M_e_3;

    // Backward iterations

    for (int i = n_joints; i > 0; --i)
    {
        Matrix4d T_next_i = T_list[i+1];

        MatrixXd Ad_trans = screw_math::adjoint(T_next_i).transpose();
        MatrixXd adv_trans = screw_math::ad(V_list[i]).transpose();

        MatrixXd G_i = G_list[i-1];

        Vector6d F = Ad_trans * F_list[i+1] 
                   + G_i * dV_list[i] 
                   - adv_trans * (G_i * V_list[i]);

        F_list[i] = F;
        tau_list(i-1) = F.dot(A_list[i]);
        
    }

    double damping_coeff = 0.5; 
    for (int i = 0; i < n_joints; ++i) {
        tau_list(i) += damping_coeff * d_X(i);
    }
    return tau_list; 
}

MatrixXd Arm::getMassMatrix(const VectorXd& X)
{
    int n_joints = 3;
    MatrixXd M = MatrixXd::Zero(n_joints, n_joints);
    
    VectorXd d_X_zero = VectorXd::Zero(n_joints);
    Vector3d g_zero = Vector3d::Zero();
    Vector6d F_tip_zero = Vector6d::Zero();

    for (int i = 0; i < n_joints; ++i)
    {
        VectorXd dd_X_probe = VectorXd::Zero(n_joints);
        dd_X_probe(i) = 1.0;
        
        M.col(i) = inverseDynamics(X, d_X_zero, dd_X_probe, F_tip_zero, g_zero);
    }
    
    return M;
}

VectorXd Arm::velQuadraticForces(const VectorXd& X, const VectorXd& d_X)
{
    VectorXd dd_X_zero = VectorXd::Zero(3);
    Vector3d g_zero = Vector3d::Zero();
    Vector6d F_tip_zero = Vector6d::Zero();
    
    return inverseDynamics(X, d_X, dd_X_zero, F_tip_zero, g_zero);
}

VectorXd Arm::endEffectorForces(const VectorXd& X, const Vector6d& F_tip)
{
    VectorXd d_X_zero = VectorXd::Zero(3);
    VectorXd dd_X_zero = VectorXd::Zero(3);
    Vector3d g_zero = Vector3d::Zero();
    
    return inverseDynamics(X, d_X_zero, dd_X_zero, F_tip, g_zero);
}

VectorXd Arm::gravityForces(const VectorXd& X, const Vector3d& g)
{
    VectorXd d_X_zero = VectorXd::Zero(3);
    VectorXd dd_X_zero = VectorXd::Zero(3);
    Vector6d F_tip_zero = Vector6d::Zero();
    
    return inverseDynamics(X, d_X_zero, dd_X_zero, F_tip_zero, g);
}

VectorXd Arm::forwardDynamics(const VectorXd& X, const VectorXd& d_X, const VectorXd& tau, const Vector6d& F_tip, const Vector3d& g)
{
    MatrixXd M = getMassMatrix(X);
    VectorXd c = velQuadraticForces(X, d_X);
    VectorXd gravity = gravityForces(X, g);
    VectorXd JTFtip = endEffectorForces(X, F_tip);
    
    VectorXd b = tau - c - gravity - JTFtip;
    
    VectorXd dd_X = M.ldlt().solve(b); // LDLT decomposition
    return dd_X;
}

VectorXd Arm::getStateDerivative(const VectorXd& X, const VectorXd& d_X, const VectorXd& tau, const Vector6d& F_tip, const Vector3d& g)
{
    VectorXd dd_X = forwardDynamics(X, d_X, tau, F_tip, g);
    Vector6d derivative;
    derivative << d_X, dd_X;

    return derivative;
}

void Arm::stepRK4(VectorXd& X, VectorXd& d_X, const VectorXd& tau, const Vector6d& F_tip, const Vector3d& g, double dt)
{

    VectorXd Y_current(6);
    Y_current << X, d_X;
    // k1
    VectorXd k1 = getStateDerivative(X, d_X, tau, F_tip, g);
    // k2
    VectorXd Y2 = Y_current + 0.5 * dt * k1;
    VectorXd k2 = getStateDerivative(Y2.head(3), Y2.tail(3), tau, F_tip, g);
    // k3
    VectorXd Y3 = Y_current + 0.5 * dt * k2;
    VectorXd k3 = getStateDerivative(Y3.head(3), Y3.tail(3), tau, F_tip, g);
    // k4
    VectorXd Y4 = Y_current + dt * k3;
    VectorXd k4 = getStateDerivative(Y4.head(3), Y4.tail(3), tau, F_tip, g);

    VectorXd Y_next = Y_current + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);

    X = Y_next.head(3);
    d_X = Y_next.tail(3);
}

