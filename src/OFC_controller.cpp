#include <cmath>
#include <Eigen/Dense>
#include <random>
#include <vector>

#include "controller_base.hpp"
#include "OFC_controller.hpp"
#include "arm.hpp"
#include "ukf.hpp"


using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;

using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Vector3d = Eigen::Matrix<double, 3, 1>;

using Matrix4d = Eigen::Matrix<double, 4, 4>;

using Vector6d = Eigen::Matrix<double, 6, 1>;


VectorXd OFCController::stepDynamics(const VectorXd& X, const VectorXd& U, const Vector6d& F_tip)
{
    // X is the extended state [q, dq]*
    int n_dof = X.size() / 2;
    VectorXd q = X.head(n_dof);
    VectorXd dq = X.tail(n_dof);
    Vector3d g;
    g << 0, 0, -9.80665;
    
    model_->stepRK4(q, dq, U, F_tip, g, dt_);
    
    VectorXd X_next = VectorXd::Zero(X.size());
    X_next << q, dq;
    
    return X_next;
}
void OFCController::computeJacobians(const VectorXd& X, const VectorXd& U, MatrixXd& Ak, MatrixXd& Bk)
{
    int n_x = X.size();
    int n_u = U.size();
    double epsilon = 1e-5;
    Ak.resize(n_x, n_x);
    Bk.resize(n_x, n_u);
    VectorXd X_ref = stepDynamics(X, U);
    for (int i = 0; i < n_x; i++)
    {
        VectorXd X_pert = X;
        X_pert(i) += epsilon;
        VectorXd X_next_pert = stepDynamics(X_pert, U);
        Ak.col(i) = (X_next_pert - X_ref) / epsilon;
    }
    for (int i = 0; i < n_u; i++)
    {
        VectorXd U_pert = U;
        U_pert(i) += epsilon;
        VectorXd X_next_pert = stepDynamics(X, U_pert);
        Bk.col(i) = (X_next_pert - X_ref) /epsilon;
    }
}
double OFCController::computeTrajectoryCost(const std::vector<VectorXd>& X_seq,
                                            const std::vector<VectorXd>& U_seq,
                                            const std::vector<VectorXd>& X_ref_seq,
                                            const std::vector<VectorXd>& U_ref_seq)
{
    double cost = 0.0;
    for (int i = 0; i < N; i++)
    {
        VectorXd state_error = X_seq[i] - X_ref_seq[i];
        VectorXd command_error = U_seq[i] - U_ref_seq[i];

        cost += 0.5 * (state_error.transpose() * Q * state_error)(0,0);
        cost += 0.5 * (command_error.transpose() * R * command_error)(0,0);
    }
    VectorXd diff = X_seq[N] - X_ref_seq[N];
    cost += 0.5 * (diff.transpose() * Q_f * diff)(0, 0);
    return cost;
}

OFCController::OFCController(Arm* m_model,
              const VectorXd& m_x_goal,
              const MatrixXd& m_Q, 
              const MatrixXd& m_R, 
              const MatrixXd& m_Qf, 
              int horizon, 
              double timestep, 
              int n_dof)
{
    model_ = m_model;
    x_goal = m_x_goal;
    Q = m_Q;
    R = m_R;
    Q_f = m_Qf;
    N = horizon;
    dt_ = timestep;
    max_iter = 100;

    tau_ = Eigen::VectorXd::Zero(n_dof);
    current_t_ = 0;
    integral_error_ = Eigen::VectorXd::Zero(n_dof);

    ukf_ = UnscentedKalmanFilter(2 * n_dof, 2 * n_dof);
    
    int n_x_aug = 3 * n_dof; 
    int n_u = n_dof;
    K.assign(N, MatrixXd::Zero(n_u, n_x_aug));
}
void OFCController::computeTVLQR(VectorXd x_init, 
                                const std::vector<VectorXd>& X_ref_seq,
                                const std::vector<VectorXd>& U_ref_seq,
                                int max_iterations)
{
    int n_dof = U_ref_seq[0].size();
    MatrixXd S_next = Q_f;
    
    for (int t = N - 1; t >= 0 ; t--)
    {
        MatrixXd Ak_orig, Bk_orig;
        computeJacobians(X_ref_seq[t], U_ref_seq[t], Ak_orig, Bk_orig);

        MatrixXd Ak = MatrixXd::Zero(3 * n_dof, 3 * n_dof);
        MatrixXd Bk = MatrixXd::Zero(3 * n_dof, n_dof);

        // [ A_orig |  0 ]
        // [ dt * I |  I ]
        Ak.topLeftCorner(2 * n_dof, 2 * n_dof) = Ak_orig;
        Ak.bottomLeftCorner(n_dof, n_dof) = MatrixXd::Identity(n_dof, n_dof) * dt_;
        Ak.bottomRightCorner(n_dof, n_dof) = MatrixXd::Identity(n_dof, n_dof);
        
        // [ B_orig ]
        // [   0    ]
        Bk.topRows(2 * n_dof) = Bk_orig;

        MatrixXd term = Bk.transpose() * S_next * Bk + R;
        term += Eigen::MatrixXd::Identity(term.rows(), term.cols()) * 1e-4; 
        MatrixXd term_inv = term.inverse();

        K[t] = term_inv * Bk.transpose() * S_next * Ak;

        MatrixXd Ak_minus_BkK = Ak - Bk * K[t];
        S_next = Ak.transpose() * S_next * Ak_minus_BkK + Q;
    }
}

void OFCController::initTrajectory(
    const std::vector<VectorXd>& planned_q,
    const std::vector<VectorXd>& planned_dq,
    const std::vector<VectorXd>& planned_ddq
)
{
    int n_dof = planned_q[0].size();
    X_ref_seq_.assign(N + 1, VectorXd::Zero(2 * n_dof));
    U_ref_seq_.assign(N, VectorXd::Zero(n_dof));

    Vector3d g;
    g << 0, 0, -9.80665;
    Vector6d F_tip_zero = Vector6d::Zero();

    for (int t = 0; t <= N; t++) {
        X_ref_seq_[t] << planned_q[t], planned_dq[t];
        if (t < N) {
            U_ref_seq_[t] = model_->inverseDynamics(planned_q[t], planned_dq[t], planned_ddq[t], F_tip_zero, g);
        }
    }
    computeTVLQR(X_ref_seq_[0], X_ref_seq_, U_ref_seq_, 1);
    current_t_ = 0;
}

VectorXd OFCController::getComputedTorque(
    const VectorXd& X_current,
    const VectorXd& d_X_current,
    const std::vector<VectorXd>& planned_q,
    const std::vector<VectorXd>& planned_dq,
    const std::vector<VectorXd>& planned_ddq
)
{
    int n_dof = X_current.size();
    Vector6d F_tip_zero = Vector6d::Zero();
    Vector3d g;
    g << 0, 0, -9.80665;

    //VectorXd current_state(2 * n_dof);
    //current_state << X_current, d_X_current;

    // Filter the input with Extended Kalman filter
    VectorXd z_measured(2*n_dof);
    z_measured << X_current, d_X_current;

    if (!ukf_.isInitialized()){
        MatrixXd P_init = MatrixXd::Identity(2* n_dof, 2*n_dof) * 0.1;

        MatrixXd Q_ukf = MatrixXd::Identity(2 * n_dof, 2 * n_dof);
        Q_ukf.block(0, 0, n_dof, n_dof) *= 1e-4; //high precision on predicted position
        Q_ukf.block(n_dof, n_dof, n_dof, n_dof) *= 1e-3; // high precision on predicted speed

        MatrixXd R_ukf = MatrixXd::Identity(2 * n_dof, 2 * n_dof);
        R_ukf.block(0, 0, n_dof, n_dof) *= 1e-4;   // high reliability of measured position
        R_ukf.block(n_dof, n_dof, n_dof, n_dof) *= 1.0; // low reliability of measured velocity

        ukf_.init(z_measured, P_init, Q_ukf, R_ukf);
    }

    
    auto f_dynamics = [&](const VectorXd& state) {
        return stepDynamics(state, tau_, F_tip_zero);
    };

    ukf_.predict(f_dynamics);
    ukf_.update(z_measured);
    
    VectorXd current_state = ukf_.getState();
    
    // Compute the command (TV-LQI)
    
    int t = std::min(current_t_, N-1);

    VectorXd current_X_ref = X_ref_seq_[t];
    VectorXd current_U_ref = U_ref_seq_[t];
    MatrixXd current_K = K[t];

    if (current_t_ >=N)
    {
        current_X_ref = X_ref_seq_[N];

        VectorXd q_target = X_ref_seq_[N].head(n_dof);
        VectorXd zero_dyn = VectorXd::Zero(n_dof);

        current_U_ref = model_->inverseDynamics(q_target, zero_dyn, zero_dyn, F_tip_zero, g);
        current_K = K[0];
    }

    VectorXd physical_error = current_state - X_ref_seq_[t];

    VectorXd q_error = physical_error.head(n_dof);

    double deadband_threshold = 1e-3;

    for (int i = 0; i < n_dof; ++i)
    {
        if (std::abs(q_error(i)) < deadband_threshold)
        {
            q_error(i) = 0.0;
            integral_error_(i) *= 0.5;
        }

    }

    integral_error_ = 0.99 * integral_error_ + q_error * dt_;
    for(int i = 0; i < n_dof; i++){
        if(integral_error_(i) > 0.3) integral_error_(i) = 0.3;
        if(integral_error_(i) < -0.3) integral_error_(i) = -0.3;
    }

    VectorXd X_aug_error(3 * n_dof);
    X_aug_error << physical_error, integral_error_;

    VectorXd tau_raw = current_U_ref - current_K * X_aug_error;    
    
    tau_ = tau_raw;


    for (int i = 0; i < tau_.size(); ++i) {
        if (std::isnan(tau_(i))) tau_(i) = 0.0;
        if (tau_(i) > 500.0) tau_(i) = 500.0;
        if (tau_(i) < -500.0) tau_(i) = -500.0;
    }

    current_t_++;

    return tau_;
}
