#include "ukf.hpp"
#include <iostream>

UnscentedKalmanFilter::UnscentedKalmanFilter(int n_x, int n_z) 
    : L_(n_x), n_z_(n_z) 
{
    double alpha = 1e-3;
    double beta = 2.0;
    double kappa = 0.0;
    lambda_ = alpha * alpha * (L_ + kappa) - L_;

    weights_m_ = Eigen::VectorXd(2 * L_ + 1);
    weights_c_ = Eigen::VectorXd(2 * L_ + 1);
    
    weights_m_(0) = lambda_ / (L_ + lambda_);
    weights_c_(0) = weights_m_(0) + (1.0 - alpha * alpha + beta);
    
    for (int i = 1; i < 2 * L_ + 1; i++) {
        double weight = 0.5 / (L_ + lambda_);
        weights_m_(i) = weight;
        weights_c_(i) = weight;
    }
}

void UnscentedKalmanFilter::init(const Eigen::VectorXd& x_init, 
                                 const Eigen::MatrixXd& P_init, 
                                 const Eigen::MatrixXd& Q, 
                                 const Eigen::MatrixXd& R) 
{
    x_hat_ = x_init;
    P_ = P_init;
    Q_ = Q;
    R_ = R;
    X_sig_ = Eigen::MatrixXd(L_, 2 * L_ + 1);
    is_initialized_ = true;
}

void UnscentedKalmanFilter::predict(std::function<Eigen::VectorXd(const Eigen::VectorXd&)> step_dynamics_func) 
{

    Eigen::MatrixXd P_safe = P_ + Eigen::MatrixXd::Identity(L_, L_) * 1e-6; // Tikhonov regularisation to avoid errors due to numerical rounding
    
    Eigen::LLT<Eigen::MatrixXd> lltOfP(P_safe);
    Eigen::MatrixXd L_matrix = lltOfP.matrixL();
    Eigen::MatrixXd spread = std::sqrt(L_ + lambda_) * L_matrix;

    X_sig_.col(0) = x_hat_;
    for (int i = 0; i < L_; i++) {
        X_sig_.col(i + 1)      = x_hat_ + spread.col(i);
        X_sig_.col(i + 1 + L_) = x_hat_ - spread.col(i);
    }

    Eigen::MatrixXd X_sig_pred = Eigen::MatrixXd(L_, 2 * L_ + 1);
    for (int i = 0; i < 2 * L_ + 1; i++) {
        X_sig_pred.col(i) = step_dynamics_func(X_sig_.col(i));
    }

    x_hat_.fill(0.0);
    for (int i = 0; i < 2 * L_ + 1; i++) {
        x_hat_ += weights_m_(i) * X_sig_pred.col(i);
    }

    P_.fill(0.0);
    for (int i = 0; i < 2 * L_ + 1; i++) {
        Eigen::VectorXd x_diff = X_sig_pred.col(i) - x_hat_;
        P_ += weights_c_(i) * x_diff * x_diff.transpose();
    }
    P_ += Q_;
}

void UnscentedKalmanFilter::update(const Eigen::VectorXd& z_measured) 
{
    Eigen::VectorXd z_pred = x_hat_;
    Eigen::MatrixXd S = P_ + R_;
    
    Eigen::MatrixXd T = P_; 

    Eigen::MatrixXd K = T * S.inverse();

    x_hat_ = x_hat_ + K * (z_measured - z_pred);
    P_ = P_ - K * S * K.transpose();
}