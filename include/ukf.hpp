#pragma once

#include <Eigen/Dense>
#include <vector>
#include <functional>

class UnscentedKalmanFilter {
public:
    UnscentedKalmanFilter() = default;
    
    UnscentedKalmanFilter(int n_x, int n_z);

    void init(const Eigen::VectorXd& x_init, 
              const Eigen::MatrixXd& P_init, 
              const Eigen::MatrixXd& Q, 
              const Eigen::MatrixXd& R);

    void predict(std::function<Eigen::VectorXd(const Eigen::VectorXd&)> f);

    void update(const Eigen::VectorXd& z_measured);

    Eigen::VectorXd getState() const { return x_hat_; }
    bool isInitialized() const { return is_initialized_; }

private:
    int L_;
    int n_z_;
    double lambda_;
    
    Eigen::VectorXd x_hat_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    
    Eigen::MatrixXd X_sig_;

    Eigen::VectorXd weights_m_; 
    Eigen::VectorXd weights_c_;
    
    bool is_initialized_ = false;
};