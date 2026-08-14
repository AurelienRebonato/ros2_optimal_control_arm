#pragma once

#include <Eigen/Dense>

class ExtendedKalmanFilter
{
    public:
        ExtendedKalmanFilter() = default;

        ExtendedKalmanFilter(int n_x, int n_z);

        void init(const Eigen::VectorXd& x_init,
                  const Eigen::MatrixXd& P_init,
                  const Eigen::MatrixXd& Q,
                  const Eigen::MatrixXd& R);

        Eigen::VectorXd update(const Eigen::VectorXd& z_measured,
                               const Eigen::VectorXd& x_pred,
                               const Eigen::MatrixXd& A_jac);
                            
        Eigen::VectorXd getState() const {return x_hat_;}
        bool isInitialized() const {return is_initialized_;}

    private:
            int n_x_;
            int n_z_;

            Eigen::VectorXd x_hat_;
            Eigen::MatrixXd P_;
            Eigen::MatrixXd Q_;
            Eigen::MatrixXd R_;

            bool is_initialized_ = false;

};