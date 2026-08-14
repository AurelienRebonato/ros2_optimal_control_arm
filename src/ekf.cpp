 #include <Eigen/Dense>

 #include "ekf.hpp"

 ExtendedKalmanFilter::ExtendedKalmanFilter(int n_x, int n_z)
 {
    n_x_ = n_x;
    n_z_ = n_z;
 }

void ExtendedKalmanFilter::init(const Eigen::VectorXd& x_init,
                                const Eigen::MatrixXd& P_init,
                                const Eigen::MatrixXd& Q,
                                const Eigen::MatrixXd& R)
{
    x_hat_ = x_init;
    P_ = P_init;
    Q_ = Q;
    R_ = R;
    is_initialized_ = true;

}                                

Eigen::VectorXd ExtendedKalmanFilter::update(const Eigen::VectorXd& z_measured,
                                            const Eigen::VectorXd& x_pred,
                                            const Eigen::MatrixXd& A_jac)
{
    if (!is_initialized_) return z_measured;

    // uncertainty
    Eigen::MatrixXd P_pred = A_jac * P_ * A_jac.transpose() + Q_;

    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(n_z_, n_x_);

    Eigen::MatrixXd S = H*P_pred * H.transpose() + R_;
    Eigen::MatrixXd K = P_pred * H.transpose() * S.inverse();

    // Combine the prediction with the measure
    x_hat_ = x_pred + K * (z_measured - H * x_pred);

    P_ = (Eigen::MatrixXd::Identity(n_x_, n_x_) - K * H) * P_pred;

    return x_hat_;


}