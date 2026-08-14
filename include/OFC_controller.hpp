#pragma once
#include <Eigen/Dense>
#include "arm.hpp"
#include "controller_base.hpp"
#include "ukf.hpp"

using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

/**
 * @class OFCController
 * @brief Optimal Feedback Controller integrating TV-LQI and UKF.
 * 
 * This class implements an optimal control architecture for robotic manipulators. 
 * It computes time-varying feedback gains (TV-LQR) backward in time along a planned 
 * reference trajectory. Robust state estimation is performed online using an 
 * Unscented Kalman Filter (UKF). An integral action (LQI) is included to reject 
 * unmodeled steady-state disturbances.
 */
class OFCController : public ControllerBase {
    private:
        Arm* model_;
        VectorXd x_goal;
        UnscentedKalmanFilter ukf_;

        // Cost matrices
        MatrixXd Q;
        MatrixXd R;
        MatrixXd Q_f;

        int N;
        double dt_;
        int max_iter;
        int current_t_ = 0;
        std::vector<Eigen::VectorXd> X_ref_seq_;
        std::vector<Eigen::VectorXd> U_ref_seq_;


        Eigen::VectorXd integral_error_;
        std::vector<MatrixXd> K;
        VectorXd tau_;

        VectorXd stepDynamics(const VectorXd& X, const VectorXd& U, const Vector6d& F_tip = Vector6d::Zero());

        void computeJacobians(const VectorXd& X, const VectorXd& U, MatrixXd& Ak, MatrixXd& Bk);

        double computeTrajectoryCost(const std::vector<VectorXd>& X_seq,
                                    const std::vector<VectorXd>& U_seq,
                                    const std::vector<VectorXd>& X_ref_seq,
                                    const std::vector<VectorXd>& U_ref_seq);
    public:
        OFCController(Arm* m_model,
                      const VectorXd& m_x_goal,
                      const MatrixXd& m_Q, 
                      const MatrixXd& m_R, 
                      const MatrixXd& m_Qf, 
                      int horizon, 
                      double timestep, 
                      int n_dof);
        /**
         * @brief Computes the Time-Varying LQR gains.
         * 
         * Solves the discrete-time finite-horizon Riccati equation backward in time 
         * along the reference trajectory to generate the optimal feedback gains.
         * 
         * @param x_init Initial state of the robot.
         * @param X_ref_seq Sequence of reference states to track.
         * @param U_ref_seq Sequence of reference feedforward torques.
         * @param max_iterations Unused/Legacy parameter (kept for compatibility).
         */
        void computeTVLQR(VectorXd x_init, const std::vector<VectorXd>& X_ref_seq, const std::vector<VectorXd>& U_ref_seq, int max_iterations);

        /**
         * @brief Initializes the internal reference trajectory and triggers the TV-LQR solver.
         * 
         * @param planned_q Sequence of planned joint positions.
         * @param planned_dq Sequence of planned joint velocities.
         * @param planned_ddq Sequence of planned joint accelerations.
         */
        void initTrajectory(const std::vector<Eigen::VectorXd>& planned_q,
                        const std::vector<Eigen::VectorXd>& planned_dq,
                        const std::vector<Eigen::VectorXd>& planned_ddq);
        /**
         * @brief Main control loop method to compute the optimal joint torques.
         * 
         * Estimates the true state via UKF and applies the pre-computed time-varying LQI gains.
         * Switches to a steady-state hold mode upon trajectory completion.
         * 
         * @param X Current measured joint position vector (q).
         * @param dX Current measured joint velocity vector (dq).
         * @param planned_q Sequence of planned joint positions.
         * @param planned_dq Sequence of planned joint velocities.
         * @param planned_ddq Sequence of planned joint accelerations.
         * 
         * @return Eigen::VectorXd Vector containing the optimal joint torques to be applied to the motors.
         */
        Eigen::VectorXd getComputedTorque(
                        const Eigen::VectorXd& X, 
                        const Eigen::VectorXd& dX, 
                        const std::vector<Eigen::VectorXd>& planned_q, 
                        const std::vector<Eigen::VectorXd>& planned_dq, 
                        const std::vector<Eigen::VectorXd>& planned_ddq) override;

};