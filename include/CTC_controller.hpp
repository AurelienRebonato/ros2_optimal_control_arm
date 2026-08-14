#pragma once
#include <Eigen/Dense>
#include "arm.hpp"
#include "controller_base.hpp"


/**
 * @class CTCController
 *
 * @brief Implements a computed torque control strategy using feedback linearization.
 * 
 */
class CTCController : public ControllerBase {
    private:
        double Kp;
        double Kd;
        Arm* robot_model;
    public:

        CTCController(double stiffness, double damping, Arm* arm_model);

        /**
         * @brief From the current state and a goal state, compute the torques needed to reach the goal state.
         * 
         * @param X_current Current State vector
         * @param d_X_current current first derivative of state vector
         * @param X_goal aimed state vector
         * @param d_X_goal aimed first derivative of state vector
         * @param dd_X_goal aimed second derivative of state vector.
         * @return VectorXd 
         */
        Eigen::VectorXd getComputedTorque(
            const Eigen::VectorXd& X_current,
            const Eigen::VectorXd& d_X_current,
            const Eigen::VectorXd& X_goal,
            const Eigen::VectorXd& d_X_goal,
            const Eigen::VectorXd& dd_X_goal
        ) override;
};