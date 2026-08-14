#include <cmath>
#include <vector>
#include <Eigen/Dense>

#include "arm.hpp"
#include "controller_base.hpp"
#include "CTC_controller.hpp"
#include "screw_math.hpp"

using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;
using Matrix4d = Eigen::Matrix<double, 4, 4>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Matrix6x3d = Eigen::Matrix<double, 6, 3>;


CTCController::CTCController(double stiffness,
                       double damping, 
                       Arm* arm_model)
{
    Kp = stiffness;
    Kd = damping;

    robot_model = arm_model;
}

VectorXd CTCController::getComputedTorque(
            const VectorXd& X_current,
            const VectorXd& d_X_current,
            const VectorXd& X_goal,
            const VectorXd& d_X_goal,
            const VectorXd& dd_X_goal
        )
{
    Vector3d g;
    g << 0, 0, -9.80665;

    VectorXd X_error =  X_goal - X_current;
    VectorXd d_X_error = d_X_goal - d_X_current;

    VectorXd dd_X_cmd = dd_X_goal + Kp * X_error + Kd * d_X_error;

    VectorXd coriolis_forces = robot_model->velQuadraticForces(X_current, d_X_current);
    VectorXd gravity_forces = robot_model->gravityForces(X_current, g);

    VectorXd h = coriolis_forces + gravity_forces;

    MatrixXd M = robot_model->getMassMatrix(X_current);
    // PD control
    VectorXd tau = M * dd_X_cmd + h;

    return tau;
}
