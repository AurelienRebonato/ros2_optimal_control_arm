#pragma once
#include <Eigen/Dense>
#include <vector>

class ControllerBase {
public:

    ControllerBase();
    virtual ~ControllerBase();

    virtual Eigen::VectorXd getComputedTorque(
        const Eigen::VectorXd& X_current,
        const Eigen::VectorXd& d_X_current,
        const Eigen::VectorXd& X_goal,
        const Eigen::VectorXd& d_X_goal,
        const Eigen::VectorXd& dd_X_goal)
    {
        return Eigen::VectorXd::Zero(X_current.size());
    }

    virtual Eigen::VectorXd getComputedTorque(
        const Eigen::VectorXd& X, 
        const Eigen::VectorXd& dX, 
        const std::vector<Eigen::VectorXd>& planned_q, 
        const std::vector<Eigen::VectorXd>& planned_dq, 
        const std::vector<Eigen::VectorXd>& planned_ddq) 
    {
        return getComputedTorque(X, dX, planned_q[0], planned_dq[0], planned_ddq[0]);
    }
};
