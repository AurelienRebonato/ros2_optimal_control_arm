#pragma once

#include <Eigen/Dense>
#include <vector>

#include "arm.hpp"
#include "RRT_planner.hpp"


struct SplineSegment {
    double duration;
    double start_time;
    Eigen::MatrixXd coeffs;
};

class TrajectoryGenerator {
public:
    TrajectoryGenerator(Arm* model, 
                        RRTPlanner* planner,
                        const std::vector<double>& joints_lower_limits,
                        const std::vector<double>& joints_upper_limits,
                        int n_dof = 3);

    bool planTrajectory(const Eigen::VectorXd& q_start, 
                        const Eigen::VectorXd& q_goal, /*
                        const std::vector<Obstacle>& obstacles,*/
                        double total_duration);

    Eigen::VectorXd getPosition(double t) const;
    Eigen::VectorXd getVelocity(double t) const;
    Eigen::VectorXd getAcceleration(double t) const;

private:
    int n_dof_;
    double duration_;
    double t_start_;
    Arm* model_;
    RRTPlanner* planner_;
    std::vector<SplineSegment> splines_;
    std::vector<double> joints_lower_limits_;
    std::vector<double> joints_upper_limits_;

    /**
     * @brief This function use the artificial potential forces framework (Khatib, 1986) to
     to simulate the movement computed by the RRT planner and smooth it.
     * 
     * @param rrt_path Path computed by the RRT algorithm
     * @param obstacles 
     * @return std::vector<Eigen::VectorXd> 
     */
    std::vector<Eigen::VectorXd> refinePathWithAPF(const std::vector<Eigen::VectorXd>& rrt_path/*, 
                                                   const std::vector<Obstacle>& obstacles*/);

    void computePolynomialCoefficients(const std::vector<Eigen::VectorXd>& refined_path, double total_duration);

};