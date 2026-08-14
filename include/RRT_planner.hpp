#pragma once
#include <Eigen/Dense>
#include <random>
#include <vector>


struct RRTNode {
    Eigen::VectorXd q;
    int parent_index;
};

struct Obstacle {
    Eigen::Vector3d position;
    double radius;
};

class RRTPlanner {
public:
    RRTPlanner(std::vector<double> joints_lower_bounds,
               std::vector<double> joints_upper_bounds,
               int n_dof = 3);
    
    /**
     * @brief Compute the path using the Rapidly-exploring Random Tree algorithm.
     The obstacle handling has not been implemented yet.
     * 
     */
    std::vector<Eigen::VectorXd> computeRRT(const Eigen::VectorXd& start, 
                                   const Eigen::VectorXd& goal/*, 
                                   const std::vector<Obstacle>& obstacles*/);
private:
    int n_dof_;
    std::vector<double> joints_lower_limits_;
    std::vector<double> joints_upper_limits_;
    
    //
    std::mt19937 gen_;
    std::uniform_real_distribution<double> bias_dist_;
    std::vector<std::uniform_real_distribution<double>> joint_dists_;

    // helpers functions
    Eigen::VectorXd sampleRandom(const Eigen::VectorXd& goal_config, double goal_bias);
    int getNearestNodeIndex(const std::vector<RRTNode>& tree, const Eigen::VectorXd& random_q);
    Eigen::VectorXd steer(const Eigen::VectorXd& q_near, const Eigen::VectorXd& q_rand, double max_step_size);
    bool checkCollision(const Eigen::VectorXd& q, const std::vector<Obstacle>& obstacles);
};