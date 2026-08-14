#include <cmath>
#include <random>
#include <vector>
#include <Eigen/Dense>

#include "RRT_planner.hpp"

using VectorXd = Eigen::VectorXd;

RRTPlanner::RRTPlanner(std::vector<double> joints_lower_limits,
                       std::vector<double> joints_upper_limits,
                       int n_dof)
{
    n_dof_ = n_dof;
    joints_lower_limits_ = joints_lower_limits;
    joints_upper_limits_ = joints_upper_limits;

    std::random_device rd;
    gen_ = std::mt19937(rd());

    bias_dist_ = std::uniform_real_distribution<double>(0.0, 1.0);

    for (size_t i = 0; i < joints_lower_limits.size(); ++i)
    {
        joint_dists_.emplace_back(joints_lower_limits[i], joints_upper_limits[i]);
    }

}

std::vector<VectorXd> RRTPlanner::computeRRT(const VectorXd& start, 
                                            const VectorXd& goal /*, 
                                const std::vector<Obstacle>& obstacles*/)
{
    int max_tree_size {5000};
    double step_size {0.05};
    double goal_tolerance {0.05};

    std::vector<RRTNode> tree;
    tree.push_back({start, -1});

    std::vector<double> goal_vec(goal.begin(), goal.end());
    
    std::vector<double> q_rand;

    for (int iterations = 0; iterations < max_tree_size; ++iterations)
    {
        VectorXd q_rand = sampleRandom(goal, 0.10);

        int nearest_idx = getNearestNodeIndex(tree, q_rand);
        VectorXd q_near = tree[nearest_idx].q;

        VectorXd q_new = steer(q_near, q_rand, step_size);

        // Possibility to add a condition on the Collision

        tree.push_back({q_new, nearest_idx});
        int new_node_idx = tree.size()-1;

        if ((q_new - goal).norm() < goal_tolerance)
        {
            tree.push_back({goal, new_node_idx});

            std::vector<VectorXd> path;
            int current_idx = tree.size() - 1;

            while (current_idx != -1)
            {
                path.push_back(tree[current_idx].q);
                current_idx = tree[current_idx].parent_index;
            }

            std::reverse(path.begin(), path.end());
            return path;

        }
    }
}


VectorXd RRTPlanner::sampleRandom(const VectorXd& goal_config, double goal_bias)
{
    if (bias_dist_(gen_) < goal_bias)
    {
        return goal_config;
    }

    VectorXd q_rand(n_dof_);

    for (int i = 0; i < joint_dists_.size(); ++i)
    {
        q_rand[i] = joint_dists_[i](gen_);
    }
    return q_rand;
}

int RRTPlanner::getNearestNodeIndex(const std::vector<RRTNode>& tree, const VectorXd& q_rand)
{
    int nearest_idx = 0;
    double min_dist = (tree[0].q - q_rand).norm();

    for (size_t i = 1; i < tree.size(); ++i)
    {
        double dist = (tree[i].q - q_rand).norm();
        if (dist < min_dist)
        {
            min_dist = dist;
            nearest_idx = i;
        }
    }
    return nearest_idx;
}

VectorXd RRTPlanner::steer(const VectorXd& q_near, const VectorXd& q_rand, double max_step_size)
{
    VectorXd direction = q_rand - q_near;
    double dist = direction.norm();

    if (dist > max_step_size)
    {
        return q_near + (direction / dist) * max_step_size;
    }
    return q_rand;
}



