#include <cmath>
#include <random>
#include <vector>
#include <Eigen/Dense>

#include "arm.hpp"
#include "trajectory_generator.hpp"
#include "screw_math.hpp"

using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;
using Vector3d = Eigen::Vector3d;

TrajectoryGenerator::TrajectoryGenerator(Arm* model,
                                        RRTPlanner* planner,
                                        const std::vector<double>& joints_lower_limits,
                                        const std::vector<double>& joints_upper_limits,
                                        int n_dof)
{
    model_ = model;
    planner_ = planner;
    n_dof_ = n_dof;
    joints_lower_limits_ = joints_lower_limits;
    joints_upper_limits_ = joints_upper_limits;
}

bool TrajectoryGenerator::planTrajectory(const VectorXd& q_start,
                    const VectorXd& q_goal/*,
                    const std::vector<Obstacle>& obstacles*/,
                    double total_duration)
{
    t_start_ = 0.0;
    std::vector<VectorXd> raw_path = planner_->computeRRT(q_start, q_goal);

    if (raw_path.empty())
    {
        return false;
    }

    std::vector<VectorXd> refined_path = refinePathWithAPF(raw_path);

    computePolynomialCoefficients(refined_path, total_duration);

    return true;
}

int getSegmentIndex(double t, const std::vector<SplineSegment>& splines)
{
    if (splines.empty()) return 0;
    
    if (t >= splines.back().start_time + splines.back().duration) {
        return splines.size() - 1;
    }
    
    for (size_t i = 0; i < splines.size(); ++i) {
        if (t >= splines[i].start_time && t < splines[i].start_time + splines[i].duration) {
            return i;
        }
    }
    return 0;
}

Eigen::VectorXd TrajectoryGenerator::getPosition(double t) const 
{
    if (splines_.empty()) return VectorXd::Zero(n_dof_); // Erreur
    if (t <= t_start_) return splines_.front().coeffs.row(0);
    
    int idx = getSegmentIndex(t, splines_);
    const auto& seg = splines_[idx];
    
    if (t >= seg.start_time + seg.duration && idx == splines_.size() - 1) {
        return splines_.back().coeffs.row(0) + splines_.back().coeffs.row(1) + 
               splines_.back().coeffs.row(2) + splines_.back().coeffs.row(3) + 
               splines_.back().coeffs.row(4) + splines_.back().coeffs.row(5);
    }

    double tau = (t - seg.start_time) / seg.duration;
    
    VectorXd pos = seg.coeffs.row(0) 
                 + seg.coeffs.row(1) * tau 
                 + seg.coeffs.row(2) * pow(tau, 2) 
                 + seg.coeffs.row(3) * pow(tau, 3) 
                 + seg.coeffs.row(4) * pow(tau, 4) 
                 + seg.coeffs.row(5) * pow(tau, 5);
    return pos;
}

Eigen::VectorXd TrajectoryGenerator::getVelocity(double t) const 
{
    if (splines_.empty() || t <= t_start_ || t >= splines_.back().start_time + splines_.back().duration) {
        return VectorXd::Zero(n_dof_);
    }
    
    int idx = getSegmentIndex(t, splines_);
    const auto& seg = splines_[idx];
    double tau = (t - seg.start_time) / seg.duration;
    
    VectorXd vel = (seg.coeffs.row(1) 
                 + 2 * seg.coeffs.row(2) * tau 
                 + 3 * seg.coeffs.row(3) * pow(tau, 2) 
                 + 4 * seg.coeffs.row(4) * pow(tau, 3) 
                 + 5 * seg.coeffs.row(5) * pow(tau, 4)) / seg.duration;
    return vel;
}

Eigen::VectorXd TrajectoryGenerator::getAcceleration(double t) const 
{
    if (splines_.empty() || t <= t_start_ || t >= splines_.back().start_time + splines_.back().duration) {
        return VectorXd::Zero(n_dof_);
    }
    
    int idx = getSegmentIndex(t, splines_);
    const auto& seg = splines_[idx];
    double tau = (t - seg.start_time) / seg.duration;
    
    VectorXd acc = (2 * seg.coeffs.row(2) 
                 + 6 * seg.coeffs.row(3) * tau 
                 + 12 * seg.coeffs.row(4) * pow(tau, 2) 
                 + 20 * seg.coeffs.row(5) * pow(tau, 3)) / (seg.duration * seg.duration);
    return acc;
}





std::vector<VectorXd> TrajectoryGenerator::refinePathWithAPF(const std::vector<VectorXd>& raw_path /*, 
                                        const std::vector<Obstacle>& obstacles*/)
{
    if (raw_path.size() < 3) return raw_path;

    std::vector<VectorXd> refined_path = raw_path;

    int max_iterations {200};
    double alpha {0.05};
    double k_att {0.5};
    double k_rep {0.01};
    double influence_radius {0.5};

    for (int iter = 0; iter < max_iterations; ++iter)
    {
        for (size_t i = 1; i < refined_path.size() - 1; ++i)
        {
            VectorXd q_current = refined_path[i];


            VectorXd f_att = k_att * (refined_path[i-1] + refined_path[i+1] - 2 * q_current);

            VectorXd f_rep_joint_space = VectorXd::Zero(n_dof_);
            
            VectorXd end_effector_pos = (model_->forwardKinematic(q_current))[2].block<3,1>(0,3);

            
            //for (const auto& obs : obstacles)
            //{
            //    Vector3d direction = end_effector_pos - obs.position;
            //    double distance = direction.norm();
//
            //    if (distance < influence_radius && distance > 0.01)
            //    {
            //        double force_magnitude = k_rep * (1.0 / distance - 1.0 / influence_radius) * (1.0 / (distance * distance));
            //        
            //        Vector3d f_rep_cartesian = force_magnitude * (direction / distance);
//
            //        MatrixXd J = model_->getJacobian(q_current);
            //        
            //        f_rep_joint_space += J.transpose() * f_rep_cartesian; 
            //    }
            //}

            refined_path[i] += alpha * (f_att + f_rep_joint_space);
        }
    }

    return refined_path;

}                                               
                                                
void TrajectoryGenerator::computePolynomialCoefficients(const std::vector<VectorXd>& path, double total_duration)
{
    splines_.clear();
    int num_points = path.size();

    if (num_points < 2) return;

    std::vector<double> distances(num_points - 1, 0.0);
    double total_distance {0.0};

    for (int i = 0; i < num_points - 1; ++i) {
        distances[i] = (path[i+1] - path[i]).norm();
        total_distance += distances[i];
    }
    if (total_distance < 1e-6)
    {
        total_distance = 1e-6; 
    }

    std::vector<double> t_alloc(num_points - 1, 0.0);
    double current_time = t_start_;

    std::vector<VectorXd> velocities(num_points, VectorXd::Zero(n_dof_));
    for (int i = 1; i < num_points - 1; ++i) {
        velocities[i] = (path[i+1] - path[i-1]) / (total_duration * (distances[i-1] + distances[i]) / total_distance);
    }
    
    VectorXd accel_zero = VectorXd::Zero(n_dof_); 

    // Compution of 5th order polynomials
    for (int i = 0; i < num_points - 1; ++i) 
    {
        SplineSegment segment;
        segment.duration = std::max(1e-4, total_duration * (distances[i] / total_distance));
        segment.start_time = current_time;
        
        segment.coeffs = Eigen::MatrixXd::Zero(6, n_dof_);
        
        VectorXd q0 = path[i];
        VectorXd q1 = path[i+1];
        VectorXd v0 = velocities[i];
        VectorXd v1 = velocities[i+1];
        
        double T = segment.duration;

        segment.coeffs.row(0) = q0; // coef c0
        segment.coeffs.row(1) = v0 * T; // coef c1
        segment.coeffs.row(2) = 0.5 * accel_zero * (T * T); // coef c2
        
        segment.coeffs.row(3) = 10*(q1 - q0) - 6*v0*T - 4*v1*T; // coef c3
        segment.coeffs.row(4) = -15*(q1 - q0) + 8*v0*T + 7*v1*T; // coef c4
        segment.coeffs.row(5) = 6*(q1 - q0) - 3*(v0 + v1)*T; // coef c5
        
        splines_.push_back(segment);
        current_time += segment.duration;
    }

}
                                            


