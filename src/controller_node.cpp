#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "arm.hpp"
#include "controller_base.hpp"
#include "OFC_controller.hpp"
#include "CTC_controller.hpp"
#include "RRT_planner.hpp"
#include "trajectory_generator.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class ControllerNode: public rclcpp::Node
{
    public:
    ControllerNode() : 
        Node("controller_node", rclcpp::NodeOptions().parameter_overrides({{"use_sim_time", true}})),
        arm_model(1.0, 0.1, 1.0, 0.1, 1.0, 0.1, 1.0, 0.5, 0.2) 
    {
        n_dof_ = 3;
        current_q.resize(3, 0.0);
        current_dq.resize(3, 0.0);

        q_start_ = Eigen::VectorXd::Zero(3);
        q_final_ = Eigen::VectorXd::Zero(3);
        dq_final_ = Eigen::VectorXd::Zero(3);
        ddq_final_ = Eigen::VectorXd::Zero(3);

        use_tvlqr_ = this->declare_parameter<bool>("use_tvlqr", true);

        // === Planner initialization ===
        std::vector<double> lower_bounds = {-3.14, -2.0, -2.0};
        std::vector<double> upper_bounds = {3.14, 2.0, 2.0};
        planner_ = std::make_unique<RRTPlanner>(lower_bounds, upper_bounds, n_dof_);
        traj_gen_ = std::make_unique<TrajectoryGenerator>(&arm_model, planner_.get(), lower_bounds, upper_bounds, n_dof_);

        // === Controller initialization ===
        if (use_tvlqr_) 
        {
            Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n_dof_*3, n_dof_*3);
            Eigen::MatrixXd Q_f = Eigen::MatrixXd::Identity(n_dof_*3, n_dof_*3);
            
            double q_pos = 1000.0; 
            double q_vel = 50.0;
            double q_int = 200.0;
            Q(0,0) = q_pos; Q(1,1) = q_pos; Q(2,2) = q_pos; 
            Q(3,3) = q_vel; Q(4,4) = q_vel; Q(5,5) = q_vel;
            Q(6,6) = q_int; Q(7,7) = q_int; Q(8,8) = q_int;

            double qf_pos = 5000.0; 
            double qf_vel = 500.0; 
            double qf_int = 200.0;

            Q_f(0,0) = qf_pos; Q_f(1,1) = qf_pos; Q_f(2,2) = qf_pos; 
            Q_f(3,3) = qf_vel; Q_f(4,4) = qf_vel; Q_f(5,5) = qf_vel;
            Q_f(6,6) = qf_int; Q_f(7,7) = qf_int; Q_f(8,8) = qf_int;
            Eigen::MatrixXd R = Eigen::MatrixXd::Identity(n_dof_, n_dof_)* 0.01;
            
            Eigen::VectorXd X_final = Eigen::VectorXd::Zero(6);
            X_final.head(3) = q_final_;
            controller_ = std::make_unique<OFCController>(&arm_model, X_final, Q, R, Q_f, 200, 0.01, 3);
        } else {
            controller_ = std::make_unique<CTCController>(50.0, 14.0, &arm_model);
        }

        // === ROS communication ===
        effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/effort_controller/commands", 10);
        
        sensor_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(), std::bind(&ControllerNode::sensor_callback, this, _1));
        
        goal_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/goal", 10, std::bind(&ControllerNode::goal_callback, this, _1));

        timer_ = this->create_timer(
        10ms, std::bind(&ControllerNode::control_loop, this));
        RCLCPP_INFO(this->get_logger(), "Constructor finished");
            
        // === Gaussian Noise generation ===
        with_noise_ = this->declare_parameter<bool>("with_noise", true);
        generator_.seed(std::chrono::system_clock::now().time_since_epoch().count());
        std::normal_distribution<double> distribution(0.0, 0.05);
    }
    private:

        void sensor_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            for (size_t i = 0; i < msg->name.size(); i++) {
                if (i >= msg->name.size()) break;

                if (msg->name[i] == "joint1_v1") {
                    current_q[0] = msg->position[i];
                    current_dq[0] = msg->velocity[i];
                } else if (msg->name[i] == "joint2_v1") {
                    current_q[1] = msg->position[i];
                    current_dq[1] = msg->velocity[i];
                } else if (msg->name[i] == "joint3_v1") {
                    current_q[2] = msg->position[i];
                    current_dq[2] = msg->velocity[i];
                }
            }
            RCLCPP_DEBUG(this->get_logger(), "Message received. Positions: %f, %f, %f", current_q[0], current_q[1], current_q[2]);
            state_received_ = true;
        }

        void control_loop()
        {
            if (!state_received_) 
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for joint states...");
                return;
            }

            static bool first_goal_received = false;
            if (is_moving_) first_goal_received = true;

            if (!first_goal_received) 
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for instructions...");
                return; 
            }
            
            // Vectorization of the current state
            Eigen::VectorXd current_q_vec = Eigen::Map<const Eigen::VectorXd>(current_q.data(), current_q.size());
            Eigen::VectorXd current_dq_vec = Eigen::Map<const Eigen::VectorXd>(current_dq.data(), current_dq.size());
            
            if (with_noise_)
            {
                for (int i=0; i < current_q_vec.size(); ++i)
                {
                    current_q_vec(i) += distribution_(generator_);
                }
                for (int i=0; i < current_dq_vec.size(); ++i)
                {
                    current_dq_vec(i) += distribution_(generator_);
                }
            }

            double t = (this->now() - t_start_).seconds();
            if (t >= traj_duration_) {
                t = traj_duration_;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Movement finished.");
            }

            Eigen::VectorXd q_goal(3);
            Eigen::VectorXd tau_vec = Eigen::VectorXd::Zero(3);

            if (use_tvlqr_) 
            {
                // === TV-LQR ===
                std::vector<Eigen::VectorXd> dummy_q, dummy_dq, dummy_ddq;
                
                tau_vec = controller_->getComputedTorque(
                    current_q_vec, 
                    current_dq_vec, 
                    dummy_q, 
                    dummy_dq, 
                    dummy_ddq
                );
            } else {
                // === CTC (impedance control) ===
                Eigen::VectorXd current_q_goal = q_final_;
                Eigen::VectorXd current_dq_goal = Eigen::VectorXd::Zero(3);
                Eigen::VectorXd current_ddq_goal = Eigen::VectorXd::Zero(3);

                if (is_moving_)
                {
                    double t = (this->now() - t_start_).seconds();

                    if (t >= traj_duration_)
                    {
                        is_moving_ = false;
                    } 
                    else 
                    {
                        for (int i = 0; i < 3; ++i) {
                            double q0 = q_start_[i];
                            double qf = q_final_[i];

                            double a2 = 3.0 * (qf - q0) / (traj_duration_ * traj_duration_);
                            double a3 = -2.0 * (qf - q0) / (traj_duration_ * traj_duration_ * traj_duration_);

                            current_q_goal(i)   = q0 + a2 * t * t + a3 * t * t * t;
                            current_dq_goal(i)  = 2.0 * a2 * t + 3.0 * a3 * t * t;
                            current_ddq_goal(i) = 2.0 * a2 + 6.0 * a3 * t;
                        }
                    }
                }
                tau_vec = controller_->getComputedTorque(
                    current_q_vec, 
                    current_dq_vec, 
                    current_q_goal, 
                    current_dq_goal, 
                    current_ddq_goal
                );

            }

            std::vector<double> tau(tau_vec.data(), tau_vec.data() + tau_vec.size());            
            std_msgs::msg::Float64MultiArray command_msg;
            command_msg.data = tau;
            
            effort_pub_->publish(command_msg);
            RCLCPP_DEBUG(this->get_logger(), "Computations done. Commands published");
        }

        void goal_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            if (msg->data.size() == 3)
            {
                q_start_ << current_q[0], current_q[1], current_q[2];
                q_final_ << msg->data[0], msg->data[1], msg->data[2];

                RCLCPP_INFO(this->get_logger(), "New goal received");

                bool success = traj_gen_->planTrajectory(q_start_, q_final_, traj_duration_);
                if (success) 
                {
                    if (use_tvlqr_) {
                        int N_traj = 200; 
                        double dt = 0.01;
                        std::vector<Eigen::VectorXd> ref_q(N_traj + 1, Eigen::VectorXd::Zero(n_dof_));
                        std::vector<Eigen::VectorXd> ref_dq(N_traj + 1, Eigen::VectorXd::Zero(n_dof_));
                        std::vector<Eigen::VectorXd> ref_ddq(N_traj, Eigen::VectorXd::Zero(n_dof_));
                        
                        for (int i = 0; i <= N_traj; ++i) 
                        {
                            double t_futur = i * dt;
                            ref_q[i] = traj_gen_->getPosition(t_futur);
                            ref_dq[i] = traj_gen_->getVelocity(t_futur);
                            if (i < N_traj) ref_ddq[i] = traj_gen_->getAcceleration(t_futur);
                        }
                        
                        auto ofc_ctrl = dynamic_cast<OFCController*>(controller_.get());
                        if (ofc_ctrl) 
                        {
                            ofc_ctrl->initTrajectory(ref_q, ref_dq, ref_ddq);
                        }
                    }
            
                    t_start_ = this->now();
                    is_moving_ = true;
                    RCLCPP_INFO(this->get_logger(), "Trajectory planned successfully. Moving...");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to plan trajectory!");
                }
            }
        }

        int n_dof_;
        Arm arm_model;
        
        std::unique_ptr<RRTPlanner> planner_;
        std::unique_ptr<TrajectoryGenerator> traj_gen_;
        std::unique_ptr<ControllerBase> controller_;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sensor_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr goal_sub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::vector<double> current_q;
        std::vector<double> current_dq;
        bool state_received_ = false;
        bool use_tvlqr_;
        bool with_noise_;

        // Variables for trajectory computation
        Eigen::VectorXd q_start_;
        Eigen::VectorXd q_final_;
        Eigen::VectorXd dq_final_;
        Eigen::VectorXd ddq_final_;
        rclcpp::Time t_start_;
        bool is_moving_ = false;
        double traj_duration_ = 2.0;

        std::default_random_engine generator_;
        std::normal_distribution<double> distribution_{0.0, 0.05};
    };

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}