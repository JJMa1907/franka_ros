// Copyright (c) 2024 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <array>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>
#include <franka_example_controllers/JointTorqueComparison.h>
#include <realtime_tools/realtime_publisher.h>

namespace franka_example_controllers {

class DualImpedanceController : public controller_interface::MultiInterfaceController<
                                    franka_hw::FrankaModelInterface,
                                    hardware_interface::EffortJointInterface,
                                    franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Hardware interfaces
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  
  // Simple mode flag
  bool is_cartesian_mode_;
  
  // Mode subscriber  
  ros::Subscriber mode_sub_;
  void modeCallback(const std_msgs::Bool::ConstPtr& msg);

  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);
  
  // Cartesian Impedance Variables
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  double dt{0.001};
  int alpha;
  double time_start;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  double nullspace_stiffness_{0.0};        // 改为0，与原版一致
  double nullspace_stiffness_target_{0.0}; // 改为0，与原版一致
  
  // Force/Torque estimation variables
  Eigen::Matrix<double, 6, 1> force_torque_;
  Eigen::Matrix<double, 6, 1> force_torque_old_;
  Eigen::Matrix<float, 7, 1> stiff_;
  int filter_step_{0};
  int filter_step_max_{10};
  double alpha_force_{1.0};
  
  // Cartesian subscribers and publishers
  ros::Subscriber sub_equilibrium_pose_;
  ros::Subscriber sub_equilibrium_config_;
  ros::Subscriber sub_stiffness_;
  ros::Publisher pub_stiff_update_;
  ros::Publisher pub_cartesian_pose_;
  ros::Publisher pub_force_torque_;
  
  // Dynamic reconfigure for cartesian mode
  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  
  // Cartesian callbacks
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void equilibriumConfigurationCallback(const std_msgs::Float32MultiArray::ConstPtr& joint);
  void equilibriumStiffnessCallback(const std_msgs::Float32MultiArray::ConstPtr& stiffness);
  void complianceParamCallback(franka_example_controllers::compliance_paramConfig& config,
                               uint32_t level);

  // Joint Impedance Variables
  std::vector<double> k_gains_;
  std::vector<double> d_gains_;
  std::array<double, 7> q_d_array_;
  
  // Joint impedance external command parameters
  std::array<double, 7> q_desired_target_;
  std::array<double, 7> dq_filtered_;
  std::array<double, 7> last_tau_d_{};
  std::array<double, 7> initial_position_;
  std::array<double, 7> max_delta_q_;
  std::array<double, 7> joint_limits_upper_;
  std::array<double, 7> joint_limits_lower_;
  
  // State estimation parameters (exponential smoothing)
  double alpha_q_{0.9};
  double alpha_dq_{0.9};
  std::array<double, 7> position_smoothed_;
  std::array<double, 7> velocity_smoothed_;
  
  // Trajectory interpolation parameters
  double time_fraction_{1.0};
  
  // Control parameters for joint impedance
  bool use_external_command_{false};
  bool is_delta_{false};
  bool external_command_received_{false};
  double coriolis_factor_{1.0};
  double joint_limit_margin_{0.1}; // Distance threshold for joint limit protection
  // Power and torque limits
  double power_limit_{50.0}; // Maximum power limit in Watts
  double power_limit_startup_{35.0}; // Reduced power limit during startu
  double tau_limit_{87.0};
  
  // Startup protection
  bool startup_phase_{true};
  bool first_command_{true}; // Track first command for ramp-up
  double startup_duration_{1.0}; // Increased time for smoother startup
  ros::Time startup_time_;
  
  // Trajectory interpolation structures
  struct TrajectoryPoint {
    std::array<double, 7> position;
    std::array<double, 7> velocity;
    double timestamp;
  };
  
  // Trajectory interpolation variables
  std::vector<TrajectoryPoint> trajectory_buffer_;
  bool trajectory_active_{false};
  size_t current_trajectory_index_{0};
  double trajectory_start_time_{0.0};
  std::array<double, 7> last_interpolated_position_;
  std::array<double, 7> last_interpolated_velocity_;
  std::array<double, 7> max_delta_position_per_cycle_;
  int trajectory_completion_countdown_{0};
  
  // Publisher for torque comparison
  realtime_tools::RealtimePublisher<franka_example_controllers::JointTorqueComparison>
      torques_publisher_;
  franka_hw::TriggerRate rate_trigger_{1.0};
  // Joint impedance trajectory functions
  void addTrajectoryPoint(const std::array<double, 7>& position, 
                         const std::array<double, 7>& velocity);
  std::array<double, 7> interpolateTrajectory(double current_time, 
                                             std::array<double, 7>& target_velocity);
  void clearTrajectory();
  bool isTrajectoryActive() const;
  
  // Joint subscriber
  ros::Subscriber joint_command_sub_;
  void jointCommandCallback(const std_msgs::Float64MultiArrayConstPtr& msg);
  
  // Enhanced saturation for joint impedance mode
  std::array<double, 7> saturateTorqueRateJoint(
      const std::array<double, 7>& tau_d_calculated,
      const std::array<double, 7>& tau_J_d);
  
  // Common parameters
  const double delta_tau_max_{1.0};
  const double kDeltaTauMax{1.0};

  // Joint impedance mode variables
  bool joint_impedance_mode_{false};
  double trajectory_duration_{1.0};
  Eigen::Matrix<double, 7, 1> q_d_start_;
  Eigen::Matrix<double, 7, 1> q_d_end_;
  Eigen::Matrix<double, 7, 1> q_d_current_;
  // Using the already declared trajectory_start_time_ for both purposes
  bool trajectory_in_progress_{false};

  // Trajectory interpolation
  void computeTrajectory(const ros::Time& time);
  
  // State estimation
  void estimateState();
  
  // Safety protection at startup
  void startupProtection();
};

}  // namespace franka_example_controllers
