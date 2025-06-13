// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>

#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

namespace franka_example_controllers {

class JointImpedanceExampleController : public controller_interface::MultiInterfaceController<
                                            franka_hw::FrankaModelInterface,
                                            hardware_interface::EffortJointInterface,
                                            franka_hw::FrankaPoseCartesianInterface,
                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Joint command callback
  void jointCommandCallback(const std_msgs::Float64MultiArrayConstPtr& msg);
  
  // Trajectory command callback (deoxys-compatible)
  void trajectoryCommandCallback(const franka_example_controllers::JointTrajectoryCommandConstPtr& msg);

  // Trajectory interpolation methods
  struct TrajectoryPoint {
    std::array<double, 7> position;
    std::array<double, 7> velocity;
    double timestamp;
  };

  void addTrajectoryPoint(const std::array<double, 7>& position, 
                         const std::array<double, 7>& velocity = {0,0,0,0,0,0,0});
  std::array<double, 7> interpolateTrajectory(double current_time, 
                                             std::array<double, 7>& target_velocity);
  void clearTrajectory();
  bool isTrajectoryActive() const;

  // Saturation
  std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated,
      const std::array<double, 7>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaCartesianPoseHandle> cartesian_pose_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

  // ROS subscriber for joint commands
  ros::Subscriber joint_command_sub_;
  
  // External command variables
  std::array<double, 7> q_desired_target_;
  bool use_external_command_;
  bool external_command_received_;
  bool is_delta_{false};  // Support for delta commands (from deoxys config)

  // Joint limits (deoxys-compatible)
  std::array<double, 7> joint_limits_upper_;
  std::array<double, 7> joint_limits_lower_;
  double joint_limit_margin_{0.1};  // Distance threshold for joint limit protection

  // Trajectory interpolation variables (deoxys-compatible)
  std::vector<TrajectoryPoint> trajectory_buffer_;
  size_t current_trajectory_index_;
  double trajectory_start_time_;
  bool trajectory_active_;
  std::array<double, 7> last_interpolated_position_;
  std::array<double, 7> last_interpolated_velocity_;
  
  // Max delta position per control cycle (from deoxys config)
  std::array<double, 7> max_delta_position_per_cycle_;

  static constexpr double kDeltaTauMax{1.0};
  double radius_{0.1};
  double acceleration_time_{2.0};
  double vel_max_{0.05};
  double angle_{0.0};
  double vel_current_{0.0};

  std::vector<double> k_gains_;
  std::vector<double> d_gains_;
  std::vector<double> max_delta_q_;
  double coriolis_factor_{1.0};
  std::array<double, 7> dq_filtered_;
  std::array<double, 16> initial_pose_;

  // State estimation parameters (exponential smoothing)
  double alpha_q_{0.9};
  double alpha_dq_{0.9};
  std::array<double, 7> position_smoothed_;
  std::array<double, 7> velocity_smoothed_;

  // Trajectory interpolation parameter
  double time_fraction_{1.0};

  franka_hw::TriggerRate rate_trigger_{1.0};
  std::array<double, 7> last_tau_d_{};
  realtime_tools::RealtimePublisher<JointTorqueComparison> torques_publisher_;
};

}  // namespace franka_example_controllers
