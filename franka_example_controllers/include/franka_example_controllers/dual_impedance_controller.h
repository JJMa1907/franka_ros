// Copyright (c) 2024 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <array>

#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

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
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  double nullspace_stiffness_{20.0};
  
  // Cartesian subscribers
  ros::Subscriber sub_equilibrium_pose_;
  ros::Subscriber sub_equilibrium_config_;
  
  // Cartesian publishers
  ros::Publisher pub_cartesian_pose_;
  
  // Cartesian callbacks
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void equilibriumConfigurationCallback(const std_msgs::Float32MultiArray::ConstPtr& joint);
  
  // Joint Impedance Variables
  std::vector<double> k_gains_;
  std::vector<double> d_gains_;
  std::array<double, 7> q_d_array_;
  
  // Joint subscriber
  ros::Subscriber joint_command_sub_;
  void jointCommandCallback(const std_msgs::Float64MultiArrayConstPtr& msg);
  
  // Common parameters
  const double delta_tau_max_{1.0};
};

}  // namespace franka_example_controllers
