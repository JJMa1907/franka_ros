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
  
  // Torque rate limiting
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);
  const double kDeltaTauMax = 1.0;
  
  // Cartesian Impedance Variables
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  double nullspace_stiffness_{0.0};
  double nullspace_stiffness_target_{0.0};
  
  // Cartesian subscribers and publishers
  ros::Subscriber sub_equilibrium_pose_;
  ros::Subscriber sub_equilibrium_config_;
  ros::Subscriber sub_stiffness_;
  ros::Publisher pub_stiff_update_;
  ros::Publisher pub_cartesian_pose_;
  ros::Publisher pub_force_torque_;
  
  // Cartesian callbacks
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void equilibriumConfigurationCallback(const std_msgs::Float32MultiArray::ConstPtr& joint);
  void equilibriumStiffnessCallback(const std_msgs::Float32MultiArray::ConstPtr& stiffness_);
  void complianceParamCallback(franka_example_controllers::compliance_paramConfig& config, uint32_t level);
  
  // Cartesian force/torque estimation
  Eigen::Matrix<double, 6, 1> force_torque;
  Eigen::Matrix<double, 6, 1> force_torque_old;
  Eigen::Matrix<double, 6, 1> stiff_;
  int filter_step = 0;
  int filter_step_ = 5;
  double alpha = 0.2;
  
  // Cartesian friction parameters
  double FI_11 = 1.0, FI_12 = 1.0, FI_13 = 1.0, FI_14 = 1.0, FI_15 = 1.0, FI_16 = 1.0, FI_17 = 1.0;
  double FI_21 = 1.0, FI_22 = 1.0, FI_23 = 1.0, FI_24 = 1.0, FI_25 = 1.0, FI_26 = 1.0, FI_27 = 1.0;
  double FI_31 = 0.0, FI_32 = 0.0, FI_33 = 0.0, FI_34 = 0.0, FI_35 = 0.0, FI_36 = 0.0, FI_37 = 0.0;
  double TAU_F_CONST_1 = 0.0, TAU_F_CONST_2 = 0.0, TAU_F_CONST_3 = 0.0, TAU_F_CONST_4 = 0.0;
  double TAU_F_CONST_5 = 0.0, TAU_F_CONST_6 = 0.0, TAU_F_CONST_7 = 0.0;
  
  // Joint Impedance Variables
  std::vector<double> k_gains_;
  std::vector<double> d_gains_;
  std::array<double, 7> q_desired_target_;
  
  // Joint command subscriber
  ros::Subscriber joint_command_sub_;
  void jointCommandCallback(const std_msgs::Float64MultiArrayConstPtr& msg);
  
  // Dynamic reconfigure
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
      dynamic_server_compliance_param_;
};

}  // namespace franka_example_controllers
