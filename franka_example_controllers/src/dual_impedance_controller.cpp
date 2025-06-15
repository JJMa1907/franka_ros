// Copyright (c) 2024 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/dual_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka_example_controllers/franka_model.h>
#include <franka_example_controllers/pseudo_inversion.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

bool DualImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {
  // Initialize mode flag - default to cartesian
  is_cartesian_mode_ = true;
  
  // Simple mode subscriber
  mode_sub_ = node_handle.subscribe(
      "/impedance_mode", 1, &DualImpedanceController::modeCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  
  // Get arm_id parameter
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("DualImpedanceController: Could not read parameter arm_id");
    return false;
  }
  
  // Get joint names
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR("DualImpedanceController: Invalid or no joint_names parameters provided!");
    return false;
  }

  // Initialize hardware interfaces exactly like original controllers
  franka_hw::FrankaModelInterface* model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("DualImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("DualImpedanceController: Exception getting model handle: " << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("DualImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("DualImpedanceController: Exception getting state handle: " << ex.what());
    return false;
  }

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("DualImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("DualImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  // Initialize Cartesian impedance parameters exactly like original
  sub_equilibrium_pose_ = node_handle.subscribe(
      "/equilibrium_pose", 20, &DualImpedanceController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  sub_equilibrium_config_ = node_handle.subscribe(
      "/equilibrium_configuration", 20, &DualImpedanceController::equilibriumConfigurationCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  // Initialize Joint impedance parameters exactly like original
  joint_command_sub_ = node_handle.subscribe(
      "/joint_command", 1, &DualImpedanceController::jointCommandCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  // Set default parameters exactly like original controllers
  cartesian_stiffness_.setIdentity();
  cartesian_stiffness_ = cartesian_stiffness_ * 150.0;
  cartesian_damping_.setIdentity();
  cartesian_damping_ = cartesian_damping_ * 20.0;
  
  k_gains_ = {3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0};
  d_gains_ = {50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0};
  
  // Initialize positions
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  q_d_nullspace_.setZero();
  q_d_array_.fill(0.0);

  return true;
}

void DualImpedanceController::starting(const ros::Time& time) {
  // Get initial robot state
  franka::RobotState initial_state = state_handle_->getRobotState();
  
  // Initialize for cartesian mode exactly like original cartesian controller
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
  
  // Set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  q_d_nullspace_ = q_initial;
  
  // Initialize joint targets
  for (size_t i = 0; i < 7; ++i) {
    q_d_array_[i] = initial_state.q[i];
  }
}

void DualImpedanceController::update(const ros::Time& time, const ros::Duration& period) {
  // Get state variables exactly like original controllers
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // Convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(robot_state.tau_J_d.data());

  Eigen::VectorXd tau_d(7);

  if (is_cartesian_mode_) {
    // ============ CARTESIAN IMPEDANCE MODE ============
    // Exactly like original cartesian controller
    
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());

    // Compute pose error exactly like original
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d_;

    // Orientation error exactly like original
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
    Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
    error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

    // Compute control exactly like original
    Eigen::VectorXd tau_task(7), tau_nullspace(7), null_vect(7);
    Eigen::MatrixXd jacobian_transpose_pinv;
    franka_example_controllers::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
    Eigen::MatrixXd Null_mat = Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv;
    
    null_vect.setZero();
    for (size_t i = 0; i < 7; ++i) {
      null_vect(i) = q_d_nullspace_(i) - q(i);
    }

    // Cartesian PD control exactly like original
    tau_task << jacobian.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
    tau_nullspace << Null_mat * (nullspace_stiffness_ * null_vect - 2.0 * sqrt(std::max(nullspace_stiffness_, 1e-6)) * dq);
    
    tau_d << tau_task + tau_nullspace + coriolis;

  } else {
    // ============ JOINT IMPEDANCE MODE ============ 
    // Simple joint impedance exactly like original
    
    for (size_t i = 0; i < 7; ++i) {
      double position_error = q_d_array_[i] - q[i];
      tau_d[i] = k_gains_[i] * position_error - d_gains_[i] * dq[i];
    }

    // Add coriolis exactly like original
    tau_d += coriolis;
  }

  // Saturate torque rate exactly like original
  tau_d = saturateTorqueRate(tau_d, tau_J_d);
  
  // Set joint commands
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
}

void DualImpedanceController::modeCallback(const std_msgs::Bool::ConstPtr& msg) {
  is_cartesian_mode_ = msg->data;
  ROS_INFO("DualImpedanceController: Switched to %s mode", 
           is_cartesian_mode_ ? "Cartesian" : "Joint");
}

Eigen::Matrix<double, 7, 1> DualImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

// ============ CARTESIAN IMPEDANCE CALLBACKS ============
void DualImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_);
  orientation_d_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_.coeffs()) < 0.0) {
    orientation_d_.coeffs() << -orientation_d_.coeffs();
  }
}

void DualImpedanceController::equilibriumConfigurationCallback(
    const std_msgs::Float32MultiArray::ConstPtr& joint) {
  if (joint->data.size() != 7) return;
  for (size_t i = 0; i < 7; ++i) {
    q_d_nullspace_[i] = joint->data[i];
  }
}

void DualImpedanceController::jointCommandCallback(const std_msgs::Float64MultiArrayConstPtr& msg) {
  if (msg->data.size() != 7) {
    ROS_ERROR("Joint command must have 7 elements");
    return;
  }
  
  // Simple joint command - directly set target
  for (size_t i = 0; i < 7; ++i) {
    q_d_array_[i] = msg->data[i];
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DualImpedanceController,
                       controller_interface::ControllerBase)
