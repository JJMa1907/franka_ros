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
  // Initialize mode (default to cartesian)
  current_mode_ = CARTESIAN_IMPEDANCE;
  prev_mode_ = CARTESIAN_IMPEDANCE;
  mode_transition_active_ = false;
  transition_time_ = 0.0;
  
  // Get transition duration parameter
  if (!node_handle.getParam("transition_duration", transition_duration_)) {
    ROS_INFO_STREAM("DualImpedanceController: transition_duration not found. Defaulting to " << transition_duration_);
  }
  
  mode_sub_ = node_handle.subscribe("/impedance_mode", 1, &DualImpedanceController::modeCallback, this);
  
  // Get arm_id parameter
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("DualImpedanceController: Could not read parameter arm_id");
    return false;
  }
  
  // Get joint names
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "DualImpedanceController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  // Initialize Cartesian impedance parameters
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

  // Cartesian subscribers
  sub_equilibrium_pose_ = node_handle.subscribe(
      "/equilibrium_pose", 20, &DualImpedanceController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  sub_equilibrium_config_ = node_handle.subscribe(
      "/equilibrium_configuration", 20, &DualImpedanceController::equilibriumConfigurationCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  sub_stiffness_ = node_handle.subscribe(
      "/stiffness", 20, &DualImpedanceController::equilibriumStiffnessCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  // Cartesian publishers
  pub_stiff_update_ = node_handle.advertise<dynamic_reconfigure::Config>(
      "/dynamic_reconfigure_compliance_param_node/parameter_updates", 5);
  pub_cartesian_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("/cartesian_pose", 1);
  pub_force_torque_ = node_handle.advertise<geometry_msgs::WrenchStamped>("/force_torque_ext", 1);

  // Initialize Joint impedance parameters
  if (!node_handle.getParam("radius", radius_)) {
    ROS_INFO_STREAM("DualImpedanceController: No parameter radius, defaulting to: " << radius_);
  }
  if (std::fabs(radius_) < 0.005) {
    ROS_INFO_STREAM("DualImpedanceController: Set radius to small, defaulting to: " << 0.1);
    radius_ = 0.1;
  }

  if (!node_handle.getParam("vel_max", vel_max_)) {
    ROS_INFO_STREAM("DualImpedanceController: No parameter vel_max, defaulting to: " << vel_max_);
  }
  if (!node_handle.getParam("acceleration_time", acceleration_time_)) {
    ROS_INFO_STREAM("DualImpedanceController: No parameter acceleration_time, defaulting to: " << acceleration_time_);
  }

  // Initialize joint stiffness and damping with default values
  std::vector<double> joint_kp = {300.0, 300.0, 300.0, 300.0, 225.0, 450.0, 150.0};
  std::vector<double> joint_kd = {20.0, 20.0, 20.0, 20.0, 7.5, 15.0, 5.0};
  std::vector<double> max_delta_q = {0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06};
  
  // Try to get parameters from param server if available
  node_handle.getParam("joint_kp", joint_kp);
  node_handle.getParam("joint_kd", joint_kd);
  node_handle.getParam("max_delta_q", max_delta_q);

  // Use joint_kp and joint_kd as k_gains and d_gains
  k_gains_.resize(7);
  d_gains_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    k_gains_[i] = joint_kp[i];
    d_gains_[i] = joint_kd[i];
  }

  // Joint control parameters
  alpha_q_ = 0.9;
  alpha_dq_ = 0.9;
  node_handle.getParam("alpha_q", alpha_q_);
  node_handle.getParam("alpha_dq", alpha_dq_);

  time_fraction_ = 1.0;
  node_handle.getParam("time_fraction", time_fraction_);
  
  // Set a more conservative default startup duration
  startup_duration_ = 5.0;  // Increased to 5.0 seconds for more gradual startup
  node_handle.getParam("startup_duration", startup_duration_);

  position_smoothed_.fill(0.0);
  velocity_smoothed_.fill(0.0);
  max_delta_q_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    max_delta_q_[i] = max_delta_q[i];
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("DualImpedanceController: publish_rate not found. Defaulting to " << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("DualImpedanceController: coriolis_factor not found. Defaulting to " << coriolis_factor_);
  }

  // Joint control flags
  if (!node_handle.getParam("use_external_command", use_external_command_)) {
    ROS_INFO_STREAM("DualImpedanceController: use_external_command not found. Defaulting to " << use_external_command_);
  }

  if (!node_handle.getParam("is_delta", is_delta_)) {
    ROS_INFO_STREAM("DualImpedanceController: is_delta not found. Defaulting to " << is_delta_);
  }

  // Initialize joint limits
  joint_limits_upper_ = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
  joint_limits_lower_ = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
  
  std::vector<double> upper_limits, lower_limits;
  if (node_handle.getParam("joint_limits_upper", upper_limits) && upper_limits.size() == 7) {
    for (size_t i = 0; i < 7; ++i) {
      joint_limits_upper_[i] = upper_limits[i];
    }
  }
  if (node_handle.getParam("joint_limits_lower", lower_limits) && lower_limits.size() == 7) {
    for (size_t i = 0; i < 7; ++i) {
      joint_limits_lower_[i] = lower_limits[i];
    }
  }
  
  if (!node_handle.getParam("joint_limit_margin", joint_limit_margin_)) {
    ROS_INFO_STREAM("DualImpedanceController: joint_limit_margin not found. Defaulting to " << joint_limit_margin_);
  }

  // Power and torque limits
  if (!node_handle.getParam("power_limit", power_limit_)) {
    ROS_INFO_STREAM("DualImpedanceController: power_limit not found. Defaulting to " << power_limit_);
  }
  
  power_limit_startup_ = power_limit_ * 0.7;
  if (!node_handle.getParam("power_limit_startup", power_limit_startup_)) {
    ROS_INFO_STREAM("DualImpedanceController: power_limit_startup not found. Defaulting to " << power_limit_startup_);
  }
  
  if (!node_handle.getParam("tau_limit", tau_limit_)) {
    ROS_INFO_STREAM("DualImpedanceController: tau_limit not found. Defaulting to " << tau_limit_);
  }

  // Get hardware interfaces
  franka_hw::FrankaModelInterface* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("DualImpedanceController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("DualImpedanceController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("DualImpedanceController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("DualImpedanceController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  // Get cartesian pose interface for joint mode
  auto* cartesian_pose_interface = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
  if (cartesian_pose_interface != nullptr) {
    try {
      cartesian_pose_handle_.reset(new franka_hw::FrankaCartesianPoseHandle(
          cartesian_pose_interface->getHandle(arm_id + "_robot")));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_WARN_STREAM("DualImpedanceController: Could not get cartesian pose handle: " << ex.what());
    }
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

  // Initialize joint mode publisher and subscriber
  torques_publisher_.init(node_handle, "torque_comparison", 1);
  joint_command_sub_ = node_handle.subscribe(
      "joint_command", 1, &DualImpedanceController::jointCommandCallback, this);

  // Initialize dynamic reconfigure for cartesian mode
  dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle("dynamic_reconfigure_compliance_param_node");
  dynamic_server_compliance_param_.reset(
      new dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>(
          dynamic_reconfigure_compliance_param_node_));
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&DualImpedanceController::complianceParamCallback, this, _1, _2));

  // Initialize variables - CRITICAL: Follow exact pattern from original cartesian controller
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  
  // CRITICAL: Initialize all stiffness/damping matrices to ZERO like original cartesian controller
  // This prevents huge torques on first control cycles - controller starts with NO impedance
  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();
  cartesian_stiffness_target_.setZero();
  cartesian_damping_target_.setZero();
  nullspace_stiffness_ = 0.0;
  nullspace_stiffness_target_ = 0.0;
  
  // CRITICAL: Flag to track if stiffness has been initialized by dynamic reconfigure
  cartesian_stiffness_initialized_ = false;
  
  // Initialize other cartesian variables exactly like original
  stiff_.setZero();
  force_torque.setZero();
  force_torque_old.setZero();
  
  // Initialize cartesian filter variables exactly like original
  filter_step = 0;
  filter_step_ = 5;  // Default filter steps like original
  alpha = 0.2;       // Default alpha like original

  // Initialize joint variables
  for (size_t i = 0; i < 7; ++i) {
    q_desired_target_[i] = 0.0;
    last_interpolated_position_[i] = 0.0;
    last_interpolated_velocity_[i] = 0.0;
    max_delta_position_per_cycle_[i] = max_delta_q[i];
  }

  external_command_received_ = false;
  trajectory_active_ = false;
  current_trajectory_index_ = 0;
  trajectory_start_time_ = 0.0;
  trajectory_buffer_.clear();
  trajectory_completion_countdown_ = 0;

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);
  
  // Initialize startup variables for smooth controller start
  startup_phase_ = true;
  startup_time_ = ros::Time(0);
  first_command_ = true;
  std::fill(initial_position_.begin(), initial_position_.end(), 0.0);
  std::fill(last_tau_d_.begin(), last_tau_d_.end(), 0.0);

  return true;
}

void DualImpedanceController::starting(const ros::Time& time) {
  // Prevent double initialization
  static bool already_started = false;
  if (already_started) {
    ROS_WARN("DualImpedanceController::starting() called multiple times - ignoring subsequent calls");
    return;
  }
  already_started = true;
  
  // Get initial robot state
  franka::RobotState initial_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q_initial(initial_state.q.data());
  
  // Initialize ONLY the default mode (Cartesian) - much simpler like original controllers
  if (current_mode_ == CARTESIAN_IMPEDANCE) {
    // Follow cartesian_impedance_example_controller pattern exactly
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    
    position_d_ = initial_transform.translation();
    orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
    q_d_nullspace_ = q_initial;
    force_torque_old.setZero();
    cartesian_stiffness_initialized_ = false; // Ensure stiffness is not yet initialized
    
    // CRITICAL: Keep stiffness matrices at ZERO - they will be set by dynamic reconfigure
    // Do NOT set any non-zero stiffness values here!
    
    ROS_INFO("DualImpedanceController: Starting in Cartesian Impedance Mode");
  } else {
    // Follow joint_impedance_example_controller pattern exactly
    if (cartesian_pose_handle_) {
      initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
      franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();
      for (size_t i = 0; i < 7; ++i) {
        q_desired_target_[i] = robot_state.q[i];
        position_smoothed_[i] = robot_state.q[i];
        velocity_smoothed_[i] = robot_state.dq[i];
        last_interpolated_position_[i] = robot_state.q[i];
        last_interpolated_velocity_[i] = robot_state.dq[i];
      }
    } else {
      for (size_t i = 0; i < 7; ++i) {
        q_desired_target_[i] = initial_state.q[i];
        position_smoothed_[i] = initial_state.q[i];
        velocity_smoothed_[i] = initial_state.dq[i];
        last_interpolated_position_[i] = initial_state.q[i];
        last_interpolated_velocity_[i] = initial_state.dq[i];
      }
    }
    
    ROS_INFO("DualImpedanceController: Starting in Joint Impedance Mode");
  }
  
  // Common initialization
  for (size_t i = 0; i < 7; ++i) {
    dq_filtered_[i] = initial_state.dq[i];
    initial_position_[i] = initial_state.q[i];
    last_tau_d_[i] = 0.0;
  }
  
  // Initialize startup and transition variables
  startup_phase_ = true;
  startup_time_ = ros::Time(0);
  first_command_ = true;
  mode_transition_active_ = false;
  transition_time_ = 0.0;
  last_mode_tau_d_.setZero();
  
  // Clear trajectory (for joint mode)
  clearTrajectory();
}

void DualImpedanceController::update(const ros::Time& time, const ros::Duration& period) {
  // Get current robot state
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 49> mass_array = model_handle_->getMass();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  std::array<double, 7> gravity = model_handle_->getGravity();

  // Convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 7> > mass(mass_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(robot_state.tau_J_d.data());

  // Handle startup smoothing (following original joint impedance controller pattern)
  double startup_factor = 1.0;
  
  if (startup_phase_) {
    // Initialize on first update cycle (matches original pattern)
    if (startup_time_.isZero()) {
      startup_time_ = time;
      for (size_t i = 0; i < 7; ++i) {
        initial_position_[i] = robot_state.q[i];
      }
      ROS_INFO("Dual impedance controller starting with smooth ramp-up over %.1f seconds", startup_duration_);
    }
    
    // Calculate time elapsed since controller start
    double time_elapsed = (time - startup_time_).toSec();
    if (time_elapsed < 0) {
      time_elapsed = 0;
    }
    
    // CRITICAL: For the first 100ms, apply essentially no control (only gravity compensation)
    if (time_elapsed < 0.1) {
      startup_factor = 0.0;  // Pure gravity compensation for first 100ms
    } else {
      // Use quadratic ramp-up like original joint controller (not cubic)
      double normalized_time = std::max(0.0, (time_elapsed - 0.1) / (startup_duration_ - 0.1));
      startup_factor = std::min(normalized_time * normalized_time, 1.0);
    }
    
    // Exit startup phase after startup_duration_ seconds
    if (time_elapsed > startup_duration_) {
      startup_phase_ = false;
      startup_factor = 1.0;
      ROS_INFO("Dual impedance controller startup complete");
    }
  }

  // Update mode transition if active
  if (mode_transition_active_) {
    transition_time_ += period.toSec();
    if (transition_time_ >= transition_duration_) {
      mode_transition_active_ = false;
      transition_time_ = 0.0;
      ROS_INFO("Mode transition completed");
    }
  }

  Eigen::VectorXd tau_cartesian(7), tau_joint(7), tau_d(7);
  tau_cartesian.setZero();
  tau_joint.setZero();

  // Compute control ONLY for the active mode to avoid conflicts
  if (current_mode_ == CARTESIAN_IMPEDANCE) {
    // ============ CARTESIAN IMPEDANCE MODE ============
    
    // CRITICAL SAFETY CHECK: Do not apply any cartesian control until stiffness is initialized
    if (!cartesian_stiffness_initialized_) {
      // Apply only gravity compensation until dynamic reconfigure sets proper stiffness values
      Eigen::Map<Eigen::Matrix<double, 7, 1> > gravity_eigen(gravity.data());
      tau_cartesian = gravity_eigen;
    } else {
    
    // Get current pose
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());

    // Force/torque estimation
    Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_ext(robot_state.tau_ext_hat_filtered.data());
    Eigen::Matrix<double, 7, 1> tau_f;
    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
    
    // Friction model
    tau_f(0) = FI_11/(1+exp(-FI_21*(dq(0)+FI_31))) - TAU_F_CONST_1;
    tau_f(1) = FI_12/(1+exp(-FI_22*(dq(1)+FI_32))) - TAU_F_CONST_2;
    tau_f(2) = FI_13/(1+exp(-FI_23*(dq(2)+FI_33))) - TAU_F_CONST_3;
    tau_f(3) = FI_14/(1+exp(-FI_24*(dq(3)+FI_34))) - TAU_F_CONST_4;
    tau_f(4) = FI_15/(1+exp(-FI_25*(dq(4)+FI_35))) - TAU_F_CONST_5;
    tau_f(5) = FI_16/(1+exp(-FI_26*(dq(5)+FI_36))) - TAU_F_CONST_6;
    tau_f(6) = FI_17/(1+exp(-FI_27*(dq(6)+FI_37))) - TAU_F_CONST_7;

    force_torque = force_torque - jacobian_transpose_pinv * (tau_ext - tau_f);

    // Publish force/torque
    filter_step = filter_step + 1;
    if (filter_step == filter_step_) {
      geometry_msgs::WrenchStamped force_torque_msg;
      force_torque_msg.wrench.force.x = force_torque_old[0] * (1-alpha) + force_torque[0] * alpha / filter_step_;
      force_torque_msg.wrench.force.y = force_torque_old[1] * (1-alpha) + force_torque[1] * alpha / filter_step_;
      force_torque_msg.wrench.force.z = force_torque_old[2] * (1-alpha) + force_torque[2] * alpha / filter_step_;
      force_torque_msg.wrench.torque.x = force_torque_old[3] * (1-alpha) + force_torque[3] * alpha / filter_step_;
      force_torque_msg.wrench.torque.y = force_torque_old[4] * (1-alpha) + force_torque[4] * alpha / filter_step_;
      force_torque_msg.wrench.torque.z = force_torque_old[5] * (1-alpha) + force_torque[5] * alpha / filter_step_;
      pub_force_torque_.publish(force_torque_msg);
      force_torque_old = force_torque / filter_step_;
      force_torque.setZero();
      filter_step = 0;
    }

    // Publish current pose
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = position[0];
    pose_msg.pose.position.y = position[1];
    pose_msg.pose.position.z = position[2];
    pose_msg.pose.orientation.x = orientation.x();
    pose_msg.pose.orientation.y = orientation.y();
    pose_msg.pose.orientation.z = orientation.z();
    pose_msg.pose.orientation.w = orientation.w();
    pub_cartesian_pose_.publish(pose_msg);

    // Compute pose error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d_;
    double stiffness_distance = 0.04;
    error[0] = std::min(std::max(error[0], -stiffness_distance), stiffness_distance);
    error[1] = std::min(std::max(error[1], -stiffness_distance), stiffness_distance);
    error[2] = std::min(std::max(error[2], -stiffness_distance), stiffness_distance);

    // Orientation error
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
    Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
    error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

    // Compute control
    Eigen::VectorXd tau_task(7), tau_nullspace(7), null_vect(7), tau_joint_limit(7);
    Eigen::MatrixXd Null_mat = Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv;
    
    null_vect.setZero();
    for (size_t i = 0; i < 7; ++i) {
      null_vect(i) = q_d_nullspace_(i) - q(i);
    }

    // Cartesian PD control with startup smoothing
    if (startup_phase_) {
      // During startup, use very conservative cartesian control
      double conservative_factor = startup_factor * 0.01;  // Only 1% of normal gains for cartesian (was 5%)
      Eigen::Matrix<double, 6, 6> effective_cartesian_stiffness = cartesian_stiffness_ * conservative_factor;
      Eigen::Matrix<double, 6, 6> effective_cartesian_damping = cartesian_damping_ * conservative_factor;
      double effective_nullspace_stiffness = nullspace_stiffness_ * conservative_factor;
      
      tau_task << jacobian.transpose() * (-effective_cartesian_stiffness * error - effective_cartesian_damping * (jacobian * dq));
      tau_nullspace << Null_mat * (effective_nullspace_stiffness * null_vect - 2.0 * sqrt(std::max(effective_nullspace_stiffness, 1e-6)) * dq);
    } else {
      // Normal operation - full gains
      tau_task << jacobian.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
      tau_nullspace << Null_mat * (nullspace_stiffness_ * null_vect - 2.0 * sqrt(std::max(nullspace_stiffness_, 1e-6)) * dq);
    }
    
    // Joint limits
    tau_joint_limit.setZero();
    if (q(0) > 2.85)  { tau_joint_limit(0) = -10; }
    if (q(0) < -2.85) { tau_joint_limit(0) = +10; }
    if (q(1) > 1.7)   { tau_joint_limit(1) = -10; }
    if (q(1) < -1.7)  { tau_joint_limit(1) = +10; }
    if (q(2) > 2.85)  { tau_joint_limit(2) = -10; }
    if (q(2) < -2.85) { tau_joint_limit(2) = +10; }
    if (q(3) > -0.1)  { tau_joint_limit(3) = -10; }
    if (q(3) < -3.0)  { tau_joint_limit(3) = +10; }
    if (q(4) > 2.85)  { tau_joint_limit(4) = -10; }
    if (q(4) < -2.85) { tau_joint_limit(4) = +10; }
    if (q(5) > 3.7)   { tau_joint_limit(5) = -10; }
    if (q(5) < -0.1)  { tau_joint_limit(5) = +10; }
    if (q(6) > 2.8)   { tau_joint_limit(6) = -10; }
    if (q(6) < -2.8)  { tau_joint_limit(6) = +10; }

    tau_cartesian << tau_task + tau_nullspace + coriolis + tau_joint_limit;

    // Update stiffness and damping
    cartesian_stiffness_ = cartesian_stiffness_target_;
    cartesian_damping_ = cartesian_damping_target_;
    nullspace_stiffness_ = nullspace_stiffness_target_;
    Eigen::AngleAxisd aa_orientation_d(orientation_d_);
    orientation_d_ = Eigen::Quaterniond(aa_orientation_d);
    
    } // End of cartesian_stiffness_initialized_ check
  }
  
  if (current_mode_ == JOINT_IMPEDANCE) {
    // ============ JOINT IMPEDANCE MODE ============
    
    // Smooth state estimation with exponential filtering
    for (size_t i = 0; i < 7; ++i) {
      position_smoothed_[i] = alpha_q_ * q[i] + (1.0 - alpha_q_) * position_smoothed_[i];
      velocity_smoothed_[i] = alpha_dq_ * dq[i] + (1.0 - alpha_dq_) * velocity_smoothed_[i];
    }
    
    // Apply additional velocity filtering for stability
    double alpha_filter = 0.99;
    for (size_t i = 0; i < 7; i++) {
      dq_filtered_[i] = (1 - alpha_filter) * dq_filtered_[i] + alpha_filter * robot_state.dq[i];
    }

    // Trajectory interpolation
    std::array<double, 7> target_position, target_velocity;
    if (use_external_command_ && trajectory_active_) {
      interpolateTrajectory(time, target_position, target_velocity);
    } else {
      // Use current position if no trajectory
      for (size_t i = 0; i < 7; ++i) {
        target_position[i] = use_external_command_ && external_command_received_ ? 
                            q_desired_target_[i] : position_smoothed_[i];
        target_velocity[i] = 0.0;
      }
    }

    // Update interpolated positions
    for (size_t i = 0; i < 7; ++i) {
      last_interpolated_position_[i] = target_position[i];
      last_interpolated_velocity_[i] = target_velocity[i];
    }

    // Compute joint impedance control with startup smoothing
    for (size_t i = 0; i < 7; ++i) {
      double position_error = target_position[i] - position_smoothed_[i];
      double velocity_error = target_velocity[i] - velocity_smoothed_[i];
      
      if (startup_phase_) {
        // During startup, use very conservative approach - start with gravity only
        // Gradually ramp up control gains using startup_factor
        double conservative_factor = startup_factor * 0.1;  // Very conservative - only 10% of normal gains
        double effective_k = k_gains_[i] * conservative_factor;
        double effective_d = d_gains_[i] * conservative_factor;
        
        tau_joint[i] = effective_k * position_error + effective_d * velocity_error;
        
        // Very conservative torque limits during startup
        double effective_tau_limit = tau_limit_ * 0.1;  // Only 10% of normal torque limit
        tau_joint[i] = std::max(std::min(tau_joint[i], effective_tau_limit), -effective_tau_limit);
      } else {
        // Normal operation - full gains and limits
        tau_joint[i] = k_gains_[i] * position_error + d_gains_[i] * velocity_error;
        tau_joint[i] = std::max(std::min(tau_joint[i], tau_limit_), -tau_limit_);
      }
      
      // Joint limit protection
      double dist_to_upper = joint_limits_upper_[i] - position_smoothed_[i];
      double dist_to_lower = position_smoothed_[i] - joint_limits_lower_[i];
      
      if (dist_to_upper < joint_limit_margin_ && tau_joint[i] > 0.0) {
        tau_joint[i] = 0.0;
      }
      if (dist_to_lower < joint_limit_margin_ && tau_joint[i] < 0.0) {
        tau_joint[i] = 0.0;
      }
    }

    // Add coriolis compensation
    tau_joint += coriolis_factor_ * coriolis;

    // Publish torque comparison
    if (rate_trigger_()) {
      if (torques_publisher_.trylock()) {
        for (size_t i = 0; i < 7; ++i) {
          torques_publisher_.msg_.tau_commanded[i] = tau_joint[i];
          torques_publisher_.msg_.tau_measured[i] = tau_J_d[i];
          torques_publisher_.msg_.tau_error[i] = tau_joint[i] - tau_J_d[i];
        }
        // Calculate root mean square error
        double sum_squared_error = 0.0;
        for (size_t i = 0; i < 7; ++i) {
          sum_squared_error += std::pow(tau_joint[i] - tau_J_d[i], 2);
        }
        torques_publisher_.msg_.root_mean_square_error = std::sqrt(sum_squared_error / 7.0);
        torques_publisher_.unlockAndPublish();
      }
    }
  }

  // Mode blending for smooth transition
  if (mode_transition_active_) {
    // During transition, blend with previous mode's last torque
    double alpha = std::min(transition_time_ / transition_duration_, 1.0);
    
    if (current_mode_ == CARTESIAN_IMPEDANCE) {
      // Transitioning to Cartesian mode, blend from last Joint torques to current Cartesian
      tau_d = (1.0 - alpha) * last_mode_tau_d_ + alpha * tau_cartesian;
    } else {
      // Transitioning to Joint mode, blend from last Cartesian torques to current Joint  
      tau_d = (1.0 - alpha) * last_mode_tau_d_ + alpha * tau_joint;
    }
  } else {
    // No transition, use current mode's torque
    if (current_mode_ == CARTESIAN_IMPEDANCE) {
      tau_d = tau_cartesian;
    } else {
      tau_d = tau_joint;
    }
  }

  // During startup phase, prioritize gravity compensation over control torques
  if (startup_phase_) {
    // Blend control torques with gravity compensation during startup
    // Start with pure gravity compensation, gradually add control
    Eigen::Map<Eigen::Matrix<double, 7, 1> > gravity_eigen(gravity.data());
    tau_d = startup_factor * tau_d + (1.0 - startup_factor) * gravity_eigen;
  }

  // Store current torque for next possible transition (before saturation)
  if (current_mode_ == CARTESIAN_IMPEDANCE) {
    last_mode_tau_d_ = tau_cartesian;
  } else {
    last_mode_tau_d_ = tau_joint;
  }

  // Saturate torque rate to avoid discontinuities
  tau_d = saturateTorqueRate(tau_d, tau_J_d);
  
  // Set joint commands
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
  
  // Store last commanded torque for next cycle
  std::array<double, 7> gravity_compensation = model_handle_->getGravity();
  for (size_t i = 0; i < 7; ++i) {
    last_tau_d_[i] = tau_d(i) + gravity_compensation[i];
  }
}

void DualImpedanceController::modeCallback(const std_msgs::Bool::ConstPtr& msg) {
  ControlMode new_mode = msg->data ? CARTESIAN_IMPEDANCE : JOINT_IMPEDANCE;
  
  if (new_mode != current_mode_) {
    // Store current torque before switching for smooth transition
    franka::RobotState robot_state = state_handle_->getRobotState();
    Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(robot_state.tau_J_d.data());
    last_mode_tau_d_ = tau_J_d;
    
    // Switch mode
    prev_mode_ = current_mode_;
    current_mode_ = new_mode;
    mode_transition_active_ = true;
    transition_time_ = 0.0;
    
    // Reinitialize ONLY the new mode (following original controller patterns)
    if (new_mode == CARTESIAN_IMPEDANCE) {
      // Reinitialize cartesian mode exactly like the original controller
      Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      position_d_ = current_transform.translation();
      orientation_d_ = Eigen::Quaterniond(current_transform.linear());
      
      Eigen::Map<Eigen::Matrix<double, 7, 1> > q_current(robot_state.q.data());
      q_d_nullspace_ = q_current;
      
      ROS_INFO("DualImpedanceController: Switched to Cartesian Impedance Mode");
    } else {
      // Reinitialize joint mode exactly like the original controller
      if (cartesian_pose_handle_) {
        initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
        franka::RobotState pose_state = cartesian_pose_handle_->getRobotState();
        for (size_t i = 0; i < 7; ++i) {
          q_desired_target_[i] = pose_state.q[i];
          position_smoothed_[i] = pose_state.q[i];
          velocity_smoothed_[i] = pose_state.dq[i];
          last_interpolated_position_[i] = pose_state.q[i];
          last_interpolated_velocity_[i] = pose_state.dq[i];
        }
      } else {
        for (size_t i = 0; i < 7; ++i) {
          q_desired_target_[i] = robot_state.q[i];
          position_smoothed_[i] = robot_state.q[i];
          velocity_smoothed_[i] = robot_state.dq[i];
          last_interpolated_position_[i] = robot_state.q[i];
          last_interpolated_velocity_[i] = robot_state.dq[i];
        }
      }
      
      // Clear trajectory to prevent jumps
      clearTrajectory();
      
      ROS_INFO("DualImpedanceController: Switched to Joint Impedance Mode");
    }
  }
}

Eigen::Matrix<double, 7, 1> DualImpedanceController::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  
  // Use variable torque rate limits based on controller state
  double delta_tau_max = kDeltaTauMax;
  
  // For startup phase or first command, use much lower torque rate limit
  if (startup_phase_ || first_command_) {
    delta_tau_max = kDeltaTauMax * 0.05; // Only 5% of normal limit during startup (extremely conservative)
    first_command_ = false; // Clear first command flag
  }
  
  // Calculate estimated mechanical power after applying rate limiting
  double total_power = 0.0;
  Eigen::Matrix<double, 7, 1> rate_limited_tau;
  
  // First calculate rate-limited torques
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    rate_limited_tau[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max), -delta_tau_max);
    
    // P = τ * ω (torque * angular velocity)
    total_power += std::abs(rate_limited_tau[i] * dq_filtered_[i]);
  }
  
  // Apply additional scaling if power limit would be exceeded
  double power_scaling = 1.0;
  
  // Use more conservative power limit during startup phase
  double effective_power_limit = startup_phase_ ? power_limit_startup_ : power_limit_;
  
  if (total_power > effective_power_limit && total_power > 0) {
    power_scaling = effective_power_limit / total_power;
    // Ensure we never scale up, only down
    power_scaling = std::min(power_scaling, 1.0);
  }
  
  // Apply power scaling to the rate-limited torques
  for (size_t i = 0; i < 7; i++) {
    // Apply power scaling to rate_limited torque
    tau_d_saturated[i] = rate_limited_tau[i] * power_scaling;
    // Finally apply absolute limit
    tau_d_saturated[i] = std::max(std::min(tau_d_saturated[i], tau_limit_), -tau_limit_);
  }
  
  return tau_d_saturated;
}

// ============ CARTESIAN IMPEDANCE CALLBACKS ============
void DualImpedanceController::equilibriumStiffnessCallback(
    const std_msgs::Float32MultiArray::ConstPtr& stiffness_) {
  
  int i = 0;
  for(std::vector<float>::const_iterator it = stiffness_->data.begin(); it != stiffness_->data.end(); ++it) {
    stiff_[i] = *it;
    i++;
  }

  cartesian_stiffness_target_(0,0) = std::max(std::min(static_cast<double>(stiff_[0]), 4000.0), 0.0);
  cartesian_stiffness_target_(1,1) = std::max(std::min(static_cast<double>(stiff_[1]), 4000.0), 0.0);
  cartesian_stiffness_target_(2,2) = std::max(std::min(static_cast<double>(stiff_[2]), 4000.0), 0.0);

  cartesian_damping_target_(0,0) = 2.0 * sqrt(cartesian_stiffness_target_(0,0));
  cartesian_damping_target_(1,1) = 2.0 * sqrt(cartesian_stiffness_target_(1,1));
  cartesian_damping_target_(2,2) = 2.0 * sqrt(cartesian_stiffness_target_(2,2));

  cartesian_stiffness_target_(3,3) = std::max(std::min(static_cast<double>(stiff_[3]), 50.0), 0.0);
  cartesian_stiffness_target_(4,4) = std::max(std::min(static_cast<double>(stiff_[4]), 50.0), 0.0);
  cartesian_stiffness_target_(5,5) = std::max(std::min(static_cast<double>(stiff_[5]), 50.0), 0.0);

  cartesian_damping_target_(3,3) = 2.0 * sqrt(cartesian_stiffness_target_(3,3));
  cartesian_damping_target_(4,4) = 2.0 * sqrt(cartesian_stiffness_target_(4,4));
  cartesian_damping_target_(5,5) = 2.0 * sqrt(cartesian_stiffness_target_(5,5));

  nullspace_stiffness_target_ = std::max(std::min(static_cast<double>(stiff_[6]), 20.0), 0.0);

  // Publish dynamic reconfigure updates
  dynamic_reconfigure::Config set_Kx;
  dynamic_reconfigure::DoubleParameter param_X_double;
  param_X_double.name = "translational_stiffness_X";
  param_X_double.value = cartesian_stiffness_target_(0,0);
  set_Kx.doubles = {param_X_double};
  pub_stiff_update_.publish(set_Kx);

  dynamic_reconfigure::Config set_Ky;
  dynamic_reconfigure::DoubleParameter param_Y_double;
  param_Y_double.name = "translational_stiffness_Y";
  param_Y_double.value = cartesian_stiffness_target_(1,1);
  set_Ky.doubles = {param_Y_double};
  pub_stiff_update_.publish(set_Ky);

  dynamic_reconfigure::Config set_Kz;
  dynamic_reconfigure::DoubleParameter param_Z_double;
  param_Z_double.name = "translational_stiffness_Z";
  param_Z_double.value = cartesian_stiffness_target_(2,2);
  set_Kz.doubles = {param_Z_double};
  pub_stiff_update_.publish(set_Kz);

  dynamic_reconfigure::Config set_K_alpha;
  dynamic_reconfigure::DoubleParameter param_alpha_double;
  param_alpha_double.name = "rotational_stiffness_X";
  param_alpha_double.value = cartesian_stiffness_target_(3,3);
  set_K_alpha.doubles = {param_alpha_double};
  pub_stiff_update_.publish(set_K_alpha);

  dynamic_reconfigure::Config set_K_beta;
  dynamic_reconfigure::DoubleParameter param_beta_double;
  param_beta_double.name = "rotational_stiffness_Y";
  param_beta_double.value = cartesian_stiffness_target_(4,4);
  set_K_beta.doubles = {param_beta_double};
  pub_stiff_update_.publish(set_K_beta);

  dynamic_reconfigure::Config set_K_gamma;
  dynamic_reconfigure::DoubleParameter param_gamma_double;
  param_gamma_double.name = "rotational_stiffness_Z";
  param_gamma_double.value = cartesian_stiffness_target_(5,5);
  set_K_gamma.doubles = {param_gamma_double};
  pub_stiff_update_.publish(set_K_gamma);

  dynamic_reconfigure::Config set_nullspace;
  dynamic_reconfigure::DoubleParameter param_nullspace_double;
  param_nullspace_double.name = "nullspace_stiffness";
  param_nullspace_double.value = nullspace_stiffness_target_;
  set_nullspace.doubles = {param_nullspace_double};
  pub_stiff_update_.publish(set_nullspace);
}

void DualImpedanceController::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_(0,0) = config.translational_stiffness_X;
  cartesian_stiffness_target_(1,1) = config.translational_stiffness_Y;
  cartesian_stiffness_target_(2,2) = config.translational_stiffness_Z;
  cartesian_stiffness_target_(3,3) = config.rotational_stiffness_X;
  cartesian_stiffness_target_(4,4) = config.rotational_stiffness_Y;
  cartesian_stiffness_target_(5,5) = config.rotational_stiffness_Z;

  cartesian_damping_target_(0,0) = 2.0 * sqrt(config.translational_stiffness_X);
  cartesian_damping_target_(1,1) = 2.0 * sqrt(config.translational_stiffness_Y);
  cartesian_damping_target_(2,2) = 2.0 * sqrt(config.translational_stiffness_Z);
  cartesian_damping_target_(3,3) = 2.0 * sqrt(config.rotational_stiffness_X);
  cartesian_damping_target_(4,4) = 2.0 * sqrt(config.rotational_stiffness_Y);
  cartesian_damping_target_(5,5) = 2.0 * sqrt(config.rotational_stiffness_Z);
  nullspace_stiffness_target_ = config.nullspace_stiffness;
  
  // CRITICAL: Mark stiffness as initialized
  cartesian_stiffness_initialized_ = true;
}

void DualImpedanceController::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_(orientation_d_);
  orientation_d_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_.coeffs().dot(orientation_d_.coeffs()) < 0.0) {
    orientation_d_.coeffs() << -orientation_d_.coeffs();
  }
}

void DualImpedanceController::equilibriumConfigurationCallback(
    const std_msgs::Float32MultiArray::ConstPtr& joint) {
  int i = 0;
  for(std::vector<float>::const_iterator it = joint->data.begin(); it != joint->data.end(); ++it) {
    q_d_nullspace_[i] = *it;
    i++;
  }
}

// ============ JOINT IMPEDANCE METHODS ============
void DualImpedanceController::jointCommandCallback(const std_msgs::Float64MultiArrayConstPtr& msg) {
  if (msg->data.size() != 7) {
    ROS_ERROR("Joint command must have 7 elements");
    return;
  }
  
  if (!use_external_command_) {
    ROS_WARN("Received joint command but external command mode is disabled");
    return;
  }
  
  // Clear existing trajectory
  clearTrajectory();
  
  // Convert ROS message to trajectory point
  std::array<double, 7> target_position, target_velocity;
  
  for (size_t i = 0; i < 7; ++i) {
    if (is_delta_) {
      target_position[i] = last_interpolated_position_[i] + msg->data[i];
    } else {
      target_position[i] = msg->data[i];
    }
    target_velocity[i] = 0.0;
  }
  
  q_desired_target_ = target_position;
  addTrajectoryPoint(target_position, target_velocity);
  external_command_received_ = true;
}

void DualImpedanceController::addTrajectoryPoint(const std::array<double, 7>& position, 
                                                 const std::array<double, 7>& velocity) {
  TrajectoryPoint point;
  point.position = position;
  point.velocity = velocity;
  point.timestamp = ros::Time::now().toSec();
  trajectory_buffer_.push_back(point);
  trajectory_active_ = true;
  current_trajectory_index_ = 0;
}

void DualImpedanceController::clearTrajectory() {
  trajectory_buffer_.clear();
  trajectory_active_ = false;
  current_trajectory_index_ = 0;
  trajectory_completion_countdown_ = 0;
}

bool DualImpedanceController::interpolateTrajectory(const ros::Time& current_time, 
                                                   std::array<double, 7>& target_position,
                                                   std::array<double, 7>& target_velocity) {
  if (!trajectory_active_ || trajectory_buffer_.empty()) {
    return false;
  }

  // Simple trajectory execution - go to the latest point in buffer
  if (current_trajectory_index_ < trajectory_buffer_.size()) {
    const auto& point = trajectory_buffer_[current_trajectory_index_];
    target_position = point.position;
    target_velocity = point.velocity;
    
    // Check if we've reached the target
    double distance = 0.0;
    for (size_t i = 0; i < 7; ++i) {
      double diff = target_position[i] - last_interpolated_position_[i];
      distance += diff * diff;
    }
    
    if (distance < 0.001) { // Close enough threshold
      current_trajectory_index_++;
      if (current_trajectory_index_ >= trajectory_buffer_.size()) {
        trajectory_active_ = false;
      }
    }
    
    return true;
  }
  
  return false;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DualImpedanceController,
                       controller_interface::ControllerBase)
