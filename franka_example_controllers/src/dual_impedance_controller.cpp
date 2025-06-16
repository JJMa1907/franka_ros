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

  // ============ JOINT IMPEDANCE ENHANCED PARAMETERS ============
  // Initialize joint stiffness and damping with default values from deoxys config
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

  // State estimation parameters (exponential smoothing)
  alpha_q_ = 0.9;
  alpha_dq_ = 0.9;
  node_handle.getParam("alpha_q", alpha_q_);
  node_handle.getParam("alpha_dq", alpha_dq_);

  // Trajectory interpolation parameters
  time_fraction_ = 1.0;
  node_handle.getParam("time_fraction", time_fraction_);
  
  // Controller startup parameters
  node_handle.getParam("startup_duration", startup_duration_);
  
  // Initialize smoothed state variables
  position_smoothed_.fill(0.0);
  velocity_smoothed_.fill(0.0);
  max_delta_q_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    max_delta_q_[i] = max_delta_q[i];
  }

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("DualImpedanceController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  // Check if external command mode should be enabled
  if (!node_handle.getParam("use_external_command", use_external_command_)) {
    ROS_INFO_STREAM("DualImpedanceController: use_external_command not found. Defaulting to "
                    << use_external_command_);
  }

  // Check if delta mode should be enabled (deoxys is_delta config)
  if (!node_handle.getParam("is_delta", is_delta_)) {
    ROS_INFO_STREAM("DualImpedanceController: is_delta not found. Defaulting to "
                    << is_delta_);
  }

  // Initialize joint limits (deoxys-compatible)
  // Default Franka Panda joint limits
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
    ROS_INFO_STREAM("DualImpedanceController: joint_limit_margin not found. Defaulting to "
                    << joint_limit_margin_);
  }

  // Load power and torque limits for power_limit_violation prevention
  if (!node_handle.getParam("power_limit", power_limit_)) {
    ROS_INFO_STREAM("DualImpedanceController: power_limit not found. Defaulting to "
                    << power_limit_);
  }
  
  // Set more conservative power limit during startup
  power_limit_startup_ = power_limit_ * 0.7; // 70% of normal power limit during startup
  if (!node_handle.getParam("power_limit_startup", power_limit_startup_)) {
    ROS_INFO_STREAM("DualImpedanceController: power_limit_startup not found. Defaulting to "
                    << power_limit_startup_);
  }
  
  if (!node_handle.getParam("tau_limit", tau_limit_)) {
    ROS_INFO_STREAM("DualImpedanceController: tau_limit not found. Defaulting to "
                    << tau_limit_);
  }
  
  if (!node_handle.getParam("startup_duration", startup_duration_)) {
    ROS_INFO_STREAM("DualImpedanceController: startup_duration not found. Defaulting to "
                    << startup_duration_);
  }

  // Initialize rate trigger for publishing
  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("DualImpedanceController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

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
  
  // Enhanced cartesian impedance features from cartesian_impedance_example_controller
  sub_stiffness_ = node_handle.subscribe(
      "/stiffness", 20, &DualImpedanceController::equilibriumStiffnessCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  pub_stiff_update_ = node_handle.advertise<dynamic_reconfigure::Config>(
      "/dynamic_reconfigure_compliance_param_node/parameter_updates", 5);

  pub_cartesian_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("/cartesian_pose", 1);

  pub_force_torque_ = node_handle.advertise<geometry_msgs::WrenchStamped>("/force_torque_ext", 1);

  // Initialize dynamic reconfigure for cartesian mode
  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_.reset(
      new dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>(
          dynamic_reconfigure_compliance_param_node_));
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&DualImpedanceController::complianceParamCallback, this, _1, _2));

  // Publisher for current cartesian pose
  pub_cartesian_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("/cartesian_pose", 1);

  // Initialize Joint impedance parameters exactly like original
  joint_command_sub_ = node_handle.subscribe(
      "/joint_command", 1, &DualImpedanceController::jointCommandCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  // Set default parameters exactly like original controllers
  cartesian_stiffness_.setZero();  // 改为零矩阵，更安全
  cartesian_damping_.setZero();    // 改为零矩阵，更安全
  
  // Initialize enhanced cartesian variables
  cartesian_stiffness_target_.setZero();  // 改为零矩阵初始化
  cartesian_damping_target_.setZero();    // 改为零矩阵初始化
  
  // Initialize force/torque estimation variables
  force_torque_.setZero();
  force_torque_old_.setZero();
  stiff_.setZero();
  
  // Initialize positions
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  q_d_nullspace_.setZero();
  q_d_array_.fill(0.0);

  // Initialize external command variables for joint impedance
  for (size_t i = 0; i < 7; ++i) {
    q_desired_target_[i] = 0.0;  // Will be set to current position in starting()
  }

  external_command_received_ = false;
  trajectory_active_ = false;
  current_trajectory_index_ = 0;
  trajectory_start_time_ = 0.0;
  trajectory_buffer_.clear();
  
  // Initialize max delta position per control cycle (from deoxys max_delta_q config)
  for (size_t i = 0; i < 7; ++i) {
    max_delta_position_per_cycle_[i] = max_delta_q[i]; // 0.06 rad per cycle default
    last_interpolated_position_[i] = 0.0;
    last_interpolated_velocity_[i] = 0.0;
  }
  
  // Initialize trajectory completion progress counter
  trajectory_completion_countdown_ = 0;

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);
  
  // Initialize torque publisher
  torques_publisher_.init(node_handle, "torque_comparison", 1);

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
  
  // Initialize force/torque estimation variables
  force_torque_old_.setZero();
  
  // Initialize joint targets
  for (size_t i = 0; i < 7; ++i) {
    q_d_array_[i] = initial_state.q[i];
    q_desired_target_[i] = initial_state.q[i];
    position_smoothed_[i] = initial_state.q[i];
    velocity_smoothed_[i] = initial_state.dq[i];
    last_interpolated_position_[i] = initial_state.q[i];
    last_interpolated_velocity_[i] = initial_state.dq[i];
    initial_position_[i] = initial_state.q[i];
  }
  
  // Initialize trajectory interpolation
  trajectory_start_time_ = time.toSec();
  trajectory_active_ = false;
  current_trajectory_index_ = 0;
  trajectory_completion_countdown_ = 0;
  clearTrajectory();
  
  // Reset startup phase for smooth transition
  startup_phase_ = true;
  startup_time_ = ros::Time(0); // Reset to zero to indicate it needs initialization
  first_command_ = true; // Reset first command flag for ramp-up
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
    // ============ ENHANCED CARTESIAN IMPEDANCE MODE ============
    // Enhanced implementation based on cartesian_impedance_example_controller
    
    std::array<double, 49> mass_array = model_handle_->getMass();
    Eigen::Map<Eigen::Matrix<double, 7, 7> > mass(mass_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_ext(robot_state.tau_ext_hat_filtered.data());
    
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());
    
    // Friction compensation for force/torque estimation
    Eigen::Matrix<double, 7, 1> tau_f;
    tau_f(0) = FI_11/(1+exp(-FI_21*(dq(0)+FI_31))) - TAU_F_CONST_1;
    tau_f(1) = FI_12/(1+exp(-FI_22*(dq(1)+FI_32))) - TAU_F_CONST_2;
    tau_f(2) = FI_13/(1+exp(-FI_23*(dq(2)+FI_33))) - TAU_F_CONST_3;
    tau_f(3) = FI_14/(1+exp(-FI_24*(dq(3)+FI_34))) - TAU_F_CONST_4;
    tau_f(4) = FI_15/(1+exp(-FI_25*(dq(4)+FI_35))) - TAU_F_CONST_5;
    tau_f(5) = FI_16/(1+exp(-FI_26*(dq(5)+FI_36))) - TAU_F_CONST_6;
    tau_f(6) = FI_17/(1+exp(-FI_27*(dq(6)+FI_37))) - TAU_F_CONST_7;

    // Force/torque estimation using pseudoinverse
    Eigen::MatrixXd jacobian_transpose_pinv;
    franka_example_controllers::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
    
    force_torque_ = force_torque_ - jacobian_transpose_pinv * (tau_ext - tau_f);

    // Publish filtered force/torque
    filter_step_++;
    if (filter_step_ == filter_step_max_) {
      geometry_msgs::WrenchStamped force_torque_msg;
      force_torque_msg.wrench.force.x = force_torque_old_[0] * (1 - alpha_force_) + force_torque_[0] * alpha_force_ / filter_step_max_;
      force_torque_msg.wrench.force.y = force_torque_old_[1] * (1 - alpha_force_) + force_torque_[1] * alpha_force_ / filter_step_max_;
      force_torque_msg.wrench.force.z = force_torque_old_[2] * (1 - alpha_force_) + force_torque_[2] * alpha_force_ / filter_step_max_;
      force_torque_msg.wrench.torque.x = force_torque_old_[3] * (1 - alpha_force_) + force_torque_[3] * alpha_force_ / filter_step_max_;
      force_torque_msg.wrench.torque.y = force_torque_old_[4] * (1 - alpha_force_) + force_torque_[4] * alpha_force_ / filter_step_max_;
      force_torque_msg.wrench.torque.z = force_torque_old_[5] * (1 - alpha_force_) + force_torque_[5] * alpha_force_ / filter_step_max_;
      pub_force_torque_.publish(force_torque_msg);
      force_torque_old_ = force_torque_ / filter_step_max_;
      force_torque_.setZero();
      filter_step_ = 0;
    }

    // Publish current cartesian pose
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose.position.x = position[0];
    pose_msg.pose.position.y = position[1];
    pose_msg.pose.position.z = position[2];
    pose_msg.pose.orientation.x = orientation.x();
    pose_msg.pose.orientation.y = orientation.y();
    pose_msg.pose.orientation.z = orientation.z();
    pose_msg.pose.orientation.w = orientation.w();
    pub_cartesian_pose_.publish(pose_msg);

    // Enhanced pose error computation with clamping
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d_;
    
    // Apply stiffness distance clamping
    double stiffness_distance = 0.04;
    error[0] = std::min(std::max(error[0], -stiffness_distance), stiffness_distance);
    error[1] = std::min(std::max(error[1], -stiffness_distance), stiffness_distance);
    error[2] = std::min(std::max(error[2], -stiffness_distance), stiffness_distance);

    // Orientation error exactly like original
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
    Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
    error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

    // Compute control with enhanced nullspace handling
    Eigen::VectorXd tau_task(7), tau_nullspace(7), null_vect(7), tau_joint_limit(7);
    
    Eigen::MatrixXd Null_mat = Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv;
    
    null_vect.setZero();
    for (size_t i = 0; i < 7; ++i) {
      null_vect(i) = q_d_nullspace_(i) - q(i);
    }

    // Cartesian PD control with enhanced damping
    tau_task << jacobian.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
    
    // Nullspace PD control with double critical damping
    tau_nullspace << Null_mat * (nullspace_stiffness_ * null_vect - 
                                 2.0 * sqrt(nullspace_stiffness_) * dq);
    
    // Enhanced joint limit protection
    tau_joint_limit.setZero();
    if (q(0) > 2.85)   { tau_joint_limit(0) = -10; }
    if (q(0) < -2.85)  { tau_joint_limit(0) = +10; }
    if (q(1) > 1.7)    { tau_joint_limit(1) = -10; }
    if (q(1) < -1.7)   { tau_joint_limit(1) = +10; }
    if (q(2) > 2.85)   { tau_joint_limit(2) = -10; }
    if (q(2) < -2.85)  { tau_joint_limit(2) = +10; }
    if (q(3) > -0.1)   { tau_joint_limit(3) = -10; }
    if (q(3) < -3.0)   { tau_joint_limit(3) = +10; }
    if (q(4) > 2.85)   { tau_joint_limit(4) = -10; }
    if (q(4) < -2.85)  { tau_joint_limit(4) = +10; }
    if (q(5) > 3.7)    { tau_joint_limit(5) = -10; }
    if (q(5) < -0.1)   { tau_joint_limit(5) = +10; }
    if (q(6) > 2.8)    { tau_joint_limit(6) = -10; }
    if (q(6) < -2.8)   { tau_joint_limit(6) = +10; }
    
    tau_d << tau_task + tau_nullspace + coriolis + tau_joint_limit;
    
    // Apply torque rate saturation for cartesian mode
    tau_d = saturateTorqueRate(tau_d, tau_J_d);
    // Update cartesian parameters for real-time adjustment (critical for dynamic reconfigure and stiffness updates)
    cartesian_stiffness_ = cartesian_stiffness_target_;
    cartesian_damping_ = cartesian_damping_target_;
    nullspace_stiffness_ = nullspace_stiffness_target_;
    
    // Ensure orientation continuity (from original cartesian controller)
    Eigen::AngleAxisd aa_orientation_d(orientation_d_);
    orientation_d_ = Eigen::Quaterniond(aa_orientation_d);
    
  } else {
    // ============ JOINT IMPEDANCE MODE - ENHANCED ============ 
    // Enhanced joint impedance with all features from joint_impedance_example_controller
    
    // Prevent the shaking issue when starting the controller by implementing a smooth startup
    double startup_factor = 1.0;
    
    if (startup_phase_) {
      // Initialize on first update cycle
      if (startup_time_.isZero()) {
        startup_time_ = time;
        for (size_t i = 0; i < 7; ++i) {
          initial_position_[i] = robot_state.q[i];
        }
        ROS_INFO("Joint impedance controller starting with smooth ramp-up over %.1f seconds", startup_duration_);
      }
      
      // Calculate time elapsed since controller start
      double time_elapsed = (time - startup_time_).toSec();
      
      // Use a quadratic ramp-up instead of linear for even smoother transition
      double normalized_time = time_elapsed / startup_duration_;
      startup_factor = std::min(normalized_time * normalized_time, 1.0);
      
      // Exit startup phase after startup_duration_ seconds
      if (time_elapsed > startup_duration_) {
        startup_phase_ = false;
        startup_factor = 1.0;
        ROS_INFO("Joint impedance controller startup complete");
      }
    }

    // Apply exponential smoothing to measurements for state estimation
    for (size_t i = 0; i < 7; i++) {
      position_smoothed_[i] = alpha_q_ * robot_state.q[i] + (1.0 - alpha_q_) * position_smoothed_[i];
      velocity_smoothed_[i] = alpha_dq_ * robot_state.dq[i] + (1.0 - alpha_dq_) * velocity_smoothed_[i];
    }
    
    double alpha = 0.99;
    for (size_t i = 0; i < 7; i++) {
      dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
    }

    std::array<double, 7> tau_d_calculated;
    
    // First, publish current cartesian pose (even in joint mode)
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());
    
    // Publish current pose
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "panda_link0";
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.position.x = position[0];
    pose_msg.pose.position.y = position[1];
    pose_msg.pose.position.z = position[2];
    pose_msg.pose.orientation.x = orientation.x();
    pose_msg.pose.orientation.y = orientation.y();
    pose_msg.pose.orientation.z = orientation.z();
    pose_msg.pose.orientation.w = orientation.w();
    pub_cartesian_pose_.publish(pose_msg);
    
    // Compute joint impedance control
    for (size_t i = 0; i < 7; ++i) {
      double q_target = 0.0;
      double dq_target = 0.0;
      
      // Use smoothed positions for control (deoxys-compatible state estimation)
      double current_q = position_smoothed_[i];
      double current_dq = velocity_smoothed_[i];
      
      if (use_external_command_ && external_command_received_) {
        // Use trajectory interpolation for external commands (deoxys-compatible)
        if (isTrajectoryActive()) {
          // Only call interpolateTrajectory once per update cycle and store the results
          static std::array<double, 7> interpolated_position_cache;
          static std::array<double, 7> target_velocity_cache;
          
          // First joint iteration - calculate interpolation for all joints
          if (i == 0) {
            target_velocity_cache = {};
            interpolated_position_cache = interpolateTrajectory(ros::Time::now().toSec(), target_velocity_cache);
          }
          
          // Use the cached values
          q_target = interpolated_position_cache[i];
          dq_target = target_velocity_cache[i];
        } else {
          // No active trajectory, use the desired target position
          q_target = q_desired_target_[i];
          dq_target = 0.0;
        }
      }
      else if (use_external_command_ && !external_command_received_) {
        // When external command mode is enabled but no command received yet,
        // maintain current position
        q_target = current_q;
        dq_target = 0.0;
      }
      else {
        // Use simple joint targets
        q_target = q_d_array_[i];
        dq_target = 0.0;
      }
      
      // Calculate position error (deoxys-compatible)
      double position_error = q_target - current_q;
      
      // PD control calculation with reduced gains during startup
      double effective_k = startup_phase_ ? k_gains_[i] * startup_factor : k_gains_[i];
      double effective_d = startup_phase_ ? d_gains_[i] * startup_factor : d_gains_[i];
      
      // Calculate the control torque with scaled gains
      double control_torque = effective_k * position_error - effective_d * current_dq;
      
      // Limit the maximum torque magnitude to tau_limit_
      control_torque = std::max(std::min(control_torque, tau_limit_), -tau_limit_);
      
      // Final commanded torque
      tau_d_calculated[i] = control_torque;
      
      // Joint limit protection (deoxys-compatible)
      double dist_to_upper = joint_limits_upper_[i] - current_q;
      double dist_to_lower = current_q - joint_limits_lower_[i];
      
      if (dist_to_upper < joint_limit_margin_ && tau_d_calculated[i] > 0.0) {
        tau_d_calculated[i] = 0.0;  // Disable positive torque near upper limit
      }
      if (dist_to_lower < joint_limit_margin_ && tau_d_calculated[i] < 0.0) {
        tau_d_calculated[i] = 0.0;  // Disable negative torque near lower limit
      }
      
      // Add coriolis compensation
      tau_d_calculated[i] += coriolis_factor_ * coriolis[i];
    }

    // Enhanced torque rate saturation for joint impedance mode
    std::array<double, 7> tau_d_saturated = saturateTorqueRateJoint(tau_d_calculated, robot_state.tau_J_d);

    // Set joint commands for joint mode
    for (size_t i = 0; i < 7; ++i) {
      joint_handles_[i].setCommand(tau_d_saturated[i]);
      tau_d[i] = tau_d_saturated[i]; // Store for last_tau_d_ update
    }
  }

  // ============ COMMON OPERATIONS FOR BOTH MODES ============
  
  // Publish torque comparison data (enhanced from joint_impedance_example_controller)
  if (rate_trigger_() && torques_publisher_.trylock()) {
    std::array<double, 7> tau_j = robot_state.tau_J;
    std::array<double, 7> tau_error;
    double error_rms(0.0);
    for (size_t i = 0; i < 7; ++i) {
      tau_error[i] = last_tau_d_[i] - tau_j[i];
      error_rms += std::sqrt(std::pow(tau_error[i], 2.0)) / 7.0;
    }
    torques_publisher_.msg_.root_mean_square_error = error_rms;
    for (size_t i = 0; i < 7; ++i) {
      torques_publisher_.msg_.tau_commanded[i] = last_tau_d_[i];
      torques_publisher_.msg_.tau_error[i] = tau_error[i];
      torques_publisher_.msg_.tau_measured[i] = tau_j[i];
    }
    torques_publisher_.unlockAndPublish();
  }

  // Update last commanded torques (common for both modes)
  std::array<double, 7> gravity = model_handle_->getGravity();
  for (size_t i = 0; i < 7; ++i) {
    last_tau_d_[i] = tau_d[i] + gravity[i];
  }
}

void DualImpedanceController::modeCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data != is_cartesian_mode_) {is_cartesian_mode_ = msg->data;
  ROS_INFO("DualImpedanceController: Switched to %s mode", 
           is_cartesian_mode_ ? "Cartesian" : "Joint");
  }
  
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

// ============ ENHANCED JOINT IMPEDANCE FUNCTIONS ============
// Enhanced saturation for joint impedance mode (from joint_impedance_example_controller)
std::array<double, 7> DualImpedanceController::saturateTorqueRateJoint(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {
  std::array<double, 7> tau_d_saturated{};
  
  // Use variable torque rate limits based on controller state
  double delta_tau_max = kDeltaTauMax;
  
  // For startup phase or first command, use much lower torque rate limit
  if (startup_phase_ || first_command_) {
    delta_tau_max = kDeltaTauMax * 0.4; // 40% of normal limit during startup
    first_command_ = false; // Clear first command flag
  }
  
  // Calculate estimated mechanical power after applying rate limiting
  double total_power = 0.0;
  std::array<double, 7> rate_limited_tau;
  
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

// Trajectory interpolation methods (deoxys-compatible implementation)
void DualImpedanceController::addTrajectoryPoint(
    const std::array<double, 7>& position, 
    const std::array<double, 7>& velocity) {
  
  TrajectoryPoint point;
  point.position = position;
  point.velocity = velocity;
  point.timestamp = ros::Time::now().toSec();
  
  trajectory_buffer_.push_back(point);
  
  // Start trajectory if this is the first point
  if (!trajectory_active_) {
    trajectory_active_ = true;
    trajectory_start_time_ = point.timestamp;
    current_trajectory_index_ = 0;
    
    // Initialize interpolated position to current smoothed robot position, not target position
    // This ensures trajectory interpolation starts from current position and gradually moves to target
    // We use position_smoothed_ which is the current filtered robot position
    for (size_t i = 0; i < 7; ++i) {
      last_interpolated_position_[i] = position_smoothed_[i];
      last_interpolated_velocity_[i] = velocity_smoothed_[i];
    }
  }
  
  // Limit buffer size to prevent memory issues
  const size_t max_buffer_size = 100;
  if (trajectory_buffer_.size() > max_buffer_size) {
    trajectory_buffer_.erase(trajectory_buffer_.begin());
  }
}

std::array<double, 7> DualImpedanceController::interpolateTrajectory(
    double current_time, 
    std::array<double, 7>& target_velocity) {
  
  std::array<double, 7> interpolated_position = last_interpolated_position_;
  target_velocity = last_interpolated_velocity_;
  
  if (!trajectory_active_ || trajectory_buffer_.empty()) {
    return interpolated_position;
  }
  
  // Apply time_fraction scaling (deoxys-compatible)
  double scaled_time = (current_time - trajectory_start_time_) * time_fraction_;
  double target_time = trajectory_start_time_ + scaled_time;
  
  // Find the current target point
  if (current_trajectory_index_ >= trajectory_buffer_.size()) {
    // We've already reached the end of the trajectory
    trajectory_active_ = false;
    return interpolated_position;
  }
  
  const TrajectoryPoint& target_point = trajectory_buffer_[current_trajectory_index_];
  
  // LINEAR_JOINT_POSITION interpolation implementation with improved velocity profile
  for (size_t i = 0; i < 7; ++i) {
    double position_error = target_point.position[i] - last_interpolated_position_[i];
    double error_abs = std::abs(position_error);
    
    // Apply max_delta_q constraint (from deoxys config)
    double max_delta = max_delta_position_per_cycle_[i];
    
    // Apply a velocity damping factor as we approach the target
    // This creates a smoother deceleration profile
    const double approach_threshold = 0.1; // radians
    double damping_factor = 1.0;
    
    if (error_abs < approach_threshold) {
      // Gradually reduce velocity as we get closer to the target
      // This prevents overshoot and oscillation
      damping_factor = error_abs / approach_threshold;
      damping_factor = std::max(0.2, damping_factor); // Don't slow down too much
    }
    
    double delta_position = std::max(std::min(position_error, max_delta * damping_factor), 
                                    -max_delta * damping_factor);
    
    // Linear interpolation with velocity constraint
    interpolated_position[i] = last_interpolated_position_[i] + delta_position;
    target_velocity[i] = delta_position / 0.001; // Control cycle is 1ms
    
    // Apply velocity smoothing with lower limit near target
    double vel_limit = 0.5;
    if (error_abs < approach_threshold) {
      vel_limit = 0.5 * damping_factor;
    }
    
    target_velocity[i] = std::max(std::min(target_velocity[i], vel_limit), -vel_limit);
  }
  
  // Check if we've reached the target point (within tolerance)
  bool reached_target = true;
  const double position_tolerance = 0.01; // 0.01 radians (about 0.57 degrees)
  const double velocity_tolerance = 0.05; // 0.05 rad/s - ensure we're also slowing down
  
  double max_pos_error = 0.0;
  double max_vel = 0.0;
  
  for (size_t i = 0; i < 7; ++i) {
    double error = std::abs(interpolated_position[i] - target_point.position[i]);
    double vel = std::abs(target_velocity[i]);
    max_pos_error = std::max(max_pos_error, error);
    max_vel = std::max(max_vel, vel);
    
    if (error > position_tolerance) {
      reached_target = false;
      break;
    }
  }
  
  // Only consider target reached if velocity is also low enough
  // This prevents oscillation around the target
  if (reached_target && max_vel > velocity_tolerance) {
    reached_target = false;
  }
  
  // Only advance if we've reached the target
  if (reached_target) {
    // We've reached the target, move to next point
    current_trajectory_index_++;
    
    if (current_trajectory_index_ >= trajectory_buffer_.size()) {
      // We've reached the last point, but don't deactivate trajectory yet
      // This ensures the final position is properly held for one more cycle
      
      // If this is the final point, let's hold it stable for a bit longer
      // to ensure the robot settles completely
      trajectory_completion_countdown_ = 0; // Reset countdown to start hold period
    } else {
      // When moving to a new target, gradually ramp up velocity again
      // Reset damping for the next target
      trajectory_completion_countdown_ = 0;
    }
  } else {
    // If we're on the last point and been trying for a while, consider forcing completion
    if (current_trajectory_index_ == (trajectory_buffer_.size() - 1)) {
      const int max_attempts = 300; // Reduced from 500ms to 300ms of attempts
      const int hold_period = 100;  // Hold completed position for 100ms
      
      if (trajectory_completion_countdown_ > max_attempts + hold_period) {
        current_trajectory_index_++;
        trajectory_completion_countdown_ = 0;
      } else if (trajectory_completion_countdown_ > max_attempts) {
        // We're in the hold period, ensure we maintain position
        // without making further adjustments
        for (size_t i = 0; i < 7; ++i) {
          target_velocity[i] = 0.0; // Zero velocity during hold period
        }
        trajectory_completion_countdown_++;
      } else {
        trajectory_completion_countdown_++;
      }
    }
  }
  
  // Update last interpolated state
  last_interpolated_position_ = interpolated_position;
  last_interpolated_velocity_ = target_velocity;
  
  // If trajectory is at end but still active, deactivate it now
  if (current_trajectory_index_ >= trajectory_buffer_.size() && trajectory_active_) {
    // Instead of an abrupt deactivation, gradually transition
    // by continuing to use the last interpolated position
    
    // Only deactivate after a short "settling period"
    static int deactivation_count = 0;
    const int settle_cycles = 50; // 50ms settling time
    
    if (deactivation_count > settle_cycles) {
      trajectory_active_ = false;
      deactivation_count = 0;
      
      // Update target positions to match final position from trajectory
      if (!trajectory_buffer_.empty()) {
        q_desired_target_ = trajectory_buffer_.back().position;
      }
    } else {
      deactivation_count++;
    }
  }
  
  return interpolated_position;
}

void DualImpedanceController::clearTrajectory() {
  trajectory_buffer_.clear();
  trajectory_active_ = false;
  trajectory_completion_countdown_ = 0;
  current_trajectory_index_ = 0;
}

bool DualImpedanceController::isTrajectoryActive() const {
  return trajectory_active_ && !trajectory_buffer_.empty();
}

// Joint command callback - Primary deoxys-compatible interface
// Handles both single-point and multi-point commands through unified trajectory interpolation
void DualImpedanceController::jointCommandCallback(
    const std_msgs::Float64MultiArrayConstPtr& msg) {
  if (msg->data.size() != 7) {
    ROS_ERROR("Joint command must have 7 elements");
    return;
  }
  
  if (!use_external_command_) {
    ROS_WARN("Received joint command but external command mode is disabled");
    return;
  }
  
  // Clear existing trajectory (deoxys-compatible behavior)
  clearTrajectory();
  
  // Convert ROS message to trajectory point with full trajectory processing
  std::array<double, 7> target_position, target_velocity;
  
  for (size_t i = 0; i < 7; ++i) {
    if (is_delta_) {
      // Delta mode: add to current position (deoxys-compatible)
      target_position[i] = last_interpolated_position_[i] + msg->data[i];
    } else {
      // Absolute mode: use direct position
      target_position[i] = msg->data[i];
    }
    target_velocity[i] = 0.0; // Default velocity for single point commands
  }
  
  // Save the target position for when the trajectory is complete
  q_desired_target_ = target_position;
  
  // Add trajectory point with full trajectory interpolation (deoxys-compatible)
  addTrajectoryPoint(target_position, target_velocity);
  external_command_received_ = true;
  
  // Auto-switch to joint mode when receiving joint commands in cartesian mode
  if (is_cartesian_mode_) {
    is_cartesian_mode_ = false;
    ROS_INFO("DualImpedanceController: Auto-switched to Joint mode due to joint command");
  }
}

// ============ ENHANCED CARTESIAN IMPEDANCE CALLBACKS ============

void DualImpedanceController::equilibriumStiffnessCallback(
    const std_msgs::Float32MultiArray::ConstPtr& stiffness) {

  int i = 0;
  // Read all stiffness values from the message
  for(std::vector<float>::const_iterator it = stiffness->data.begin(); it != stiffness->data.end(); ++it) {
    if (i < 7) {
      stiff_[i] = *it;
      i++;
    }
  }

  // Set translational stiffness with limits (0-4000)
  cartesian_stiffness_target_(0,0) = std::max(std::min(stiff_[0], 4000.0f), 0.0f);
  cartesian_stiffness_target_(1,1) = std::max(std::min(stiff_[1], 4000.0f), 0.0f);
  cartesian_stiffness_target_(2,2) = std::max(std::min(stiff_[2], 4000.0f), 0.0f);

  // Calculate corresponding damping (critical damping)
  cartesian_damping_target_(0,0) = 2.0 * sqrt(cartesian_stiffness_target_(0,0));
  cartesian_damping_target_(1,1) = 2.0 * sqrt(cartesian_stiffness_target_(1,1));
  cartesian_damping_target_(2,2) = 2.0 * sqrt(cartesian_stiffness_target_(2,2));

  // Set rotational stiffness with limits (0-50)
  cartesian_stiffness_target_(3,3) = std::max(std::min(stiff_[3], 50.0f), 0.0f);
  cartesian_stiffness_target_(4,4) = std::max(std::min(stiff_[4], 50.0f), 0.0f);
  cartesian_stiffness_target_(5,5) = std::max(std::min(stiff_[5], 50.0f), 0.0f);

  // Calculate corresponding rotational damping
  cartesian_damping_target_(3,3) = 2.0 * sqrt(cartesian_stiffness_target_(3,3));
  cartesian_damping_target_(4,4) = 2.0 * sqrt(cartesian_stiffness_target_(4,4));
  cartesian_damping_target_(5,5) = 2.0 * sqrt(cartesian_stiffness_target_(5,5));

  // Set nullspace stiffness with limits (0-20)
  nullspace_stiffness_target_ = std::max(std::min(stiff_[6], 20.0f), 0.0f);

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
}

// Enhanced equilibrium pose callback with orientation continuity
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
    if (i < 7) {
      q_d_nullspace_[i] = *it;
      i++;
    }
  }
}
}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DualImpedanceController,
                       controller_interface::ControllerBase)