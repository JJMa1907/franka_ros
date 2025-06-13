// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_impedance_example_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_example_controllers {

bool JointImpedanceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }
  if (!node_handle.getParam("radius", radius_)) {
    ROS_INFO_STREAM(
        "JointImpedanceExampleController: No parameter radius, defaulting to: " << radius_);
  }
  if (std::fabs(radius_) < 0.005) {
    ROS_INFO_STREAM("JointImpedanceExampleController: Set radius to small, defaulting to: " << 0.1);
    radius_ = 0.1;
  }

  if (!node_handle.getParam("vel_max", vel_max_)) {
    ROS_INFO_STREAM(
        "JointImpedanceExampleController: No parameter vel_max, defaulting to: " << vel_max_);
  }
  if (!node_handle.getParam("acceleration_time", acceleration_time_)) {
    ROS_INFO_STREAM(
        "JointImpedanceExampleController: No parameter acceleration_time, defaulting to: "
        << acceleration_time_);
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "JointImpedanceExampleController: Invalid or no joint_names parameters provided, aborting "
        "controller init!");
    return false;
  }

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

  // Initialize smoothed state variables
  position_smoothed_.fill(0.0);
  velocity_smoothed_.fill(0.0);
  max_delta_q_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    max_delta_q_[i] = max_delta_q[i];
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("JointImpedanceExampleController: publish_rate not found. Defaulting to "
                    << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    ROS_INFO_STREAM("JointImpedanceExampleController: coriolis_factor not found. Defaulting to "
                    << coriolis_factor_);
  }

  // Check if external command mode should be enabled
  if (!node_handle.getParam("use_external_command", use_external_command_)) {
    ROS_INFO_STREAM("JointImpedanceExampleController: use_external_command not found. Defaulting to "
                    << use_external_command_);
  }

  // Check if delta mode should be enabled (deoxys is_delta config)
  if (!node_handle.getParam("is_delta", is_delta_)) {
    ROS_INFO_STREAM("JointImpedanceExampleController: is_delta not found. Defaulting to "
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
    ROS_INFO_STREAM("JointImpedanceExampleController: joint_limit_margin not found. Defaulting to "
                    << joint_limit_margin_);
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // Always get the state interface for robot state access
  if (use_external_command_) {
    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
      ROS_ERROR_STREAM(
          "JointImpedanceExampleController: Error getting state interface from hardware");
      return false;
    }
    try {
      state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
          state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointImpedanceExampleController: Exception getting state handle from interface: "
          << ex.what());
      return false;
    }
  }

  // Only claim Cartesian interface if external command mode is disabled
  if (!use_external_command_) {
    auto* cartesian_pose_interface = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
    if (cartesian_pose_interface == nullptr) {
      ROS_ERROR_STREAM(
          "JointImpedanceExampleController: Error getting cartesian pose interface from hardware");
      return false;
    }
    try {
      cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
          cartesian_pose_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointImpedanceExampleController: Exception getting cartesian pose handle from interface: "
          << ex.what());
      return false;
    }
  } else {
    ROS_INFO("JointImpedanceExampleController: External command mode enabled - skipping Cartesian interface");
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "JointImpedanceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "JointImpedanceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
  torques_publisher_.init(node_handle, "torque_comparison", 1);
  joint_command_sub_ = node_handle.subscribe(
    "joint_command", 1, &JointImpedanceExampleController::jointCommandCallback, this);

  // Initialize external command variables
  for (size_t i = 0; i < 7; ++i) {
    q_desired_target_[i] = 0.0;  // Will be set to current position in starting()
  }

  external_command_received_ = false;

  // Initialize trajectory interpolation variables
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

  return true;
}

// Joint command callback - Primary deoxys-compatible interface
// Handles both single-point and multi-point commands through unified trajectory interpolation
void JointImpedanceExampleController::jointCommandCallback(
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
  
  // Log the received joint command (keep this for user troubleshooting)
  ROS_INFO("Received joint command: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
           target_position[0], target_position[1], target_position[2],
           target_position[3], target_position[4], target_position[5], target_position[6]);
  
  // Save the target position for when the trajectory is complete
  q_desired_target_ = target_position;
  
  // Add trajectory point with full trajectory interpolation (deoxys-compatible)
  addTrajectoryPoint(target_position, target_velocity);
  external_command_received_ = true;
  
  ROS_DEBUG("Added trajectory point to buffer. Buffer size: %zu", trajectory_buffer_.size());
}

void JointImpedanceExampleController::starting(const ros::Time& time) {
  if (cartesian_pose_handle_) {
    initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
    
    // Initialize target positions to current joint positions
    franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();
    for (size_t i = 0; i < 7; ++i) {
      q_desired_target_[i] = robot_state.q[i];
      position_smoothed_[i] = robot_state.q[i];
      velocity_smoothed_[i] = robot_state.dq[i];
      last_interpolated_position_[i] = robot_state.q[i];
      last_interpolated_velocity_[i] = robot_state.dq[i];
    }
  } else {
    // In external command mode, initialize target positions to current joint positions
    franka::RobotState robot_state = state_handle_->getRobotState();
    for (size_t i = 0; i < 7; ++i) {
      q_desired_target_[i] = robot_state.q[i];
      position_smoothed_[i] = robot_state.q[i];
      velocity_smoothed_[i] = robot_state.dq[i];
      last_interpolated_position_[i] = robot_state.q[i];
      last_interpolated_velocity_[i] = robot_state.dq[i];
    }
    ROS_INFO("JointImpedanceExampleController: Starting in external command mode");
  }
  
  // Initialize trajectory interpolation
  trajectory_start_time_ = time.toSec();
  trajectory_active_ = false;
  current_trajectory_index_ = 0;
  trajectory_completion_countdown_ = 0;
  clearTrajectory();
}

void JointImpedanceExampleController::update(const ros::Time& /*time*/,
                                             const ros::Duration& period) {
  franka::RobotState robot_state;
  if (cartesian_pose_handle_) {
    robot_state = cartesian_pose_handle_->getRobotState();
  } else {
    robot_state = state_handle_->getRobotState();
  }
  
  std::array<double, 7> coriolis = model_handle_->getCoriolis();
  std::array<double, 7> gravity = model_handle_->getGravity();

  if (!use_external_command_ && cartesian_pose_handle_)
  {
    if (vel_current_ < vel_max_) {
      vel_current_ += period.toSec() * std::fabs(vel_max_ / acceleration_time_);
    }
    vel_current_ = std::fmin(vel_current_, vel_max_);

    angle_ += period.toSec() * vel_current_ / std::fabs(radius_);
    if (angle_ > 2 * M_PI) {
      angle_ -= 2 * M_PI;
    }

    double delta_y = radius_ * (1 - std::cos(angle_));
    double delta_z = radius_ * std::sin(angle_);

    std::array<double, 16> pose_desired = initial_pose_;
    pose_desired[13] += delta_y;
    pose_desired[14] += delta_z;
    cartesian_pose_handle_->setCommand(pose_desired);
  }
  // Note: When using external command mode, we only use joint torque control
  // and don't send Cartesian pose commands to avoid conflicts
  
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
  
  // Debug counter for periodic logging
  static int debug_counter = 0;
  
  for (size_t i = 0; i < 7; ++i) {

    double q_target = 0.0;
    double dq_target = 0.0;
    
    // Use smoothed positions for control (deoxys-compatible state estimation)
    double current_q = position_smoothed_[i];
    double current_dq = velocity_smoothed_[i];
    
  // Debug: Print trajectory status for the first joint only
    if (i == 0 && (debug_counter % 1000 == 0)) {
      ROS_INFO("Debug: use_external_command_=%s, external_command_received_=%s, isTrajectoryActive()=%s",
               use_external_command_ ? "true" : "false",
               external_command_received_ ? "true" : "false",
               isTrajectoryActive() ? "true" : "false");
      ROS_INFO("Debug: trajectory_buffer_.size()=%zu, trajectory_active_=%s",
               trajectory_buffer_.size(), trajectory_active_ ? "true" : "false");
    }
    
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
      // Internal trajectory mode
      q_target = robot_state.q_d[i];
      dq_target = robot_state.dq_d[i];
    }
    
    // Calculate position error (deoxys-compatible)
    double position_error = q_target - current_q;
    
    // PD control calculation (matching deoxys equation)
    tau_d_calculated[i] = k_gains_[i] * position_error - d_gains_[i] * current_dq;
    
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

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  // Debug: Print joint targets, current positions, and torques periodically
  if (use_external_command_ && external_command_received_ && (debug_counter % 1000 == 0)) {
    std::array<double, 7> control_targets;
    
    // Let's get the actual control targets we're using (either from trajectory interpolation or desired targets)
    if (isTrajectoryActive()) {
      std::array<double, 7> dummy_velocity;
      control_targets = interpolateTrajectory(ros::Time::now().toSec(), dummy_velocity);
    } else {
      control_targets = q_desired_target_;
    }
    
    ROS_INFO("Desired target: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
             q_desired_target_[0], q_desired_target_[1], q_desired_target_[2], 
             q_desired_target_[3], q_desired_target_[4], q_desired_target_[5], q_desired_target_[6]);
             
    ROS_INFO("Control target: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
             control_targets[0], control_targets[1], control_targets[2], 
             control_targets[3], control_targets[4], control_targets[5], control_targets[6]);
             
    ROS_INFO("Current joints: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", 
             robot_state.q[0], robot_state.q[1], robot_state.q[2], 
             robot_state.q[3], robot_state.q[4], robot_state.q[5], robot_state.q[6]);
             
    ROS_INFO("Joint errors: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
             control_targets[0] - robot_state.q[0], control_targets[1] - robot_state.q[1], 
             control_targets[2] - robot_state.q[2], control_targets[3] - robot_state.q[3],
             control_targets[4] - robot_state.q[4], control_targets[5] - robot_state.q[5], 
             control_targets[6] - robot_state.q[6]);
             
    ROS_INFO("Calculated torques: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
             tau_d_calculated[0], tau_d_calculated[1], tau_d_calculated[2],
             tau_d_calculated[3], tau_d_calculated[4], tau_d_calculated[5], tau_d_calculated[6]);
             
    ROS_INFO("Saturated torques: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
             tau_d_saturated[0], tau_d_saturated[1], tau_d_saturated[2],
             tau_d_saturated[3], tau_d_saturated[4], tau_d_saturated[5], tau_d_saturated[6]);
  }
  debug_counter++;

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }

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

  for (size_t i = 0; i < 7; ++i) {
    last_tau_d_[i] = tau_d_saturated[i] + gravity[i];
  }
}

std::array<double, 7> JointImpedanceExampleController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}

// Trajectory interpolation methods (deoxys-compatible implementation)
void JointImpedanceExampleController::addTrajectoryPoint(
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

std::array<double, 7> JointImpedanceExampleController::interpolateTrajectory(
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
  
  // LINEAR_JOINT_POSITION interpolation implementation
  for (size_t i = 0; i < 7; ++i) {
    double position_error = target_point.position[i] - last_interpolated_position_[i];
    
    // Apply max_delta_q constraint (from deoxys config)
    double max_delta = max_delta_position_per_cycle_[i];
    double delta_position = std::max(std::min(position_error, max_delta), -max_delta);
    
    // Linear interpolation with velocity constraint
    interpolated_position[i] = last_interpolated_position_[i] + delta_position;
    target_velocity[i] = delta_position / 0.001; // Control cycle is 1ms
    
    // Apply velocity smoothing
    target_velocity[i] = std::max(std::min(target_velocity[i], 0.5), -0.5); // rad/s limit
  }
  
  // Check if we've reached the target point (within tolerance)
  bool reached_target = true;
  const double position_tolerance = 0.01; // 0.01 radians (about 0.57 degrees)
  
  // Debug: Log tolerance check details for first interpolation call
  static bool first_call = true;
  
  for (size_t i = 0; i < 7; ++i) {
    double error = std::abs(interpolated_position[i] - target_point.position[i]);
    
    // Debug logging for first call
    if (first_call) {
      if (i == 0) {
        ROS_INFO("First interpolation call - tolerance check:");
      }
      ROS_INFO("Joint %zu: interpolated=%.6f, target=%.6f, error=%.6f, tolerance=%.6f", 
               i, interpolated_position[i], target_point.position[i], error, position_tolerance);
    }
    
    // Actual tolerance check logic
    if (error > position_tolerance) {
      reached_target = false;
      break;
    }
  }
  
  // Only advance if we've reached the target
  if (reached_target) {
    // We've reached the target, move to next point
    current_trajectory_index_++;
    
    if (current_trajectory_index_ >= trajectory_buffer_.size()) {
      // We've reached the last point, but don't deactivate trajectory yet
      // This ensures the final position is properly held for one more cycle
    }
  } else {
    // If we're on the last point and been trying for a while, consider forcing completion
    if (current_trajectory_index_ == (trajectory_buffer_.size() - 1)) {
      const int max_attempts = 500; // Allow 500ms of attempts (500 control cycles)
      if (trajectory_completion_countdown_ > max_attempts) {
        current_trajectory_index_++;
        trajectory_completion_countdown_ = 0;
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
    trajectory_active_ = false;
    
    // Update target positions to match final position from trajectory
    if (!trajectory_buffer_.empty()) {
      q_desired_target_ = trajectory_buffer_.back().position;
    }
  }
  
  return interpolated_position;
}

void JointImpedanceExampleController::clearTrajectory() {
  trajectory_buffer_.clear();
  trajectory_active_ = false;
  trajectory_completion_countdown_ = 0;
  current_trajectory_index_ = 0;
}

bool JointImpedanceExampleController::isTrajectoryActive() const {
  
  //ROS info for debugging trajectory status
  // ROS_INFO("isTrajectoryActive() called: trajectory_active_ = %s, buffer size = %zu",
    // trajectory_active_ ? "true" : "false", trajectory_buffer_.empty() ? 0 : trajectory_buffer_.size());
  return trajectory_active_ && !trajectory_buffer_.empty();
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceExampleController,
                       controller_interface::ControllerBase)
