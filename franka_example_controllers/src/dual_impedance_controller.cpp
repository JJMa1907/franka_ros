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
  is_cartesian_mode_ = true;

  // ============ ROS TOPICS AND SERVICES ============
  // Subscribers
  mode_sub_ = node_handle.subscribe(
      "/impedance_mode", 1, &DualImpedanceController::modeCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  
  sub_equilibrium_pose_ = node_handle.subscribe(
      "/equilibrium_pose", 20, &DualImpedanceController::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  
  sub_equilibrium_config_ = node_handle.subscribe(
      "/equilibrium_configuration", 20, &DualImpedanceController::equilibriumConfigurationCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  
  sub_stiffness_ = node_handle.subscribe(
      "/stiffness", 20, &DualImpedanceController::equilibriumStiffnessCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
  
  joint_command_sub_ = node_handle.subscribe(
      "/joint_command", 1, &DualImpedanceController::jointCommandCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  // Publishers
  pub_stiff_update_ = node_handle.advertise<dynamic_reconfigure::Config>(
      "/dynamic_reconfigure_compliance_param_node/parameter_updates", 5);
  
  pub_cartesian_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("/cartesian_pose", 1);
  
  pub_force_torque_ = node_handle.advertise<geometry_msgs::WrenchStamped>("/force_torque_ext", 1);
  
  pub_impedance_mode_status_ = node_handle.advertise<std_msgs::Bool>("/impedance_mode_status", 1);
  
  pub_camera_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("/camera_pose", 1);

  // ============ GRIPPER CONTROL INITIALIZATION ============
  // Gripper control subscriber
  gripper_control_sub_ = node_handle.subscribe(
      "/gripper_control", 1, &DualImpedanceController::gripperControlCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  // Initialize gripper action goal publishers
  std::string gripper_ns = "/franka_gripper";
  gripper_move_pub_ = node_handle.advertise<franka_gripper::MoveActionGoal>(
      gripper_ns + "/move/goal", 1);
  gripper_grasp_pub_ = node_handle.advertise<franka_gripper::GraspActionGoal>(
      gripper_ns + "/grasp/goal", 1);
      
  // ROS_INFO_STREAM("DualImpedanceController: Gripper publishers initialized for namespace: " << gripper_ns);

  // Initialize timer for periodic impedance mode status publishing (1Hz)
  impedance_mode_status_timer_ = node_handle.createTimer(
      ros::Duration(1.0), &DualImpedanceController::publishImpedanceModeStatus, this);

  // Dynamic reconfigure server
  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle("dynamic_reconfigure_compliance_param_node");
  dynamic_server_compliance_param_.reset(
      new dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>(
          dynamic_reconfigure_compliance_param_node_));
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&DualImpedanceController::complianceParamCallback, this, _1, _2));

  // ============ PARAMETER LOADING ============
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("DualImpedanceController: Could not read parameter arm_id");
    return false;
  }
  
  // Store arm_id for gripper control
  arm_id_ = arm_id;
  
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR("DualImpedanceController: Invalid or no joint_names parameters provided!");
    return false;
  }

  // Joint impedance parameters - 降低增益以避免force threshold错误
  std::vector<double> joint_kp = {150.0, 150.0, 150.0, 150.0, 100.0, 200.0, 80.0};  // 显著降低P增益
  std::vector<double> joint_kd = {15.0, 15.0, 15.0, 15.0, 8.0, 12.0, 6.0};          // 适度降低D增益
  
  // Trajectory interpolation parameters
  // time_fraction_ = 0.2; // up, speed up, waypoints down 
  // node_handle.getParam("time_fraction", time_fraction_);
  std::vector<double> max_delta_q = {0.05, 0.05, 0.05, 0.05, 0.06, 0.06, 0.06};
  
  // std::vector<double> max_delta_q = {0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03}; //rad per cycle default
  // std::vector<double> max_delta_q = {0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02}; //rad per cycle
  // std::vector<double> max_delta_q = {0.005, 0.005, 0.005, 0.005, 0.006, 0.006, 0.006};
  node_handle.getParam("joint_kp", joint_kp);
  node_handle.getParam("joint_kd", joint_kd);
  node_handle.getParam("max_delta_q", max_delta_q);
  // output max_delta_q for debugging
  for (size_t i = 0; i < max_delta_q.size(); ++i) {
    ROS_INFO("DualImpedanceController: Max delta position for joint %zu: %.4f rad", i, max_delta_q[i]);
  }
  // Use joint_kp and joint_kd as k_gains and d_gains
  k_gains_.resize(7);
  d_gains_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    k_gains_[i] = joint_kp[i];
    d_gains_[i] = joint_kd[i];
  }

  // State estimation parameters (exponential smoothing)
  alpha_q_ = 0.95; // 提高状态估计更新率以减少滞后
  alpha_dq_ = 0.95;
  node_handle.getParam("alpha_q", alpha_q_);
  node_handle.getParam("alpha_dq", alpha_dq_);

  
  
  // Controller startup parameters
  node_handle.getParam("startup_duration", startup_duration_);
  
  // Initialize smoothed state variables
  position_smoothed_.fill(0.0);
  velocity_smoothed_.fill(0.0);
  for (size_t i = 0; i < 7; ++i) {
    max_delta_q_[i] = max_delta_q[i];
  }

  if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
    coriolis_factor_ = 1.0;
  }

  if (!node_handle.getParam("use_external_command", use_external_command_)) {
    use_external_command_ = false;
  }

  if (!node_handle.getParam("is_delta", is_delta_)) {
    is_delta_ = false;
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
    joint_limit_margin_ = 0.1;
  }
  
  if (!node_handle.getParam("tau_limit", tau_limit_)) {
    tau_limit_ = 30.0; // 进一步降低力矩限制以避免force threshold错误
  }
  
  if (!node_handle.getParam("startup_duration", startup_duration_)) {
    startup_duration_ = 0.5;
  }

  double publish_rate(500.0);  // Increase publishing rate to maximum
  node_handle.getParam("publish_rate", publish_rate);
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

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

  // ============ CONTROLLER INITIALIZATION ============
  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();
  cartesian_stiffness_target_.setZero();
  cartesian_damping_target_.setZero();
  
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
  
  // Initialize max delta position per control cycle (from deoxys max_delta_q config)
  for (size_t i = 0; i < 7; ++i) {
    ROS_INFO("DualImpedanceController: Max delta position for joint %zu: %.4f rad", i, max_delta_q[i]);
    max_delta_position_per_cycle_[i] = max_delta_q[i]; // 0.06 rad per cycle default
    last_interpolated_position_[i] = 0.0;
    last_interpolated_velocity_[i] = 0.0;
  }
  
  // Initialize trajectory completion progress counter
  trajectory_completion_countdown_ = 0;

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);
  std::fill(velocity_filtered_.begin(), velocity_filtered_.end(), 0);
  
  // Initialize torque publisher
  torques_publisher_.init(node_handle, "torque_comparison", 1);
  
  // Initialize camera transformation from link8 to camera
  // Based on URDF: <origin xyz="0.03 -0.03 0.05" rpy="0 ${-pi/2} ${3*pi/4}" />
  Eigen::Vector3d camera_translation(0.05, -0.03, 0.05);
  Eigen::Matrix3d camera_rotation;
  double roll = M_PI/2.0;
  double pitch = 0.0;  // -pi/2
  double yaw = 0.0; // 3*pi/4
  
  // Create rotation matrix from RPY
  camera_rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                   Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  
  camera_transform_from_link8_.translation() = camera_translation;
  camera_transform_from_link8_.linear() = camera_rotation;

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
  
  // Reset startup phase for smooth transition
  startup_phase_ = true;
  startup_time_ = ros::Time(0); // Reset to zero to indicate it needs initialization
  first_command_ = true; // Reset first command flag for ramp-up
  
  // Initialize Cartesian stiffness if starting in Cartesian mode and not already initialized
  if (is_cartesian_mode_) {
    bool is_stiffness_initialized = false;
    for (int i = 0; i < 6; ++i) {
      if (cartesian_stiffness_target_(i, i) > 0.0) {
        is_stiffness_initialized = true;
        break;
      }
    }
    
    if (!is_stiffness_initialized) {
      ROS_INFO("DualImpedanceController: Starting in Cartesian mode, initializing stiffness with default values");
      initializeCartesianStiffness();
    }
  }
  
  // Publish initial mode status
  std_msgs::Bool mode_status;
  mode_status.data = is_cartesian_mode_;
  pub_impedance_mode_status_.publish(mode_status);
}

void DualImpedanceController::update(const ros::Time& time, const ros::Duration& period) {
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  Eigen::Map<Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(robot_state.tau_J_d.data());

  Eigen::VectorXd tau_d(7);

  if (is_cartesian_mode_) {
    
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

    // Publish force/torque directly without filtering to reduce delay
    geometry_msgs::WrenchStamped force_torque_msg;
    force_torque_msg.wrench.force.x = force_torque_[0]; 
    force_torque_msg.wrench.force.y = force_torque_[1];
    force_torque_msg.wrench.force.z = force_torque_[2];
    force_torque_msg.wrench.torque.x = force_torque_[3];
    force_torque_msg.wrench.torque.y = force_torque_[4];
    force_torque_msg.wrench.torque.z = force_torque_[5];
    pub_force_torque_.publish(force_torque_msg);
    force_torque_.setZero();

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
    
    // Publish camera pose (need link8 transform for camera calculation)
    std::array<double, 16> link8_array = model_handle_->getPose(franka::Frame::kFlange);
    Eigen::Affine3d link8_transform(Eigen::Matrix4d::Map(link8_array.data()));
    publishCameraPose(link8_transform);

    // Pose error computation
    Eigen::Matrix<double, 6, 1> error;
    ROS_INFO_THROTTLE(5.0, "Cartesian Pose: Position: [%.4f, %.4f, %.4f] m, Orientation: [%.4f, %.4f, %.4f] rad",
                     position[0], position[1], position[2],
                     orientation.x(), orientation.y(), orientation.z());
    error.head(3) << position - position_d_;
    
    double stiffness_distance = 0.04;
    error[0] = std::min(std::max(error[0], -stiffness_distance), stiffness_distance);
    error[1] = std::min(std::max(error[1], -stiffness_distance), stiffness_distance);
    error[2] = std::min(std::max(error[2], -stiffness_distance), stiffness_distance);

    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
      orientation.coeffs() << -orientation.coeffs();
    }
    Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
    Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
    error.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

    // Display error between current state and target in Cartesian mode
    ROS_INFO_THROTTLE(5.0, "Cartesian Mode Error - Position: [%.4f, %.4f, %.4f] m, Orientation: [%.4f, %.4f, %.4f] rad",
                     error[0], error[1], error[2], error[3], error[4], error[5]);

    Eigen::VectorXd tau_task(7), tau_nullspace(7), null_vect(7), tau_joint_limit(7);
    
    Eigen::MatrixXd Null_mat = Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv;
    
    null_vect.setZero();
    for (size_t i = 0; i < 7; ++i) {
      null_vect(i) = q_d_nullspace_(i) - q(i);
    }

    tau_task << jacobian.transpose() * (-cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq));
    // ROS_INFO_THROTTLE(5.0, "Cartesian stiffness: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", 
    //              cartesian_stiffness_(0, 0), cartesian_stiffness_(1, 1),
    //              cartesian_stiffness_(2, 2), cartesian_stiffness_(3, 3),
    //              cartesian_stiffness_(4, 4), cartesian_stiffness_(5, 5));
    // ROS_INFO_THROTTLE(5.0, "Cartesian damping: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
    //               cartesian_damping_(0, 0), cartesian_damping_(1, 1),
    //               cartesian_damping_(2, 2), cartesian_damping_(3, 3),
    //               cartesian_damping_(4, 4), cartesian_damping_(5, 5));  
    tau_nullspace << Null_mat * (nullspace_stiffness_ * null_vect - 
                                 2.0 * sqrt(nullspace_stiffness_) * dq);
    
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
  //     {
  //     ROS_INFO_THROTTLE(5.0, "Cartesian Joint torques (tau_d): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
  //              tau_d[0], tau_d[1], tau_d[2], tau_d[3], tau_d[4], tau_d[5], tau_d[6]);
  // }
    tau_d = saturateTorqueRate(tau_d, tau_J_d);
    
    cartesian_stiffness_ = cartesian_stiffness_target_;
    cartesian_damping_ = cartesian_damping_target_;
    nullspace_stiffness_ = nullspace_stiffness_target_;
    ROS_INFO_THROTTLE(5.0, "Cartesian stiffness target: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                 cartesian_stiffness_target_(0, 0), cartesian_stiffness_target_(1, 1),
                 cartesian_stiffness_target_(2, 2), cartesian_stiffness_target_(3, 3),
                 cartesian_stiffness_target_(4, 4), cartesian_stiffness_target_(5, 5));
    ROS_INFO_THROTTLE(5.0, "Cartesian damping target: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                  cartesian_damping_target_(0, 0), cartesian_damping_target_(1, 1),
                  cartesian_damping_target_(2, 2), cartesian_damping_target_(3, 3),
                  cartesian_damping_target_(4, 4), cartesian_damping_target_(5, 5));

    Eigen::AngleAxisd aa_orientation_d(orientation_d_);
    orientation_d_ = Eigen::Quaterniond(aa_orientation_d);
    
  } else {
    // Joint impedance mode
    double startup_factor = 1.0;
    
    if (startup_phase_) {
      if (startup_time_.isZero()) {
        startup_time_ = time;
        for (size_t i = 0; i < 7; ++i) {
          initial_position_[i] = robot_state.q[i];
        }
      }
      
      double time_elapsed = (time - startup_time_).toSec();
      double normalized_time = time_elapsed / startup_duration_;
      startup_factor = std::min(normalized_time * normalized_time, 1.0);
      
      if (time_elapsed > startup_duration_) {
        startup_phase_ = false;
        startup_factor = 1.0;
      }
    }

    for (size_t i = 0; i < 7; i++) {
      position_smoothed_[i] = alpha_q_ * robot_state.q[i] + (1.0 - alpha_q_) * position_smoothed_[i];
      velocity_smoothed_[i] = alpha_dq_ * robot_state.dq[i] + (1.0 - alpha_dq_) * velocity_smoothed_[i];
    }
    
    double alpha = 0.90; // 降低滤波系数，提高响应性
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
    
    // Publish camera pose (use link8 transform which is same as end-effector in this case)
    publishCameraPose(transform);
    
    // Compute joint impedance control
    
    // Debug output for joint mode without rate limiting
    //pos_error 
    std::array<double, 7> pos_error;
    for (size_t i = 0; i < 7; ++i) {
      double q_target = 0.0;
      double dq_target = 0.0;
      
      // Use smoothed positions for control (deoxys-compatible state estimation)
      double current_q = position_smoothed_[i];
      double current_dq = velocity_smoothed_[i];
      
      if (use_external_command_ && external_command_received_) {
        // Use trajectory interpolation for external commands (deoxys-compatible)
        // if (isTrajectoryActive()) {
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
          ROS_INFO_THROTTLE(1.0, "Interpolated joint %zu: q_target = %.3f, dq_target = %.3f", i, q_target, dq_target);
        // } 
        // else {
        //   // No active trajectory, use the desired target position
        //   q_target = q_desired_target_[i];
        //   dq_target = 0.0;
        //   ROS_INFO_THROTTLE(1.0, "Using external command for joint %zu: q_target = %.3f", i, q_target);
        // }
      }
      else if (use_external_command_ && !external_command_received_) {
        // When external command mode is enabled but no command received yet,
        // maintain current position
        q_target = current_q;
        dq_target = 0.0;
        ROS_INFO_THROTTLE(1.0, "No external command received for joint %zu, maintaining position: q_target = %.3f", i, q_target);
      }
      else {
        // Use simple joint targets
        q_target = q_d_array_[i];
        dq_target = 0.0;
        ROS_INFO_THROTTLE(1.0, "Using joint target for joint %zu: q_target = %.3f", i, q_target);
      }
      
      // Calculate position error (deoxys-compatible)

      double position_error = q_target - current_q;
      pos_error[i] = position_error;

      // PD control calculation with reduced gains during startup
      double effective_k = startup_phase_ ? k_gains_[i] * startup_factor : k_gains_[i];
      double effective_d = startup_phase_ ? d_gains_[i] * startup_factor : d_gains_[i];
      
      // Calculate the control torque with scaled gains
      double control_torque = effective_k * position_error - effective_d * current_dq;
      
      // Limit the maximum torque magnitude to tau_limit_
      control_torque = std::max(std::min(control_torque, tau_limit_), -tau_limit_);
      
      tau_d_calculated[i] = control_torque;
      
      double dist_to_upper = joint_limits_upper_[i] - current_q;
      double dist_to_lower = current_q - joint_limits_lower_[i];
      
      if (dist_to_upper < joint_limit_margin_ && tau_d_calculated[i] > 0.0) {
        tau_d_calculated[i] = 0.0;
      }
      if (dist_to_lower < joint_limit_margin_ && tau_d_calculated[i] < 0.0) {
        tau_d_calculated[i] = 0.0;
      }
      
      tau_d_calculated[i] += coriolis_factor_ * coriolis[i];
    }
    
    // Convert std::array to Eigen::Matrix for saturateTorqueRate
    Eigen::Matrix<double, 7, 1> tau_d_eigen;
    for (size_t i = 0; i < 7; ++i) {
      tau_d_eigen[i] = tau_d_calculated[i];
    }
    
    Eigen::Matrix<double, 7, 1> tau_d_saturated = saturateTorqueRate(tau_d_eigen, tau_J_d);

    for (size_t i = 0; i < 7; ++i) {
      tau_d[i] = tau_d_saturated[i];
    }    
  }

  // Set joint commands
  for (size_t i = 0; i < 7; ++i) {
    ROS_INFO_THROTTLE(5.0, "Joint torques (tau_d): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
               tau_d[0], tau_d[1], tau_d[2], tau_d[3], tau_d[4], tau_d[5], tau_d[6]);
    joint_handles_[i].setCommand(tau_d[i]);
  }

  // Publish torque data - always publish without rate limiting
  if (torques_publisher_.trylock()) {
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
  if (msg->data != is_cartesian_mode_) {
    // Get current robot state for smooth transition
    franka::RobotState robot_state = state_handle_->getRobotState();
    
    is_cartesian_mode_ = msg->data;
    
    // Reset force/torque estimation variables for smooth transition
    force_torque_.setZero();
    force_torque_old_.setZero();
    filter_step_ = 0;
    
    // When switching to Cartesian mode, set current pose as target
    if (is_cartesian_mode_) {
      // Set current pose as equilibrium point to prevent motion
      Eigen::Affine3d current_transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      position_d_ = current_transform.translation();
      orientation_d_ = Eigen::Quaterniond(current_transform.linear());
      
      // Also update nullspace target to current joint configuration
      Eigen::Map<Eigen::Matrix<double, 7, 1>> q_current(robot_state.q.data());
      q_d_nullspace_ = q_current;
      
      bool is_stiffness_initialized = false;
      for (int i = 0; i < 6; ++i) {
        if (cartesian_stiffness_target_(i, i) > 0.0) {
          is_stiffness_initialized = true;
          break;
        }
      }
      
      if (!is_stiffness_initialized) {
        ROS_INFO("DualImpedanceController: Switching to Cartesian mode, initializing stiffness with default values");
        initializeCartesianStiffness();
      }
      
      ROS_INFO("DualImpedanceController: Switched to Cartesian mode, current pose set as target");
    } else {
      // When switching to Joint mode, set current joint positions as target
      for (size_t i = 0; i < 7; ++i) {
        q_d_array_[i] = robot_state.q[i];
        q_desired_target_[i] = robot_state.q[i];
        // Update smoothed positions to current positions for smooth transition
        position_smoothed_[i] = robot_state.q[i];
        velocity_smoothed_[i] = robot_state.dq[i];
        last_interpolated_position_[i] = robot_state.q[i];
        last_interpolated_velocity_[i] = robot_state.dq[i];
        initial_position_[i] = robot_state.q[i];
        // Reset filtered velocities
        dq_filtered_[i] = robot_state.dq[i];
      }
      
      // Reset startup phase for smooth transition
      startup_phase_ = true;
      startup_time_ = ros::Time(0); // Reset to zero to indicate it needs initialization
      
      external_command_received_ = true;
      
      ROS_INFO("DualImpedanceController: Switched to Joint mode, current joint positions set as target");
    }
    
    ROS_INFO("DualImpedanceController: Switched to %s mode", 
             is_cartesian_mode_ ? "Cartesian" : "Joint");
    
    // Publish mode status
    std_msgs::Bool mode_status;
    mode_status.data = is_cartesian_mode_;
    pub_impedance_mode_status_.publish(mode_status);
  }
}

void DualImpedanceController::initializeCartesianStiffness() {
  // Set reasonable default stiffness values (same as dynamic reconfigure defaults)
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_(0,0) = 200.0;  // Default translational stiffness X
  cartesian_stiffness_target_(1,1) = 200.0;  // Default translational stiffness Y  
  cartesian_stiffness_target_(2,2) = 200.0;  // Default translational stiffness Z
  cartesian_stiffness_target_(3,3) = 80.0;   // Default rotational stiffness X
  cartesian_stiffness_target_(4,4) = 80.0;   // Default rotational stiffness Y
  cartesian_stiffness_target_(5,5) = 80.0;   // Default rotational stiffness Z
  
  // Set corresponding damping (critical damping)
  cartesian_damping_target_(0,0) = 2.0 * sqrt(200.0);
  cartesian_damping_target_(1,1) = 2.0 * sqrt(200.0);
  cartesian_damping_target_(2,2) = 2.0 * sqrt(200.0);
  cartesian_damping_target_(3,3) = 2.0 * sqrt(80.0);
  cartesian_damping_target_(4,4) = 2.0 * sqrt(80.0);
  cartesian_damping_target_(5,5) = 2.0 * sqrt(80.0);
  
  // Set nullspace stiffness
  nullspace_stiffness_target_ = 0.0; // Default from dynamic reconfigure
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

std::array<double, 7> DualImpedanceController::interpolateTrajectory(
    double current_time, 
    std::array<double, 7>& target_velocity) {

  // Output q_desired_target_ for debugging
  // ROS_INFO_THROTTLE(1.0, "q_desired_target_: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
  //          q_desired_target_[0], q_desired_target_[1], q_desired_target_[2],
  //          q_desired_target_[3], q_desired_target_[4], q_desired_target_[5],
  //          q_desired_target_[6]);
  
  std::array<double, 7> interpolated_position;
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_current(robot_state.q.data());
  
  // Simple interpolation: based on max step limits and position error
  bool target_reached = true;
  double max_position_error = 0.0;
    
  // 控制参数
  const double control_frequency = 1000.0; // 1kHz
  const double position_tolerance = 0.002; // 减小到2毫弧度
  const double velocity_tolerance = 0.005; // 速度容差
  
  for (size_t i = 0; i < 7; ++i) {
    double position_error = q_desired_target_[i] - last_interpolated_position_[i];
    double max_step = max_delta_position_per_cycle_[i];
    
    // 计算到目标的绝对误差
    double error_abs = std::abs(position_error);
    
    // 超保守的平滑插值 - 避免force threshold错误
    double delta_position = 0.0;
    
    if (error_abs < position_tolerance) {
      // 非常接近目标时，完全停止
      delta_position = 0.0;
      target_velocity[i] = 0.0;
    }
    else if (error_abs < 0.05) {
      // 在50毫弧度内，渐进式减速（10%-50%最大步长）
      double scale_factor = 0.1 + 0.4 * (error_abs - 0.01) / (0.05 - 0.01);
      double scaled_step = max_step * scale_factor;
      delta_position = std::max(std::min(position_error, scaled_step), -scaled_step);
    }
    else {
      // 远距离时，使用限制步长（最大50%标准步长）
      double conservative_step = max_step * 0.5;
      delta_position = std::max(std::min(position_error, conservative_step), -conservative_step);
    }
    
    // 额外的速度变化率限制 - 防止突然的速度变化
    double previous_velocity = last_interpolated_velocity_[i];
    double proposed_velocity = delta_position * control_frequency;
    
    // 限制加速度：最大速度变化率 = 5 rad/s²
    double max_accel = 5.0; // rad/s²
    double max_velocity_change = max_accel / control_frequency; // per cycle
    
    if (std::abs(proposed_velocity - previous_velocity) > max_velocity_change) {
      // 限制速度变化，重新计算位置变化
      double limited_velocity = previous_velocity + 
        std::max(std::min(proposed_velocity - previous_velocity, max_velocity_change), -max_velocity_change);
      delta_position = limited_velocity / control_frequency;
    }
    
    // 更新插值位置
    interpolated_position[i] = last_interpolated_position_[i] + delta_position;
    
    // 速度计算 - 渐进式衰减
    double raw_velocity = delta_position * control_frequency;
    
    // 根据误差大小应用不同的速度衰减
    if (error_abs < 0.005) {
      raw_velocity *= (error_abs / 0.005); // 在5毫弧度内线性衰减
    } else if (error_abs < 0.02) {
      raw_velocity *= (0.3 + 0.7 * (error_abs - 0.005) / (0.02 - 0.005)); // 在20毫弧度内衰减到30%
    }
    
    // 超强速度滤波 - 减少抖动和突变
    double velocity_alpha = (error_abs > 0.02) ? 0.2 : 0.05; // 接近目标时极强滤波
    velocity_filtered_[i] = velocity_alpha * raw_velocity + (1.0 - velocity_alpha) * velocity_filtered_[i];
    target_velocity[i] = velocity_filtered_[i];
    
    // Debug output for joint 0 to see interpolation progress
    if (i == 0) {
      ROS_INFO_THROTTLE(2.0, "Joint 0 ultra-smooth: last=%.4f, target=%.4f, error=%.6f, delta=%.6f, new=%.4f, vel=%.4f", 
                       last_interpolated_position_[i], q_desired_target_[i], error_abs, delta_position, interpolated_position[i], target_velocity[i]);
    }
    
    // Check if target is reached - 使用原始位置误差而非插值误差
    double final_error = std::abs(q_desired_target_[i] - interpolated_position[i]);
    max_position_error = std::max(max_position_error, final_error);
    
    // 更严格的收敛条件：位置精度 + 速度接近零
    if (final_error > position_tolerance || std::abs(target_velocity[i]) > velocity_tolerance) {
      target_reached = false;
    }
  }
  
  // Trajectory completion logic
  if (target_reached) {
    trajectory_active_ = false;
    // ROS_INFO("DualImpedanceController: Trajectory completed with smooth interpolation (max_error: %.6f)", max_position_error);
  }
  
  // Update state
  last_interpolated_position_ = interpolated_position;
  last_interpolated_velocity_ = target_velocity;
  
  return interpolated_position;
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
  
  // Debug output to track commands
  // ROS_INFO_THROTTLE(0.5, "Received joint command: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
  //                  msg->data[0], msg->data[1], msg->data[2], msg->data[3],
  //                  msg->data[4], msg->data[5], msg->data[6]);
  
  
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
  
  // Debug output to validate target position calculation
  // ROS_INFO_THROTTLE(0.5, "Generated target position: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
  //                  target_position[0], target_position[1], target_position[2], target_position[3],
  //                  target_position[4], target_position[5], target_position[6]);
  
  // Save the target position for when the trajectory is complete
  q_desired_target_ = target_position;
  
  // Add trajectory point with full trajectory interpolation (deoxys-compatible)
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

void DualImpedanceController::equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  if(!is_cartesian_mode_) {
    return;
  }

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

void DualImpedanceController::publishImpedanceModeStatus(const ros::TimerEvent& /*event*/) {
  std_msgs::Bool mode_status;
  mode_status.data = is_cartesian_mode_;
  pub_impedance_mode_status_.publish(mode_status);
}

void DualImpedanceController::publishCameraPose(const Eigen::Affine3d& link8_transform) {
  // Calculate camera pose by applying transformation from link8 to camera
  Eigen::Affine3d camera_transform = link8_transform * camera_transform_from_link8_;
  
  // Extract position and orientation
  Eigen::Vector3d camera_position = camera_transform.translation();
  Eigen::Quaterniond camera_orientation(camera_transform.linear());
  
  // Publish camera pose
  geometry_msgs::PoseStamped camera_pose_msg;
  camera_pose_msg.header.frame_id = "panda_link0";
  camera_pose_msg.header.stamp = ros::Time::now();
  camera_pose_msg.pose.position.x = camera_position[0];
  camera_pose_msg.pose.position.y = camera_position[1];
  camera_pose_msg.pose.position.z = camera_position[2];
  camera_pose_msg.pose.orientation.x = camera_orientation.x();
  camera_pose_msg.pose.orientation.y = camera_orientation.y();
  camera_pose_msg.pose.orientation.z = camera_orientation.z();
  camera_pose_msg.pose.orientation.w = camera_orientation.w();
  pub_camera_pose_.publish(camera_pose_msg);
}

void DualImpedanceController::gripperControlCallback(const std_msgs::Float64MultiArrayConstPtr& msg) {
  // Expected message format: [position, speed, force]
  // position: 0-1 (normalized) or 0-0.08 (meters), speed: m/s, force: N (optional, -1 for position mode)
  
  if (msg->data.size() < 2) {
    ROS_ERROR("DualImpedanceController: Gripper control message must have at least 2 elements [position, speed]. Optionally 3rd element for force.");
    return;
  }
  
  double position = msg->data[0];
  double speed = msg->data[1];
  double force = (msg->data.size() >= 3) ? msg->data[2] : -1.0; // Default to position mode
  
  // Validate parameters
  if (speed <= 0.0) {
    ROS_WARN("DualImpedanceController: Speed must be positive, using default 0.1 m/s");
    speed = 0.1;
  }
  
  // Call gripper control function
  controlGripper(position, speed, force);
}


bool DualImpedanceController::controlGripper(double position, double speed, double force) {
  try {
    // Convert position to meters if normalized (0-1 range)
    double width_meters;
    if (position >= 0.0 && position <= 1.0) {
      // Convert normalized position to meters (0.08m is typical max width for Franka)
      width_meters = position * 0.08;
    } else {
      // Use position directly as meters
      width_meters = position;
    }
    
    // Clamp width to valid range
    width_meters = std::max(0.0, std::min(0.08, width_meters));
    
    ROS_INFO_THROTTLE(5.0, "DualImpedanceController: Gripper command: width=%.4fm, speed=%.3fm/s, force=%.1fN", 
             width_meters, speed, force);
    
    // Choose between move and grasp based on force parameter
    if (force < 0.0) {
      // Simple positioning movement
      franka_gripper::MoveActionGoal action_goal;
      action_goal.header.stamp = ros::Time::now();
      action_goal.goal.width = width_meters;
      action_goal.goal.speed = speed;
      
      gripper_move_pub_.publish(action_goal);
      
    } else {
      // Grasping movement with force
      franka_gripper::GraspActionGoal action_goal;
      action_goal.header.stamp = ros::Time::now();
      action_goal.goal.width = width_meters;
      action_goal.goal.speed = speed;
      action_goal.goal.force = force;
      // Set default epsilon values for grasp tolerance
      action_goal.goal.epsilon.inner = 0.005;  // 5mm inner tolerance
      action_goal.goal.epsilon.outer = 0.005;  // 5mm outer tolerance
      
      ROS_INFO("DualImpedanceController: Publishing grasp command to gripper: %.4fm at %.3fm/s with %.1fN force", 
               width_meters, speed, force);
      gripper_grasp_pub_.publish(action_goal);
    }
    
    return true;
    
  } catch (const std::exception& e) {
    ROS_ERROR("DualImpedanceController: Error controlling gripper: %s", e.what());
    return false;
  }
}
}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::DualImpedanceController,
                       controller_interface::ControllerBase)