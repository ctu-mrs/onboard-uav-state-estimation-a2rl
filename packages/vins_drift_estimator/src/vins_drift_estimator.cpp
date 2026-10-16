/* includes //{*/
#include "vins_drift_estimator.h"
#include "git_info.hpp"
//}

namespace vins_drift_estimator
{

/* onInit() //{*/
void VinsDriftEstimator::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[VinsDriftEstimator]: initializing");

  mrs_lib::ParamLoader param_loader(nh_, "VinsDriftEstimator");

  param_loader.loadParam("uav_name", uav_name_);
  
  double main_timer_rate;
  param_loader.loadParam("main_timer_rate", main_timer_rate);
  param_loader.loadParam("use_velocity_in_drift_model", use_velocity_in_drift_model_);
  param_loader.loadParam("disable_velocity_before_tracker_activation", disable_velocity_before_tracker_activation_);
  param_loader.loadParam("drifting_odom_buffer_size", drifting_odom_buffer_size_);
  
  param_loader.loadParam("stop_using_hw_api", stop_using_hw_api_);
  param_loader.loadParam("do_velocity_correction_x", do_velocity_correction_x_);
  param_loader.loadParam("do_velocity_correction_y", do_velocity_correction_y_);
  param_loader.loadParam("do_velocity_correction_z", do_velocity_correction_z_);
  
  double mrs_uav_status_timer_duration;
  param_loader.loadParam("mrs_uav_status_timer_duration", mrs_uav_status_timer_duration);

  // | ----------------- check valid measurement ---------------- |
  param_loader.loadParam("check_valid_measurement/minimal_fusable_z", minimal_fusable_z_); 
  param_loader.loadParam("check_valid_measurement/gate_size_threshold", gate_size_threshold_);
  param_loader.loadParam("check_valid_measurement/gate_size_threshold_xy_correction_noise", gate_size_threshold_xy_correction_noise_);

  // | ------------------ transformation frames ----------------- |
  std::string output_frame = "";
  std::string output_frame_for_drifting_odom = "";
  std::string drift_corrected_frame_id = "";
  param_loader.loadParam("transform_corrected_odom", transform_corrected_odom_);
  param_loader.loadParam("output_frame", output_frame);
  param_loader.loadParam("output_frame_for_drifting_odom", output_frame_for_drifting_odom);
  param_loader.loadParam("drift_corrected_frame_id", drift_corrected_frame_id);
  
  // | ----------- initialize process noise covariance ---------- |
  lkf_x_.Q_ = Q_t::Zero();
  lkf_y_.Q_ = Q_t::Zero();
  lkf_z_.Q_ = Q_t::Zero();
  lkf_yaw_.Q_ = Q_t::Zero();
  double pos_noise_x, pos_noise_y, pos_noise_z, vel_noise_x, vel_noise_y, vel_noise_z, yaw_noise, yaw_rate_noise;
  param_loader.loadParam("process_noise/pos_x", pos_noise_x);
  param_loader.loadParam("process_noise/pos_y", pos_noise_y);
  param_loader.loadParam("process_noise/pos_z", pos_noise_z);
  param_loader.loadParam("process_noise/yaw", yaw_noise);
  param_loader.loadParam("process_noise/vel_x", vel_noise_x);
  param_loader.loadParam("process_noise/vel_y", vel_noise_y);
  param_loader.loadParam("process_noise/vel_z", vel_noise_z);
  param_loader.loadParam("process_noise/yaw_rate", yaw_rate_noise);
  param_loader.loadParam("process_noise/x_friction", x_friction_);
  param_loader.loadParam("process_noise/y_friction", y_friction_);
  param_loader.loadParam("process_noise/z_friction", z_friction_);
  param_loader.loadParam("process_noise/yaw_friction", yaw_friction_);
  
  setQ(pos_noise_x, pos_noise_y, pos_noise_z, vel_noise_x, vel_noise_y, vel_noise_z, yaw_noise, yaw_rate_noise); 
  ROS_INFO_STREAM("[VinsDriftEstimator]: Q for x set to: \n" << lkf_x_.Q_);
  ROS_INFO_STREAM("[VinsDriftEstimator]: Q for y set to: \n" << lkf_y_.Q_);
  ROS_INFO_STREAM("[VinsDriftEstimator]: Q for z set to: \n" << lkf_z_.Q_);
  ROS_INFO_STREAM("[VinsDriftEstimator]: Q for yaw set to: \n" << lkf_yaw_.Q_);
 
  // | -------------------- correction noise -------------------- |
  std::string correction_noise_method_name;
  param_loader.loadParam("correction_noise/method", correction_noise_method_name);
  correction_noise_method_ = mapCorrectionNoiseMethodName(correction_noise_method_name);
  if(correction_noise_method_ < 0) {
    ROS_ERROR_STREAM("[VinsDriftEstimator]: invalid correction noise method: " << correction_noise_method_name);
    ros::shutdown();
  }
  
  // correction noise parameter - x
  param_loader.loadParam("correction_noise/x/single_value_correction_noise", correction_noise_params_x_.correction_noise_single_value_);
  param_loader.loadParam("correction_noise/x/sigmoid/min_correction_value", correction_noise_params_x_.sigmoid_params.min_saturate_value_);
  param_loader.loadParam("correction_noise/x/sigmoid/max_correction_value", correction_noise_params_x_.sigmoid_params.max_saturate_value_);
  param_loader.loadParam("correction_noise/x/sigmoid/correction_function_center", correction_noise_params_x_.sigmoid_params.saturate_function_center_);
  param_loader.loadParam("correction_noise/x/sigmoid/correction_function_steepness", correction_noise_params_x_.sigmoid_params.saturate_function_steepness_);
  param_loader.loadParam("correction_noise/x/fraction/correction_scaling", correction_noise_params_x_.frac_params.correction_scaling);
  param_loader.loadParam("correction_noise/x/fraction/correction_confidence", correction_noise_params_x_.frac_params.correction_confidence);
  
  param_loader.loadParam("correction_noise/x/confidence_x_passthrough_value", confidence_x_passthrough_value_);
  
  // correction noise parameter - y
  param_loader.loadParam("correction_noise/y/single_value_correction_noise", correction_noise_params_y_.correction_noise_single_value_);
  param_loader.loadParam("correction_noise/y/sigmoid/min_correction_value", correction_noise_params_y_.sigmoid_params.min_saturate_value_);
  param_loader.loadParam("correction_noise/y/sigmoid/max_correction_value", correction_noise_params_y_.sigmoid_params.max_saturate_value_);
  param_loader.loadParam("correction_noise/y/sigmoid/correction_function_center", correction_noise_params_y_.sigmoid_params.saturate_function_center_);
  param_loader.loadParam("correction_noise/y/sigmoid/correction_function_steepness", correction_noise_params_y_.sigmoid_params.saturate_function_steepness_);
  param_loader.loadParam("correction_noise/y/fraction/correction_scaling", correction_noise_params_y_.frac_params.correction_scaling);
  param_loader.loadParam("correction_noise/y/fraction/correction_confidence", correction_noise_params_y_.frac_params.correction_confidence);
  
  param_loader.loadParam("correction_noise/y/confidence_y_passthrough_value", confidence_y_passthrough_value_);
  
  // correction noise parameter - z
  param_loader.loadParam("correction_noise/z/single_value_correction_noise", correction_noise_params_z_.correction_noise_single_value_);
  param_loader.loadParam("correction_noise/z/sigmoid/min_correction_value", correction_noise_params_z_.sigmoid_params.min_saturate_value_);
  param_loader.loadParam("correction_noise/z/sigmoid/max_correction_value", correction_noise_params_z_.sigmoid_params.max_saturate_value_);
  param_loader.loadParam("correction_noise/z/sigmoid/correction_function_center", correction_noise_params_z_.sigmoid_params.saturate_function_center_);
  param_loader.loadParam("correction_noise/z/sigmoid/correction_function_steepness", correction_noise_params_z_.sigmoid_params.saturate_function_steepness_);
  param_loader.loadParam("correction_noise/z/fraction/correction_scaling", correction_noise_params_z_.frac_params.correction_scaling);
  param_loader.loadParam("correction_noise/z/fraction/correction_confidence", correction_noise_params_z_.frac_params.correction_confidence);
  
  // correction noise parameter - yaw
  param_loader.loadParam("correction_noise/yaw/single_value_correction_noise", correction_noise_params_yaw_.correction_noise_single_value_);
  param_loader.loadParam("correction_noise/yaw/sigmoid/min_correction_value", correction_noise_params_yaw_.sigmoid_params.min_saturate_value_);
  param_loader.loadParam("correction_noise/yaw/sigmoid/max_correction_value", correction_noise_params_yaw_.sigmoid_params.max_saturate_value_);
  param_loader.loadParam("correction_noise/yaw/sigmoid/correction_function_center", correction_noise_params_yaw_.sigmoid_params.saturate_function_center_);
  param_loader.loadParam("correction_noise/yaw/sigmoid/correction_function_steepness", correction_noise_params_yaw_.sigmoid_params.saturate_function_steepness_);
  param_loader.loadParam("correction_noise/yaw/fraction/correction_scaling", correction_noise_params_yaw_.frac_params.correction_scaling);
  param_loader.loadParam("correction_noise/yaw/fraction/correction_confidence", correction_noise_params_yaw_.frac_params.correction_confidence);

  // | ------------------ correction saturation ----------------- |
  param_loader.loadParam("correction_saturation/enabled", enable_correction_saturation_);
  std::string saturation_method_name;
  param_loader.loadParam("correction_saturation/method", saturation_method_name);
  saturation_method_ = mapSaturationMethodString(saturation_method_name);
  if(saturation_method_ < 0) {
    ROS_ERROR_STREAM("[VinsDriftEstimator]: invalid saturation method: " << saturation_method_name);
    ros::shutdown();
  }
  
  // saturation parameter - x
  param_loader.loadParam("correction_saturation/x/single_value_saturation", saturation_params_x_.saturate_single_value_);
  param_loader.loadParam("correction_saturation/x/sigmoid/min_saturate_value", saturation_params_x_.sigmoid_params.min_saturate_value_);
  param_loader.loadParam("correction_saturation/x/sigmoid/max_saturate_value", saturation_params_x_.sigmoid_params.max_saturate_value_);
  param_loader.loadParam("correction_saturation/x/sigmoid/saturate_function_center", saturation_params_x_.sigmoid_params.saturate_function_center_);
  param_loader.loadParam("correction_saturation/x/sigmoid/saturate_function_steepness", saturation_params_x_.sigmoid_params.saturate_function_steepness_);
  
  // saturation parameter - y
  param_loader.loadParam("correction_saturation/y/single_value_saturation", saturation_params_y_.saturate_single_value_);
  param_loader.loadParam("correction_saturation/y/sigmoid/min_saturate_value", saturation_params_y_.sigmoid_params.min_saturate_value_);
  param_loader.loadParam("correction_saturation/y/sigmoid/max_saturate_value", saturation_params_y_.sigmoid_params.max_saturate_value_);
  param_loader.loadParam("correction_saturation/y/sigmoid/saturate_function_center", saturation_params_y_.sigmoid_params.saturate_function_center_);
  param_loader.loadParam("correction_saturation/y/sigmoid/saturate_function_steepness", saturation_params_y_.sigmoid_params.saturate_function_steepness_);
   
  // saturation parameter - z
  param_loader.loadParam("correction_saturation/z/single_value_saturation", saturation_params_z_.saturate_single_value_);
  param_loader.loadParam("correction_saturation/z/sigmoid/min_saturate_value", saturation_params_z_.sigmoid_params.min_saturate_value_);
  param_loader.loadParam("correction_saturation/z/sigmoid/max_saturate_value", saturation_params_z_.sigmoid_params.max_saturate_value_);
  param_loader.loadParam("correction_saturation/z/sigmoid/saturate_function_center", saturation_params_z_.sigmoid_params.saturate_function_center_);
  param_loader.loadParam("correction_saturation/z/sigmoid/saturate_function_steepness", saturation_params_z_.sigmoid_params.saturate_function_steepness_);
  
  // saturation parameter - yaw
  param_loader.loadParam("correction_saturation/yaw/single_value_saturation", saturation_params_yaw_.saturate_single_value_);
  param_loader.loadParam("correction_saturation/yaw/sigmoid/min_saturate_value", saturation_params_yaw_.sigmoid_params.min_saturate_value_);
  param_loader.loadParam("correction_saturation/yaw/sigmoid/max_saturate_value", saturation_params_yaw_.sigmoid_params.max_saturate_value_);
  param_loader.loadParam("correction_saturation/yaw/sigmoid/saturate_function_center", saturation_params_yaw_.sigmoid_params.saturate_function_center_);
  param_loader.loadParam("correction_saturation/yaw/sigmoid/saturate_function_steepness", saturation_params_yaw_.sigmoid_params.saturate_function_steepness_);

  // | ------------------- correction skipping ------------------ |
  double tmp_max_corr_size_x, tmp_max_corr_size_y, tmp_max_corr_size_z, tmp_max_corr_size_yaw;
  param_loader.loadParam("correction_skipping/enabled", use_correction_skipping_);
  param_loader.loadParam("correction_skipping/max_correction_x", tmp_max_corr_size_x);
  param_loader.loadParam("correction_skipping/max_correction_y", tmp_max_corr_size_y);
  param_loader.loadParam("correction_skipping/max_correction_z", tmp_max_corr_size_z);
  param_loader.loadParam("correction_skipping/max_correction_yaw", tmp_max_corr_size_yaw);
  
  max_corr_size_sq_x_   = tmp_max_corr_size_x * tmp_max_corr_size_x;
  max_corr_size_sq_y_   = tmp_max_corr_size_y * tmp_max_corr_size_y;
  max_corr_size_sq_z_   = tmp_max_corr_size_z * tmp_max_corr_size_z;
  max_corr_size_sq_yaw_ = tmp_max_corr_size_yaw * tmp_max_corr_size_yaw;

  // | ------------------- cofidence parameter ------------------ |
  param_loader.loadParam("covariance_coefficient/confidence_threshold", confidence_threshold_);

  // | ----------- check whether all parameters loaded ---------- |
  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[VinsDriftEstimator]: Could not load all non-optional parameters. Shutting down.");
    ros::shutdown();
  }
  
  // | -------------------- init system model ------------------- |
  // clang-format off
  double dt = 1.0 / main_timer_rate;
  generateA(dt);
  /* generateB(); */
  lkf_x_.H_ << 1, 0;
  lkf_y_.H_ << 1, 0;
  lkf_z_.H_ << 1, 0;
  lkf_yaw_.H_ << 1, 0; 
  // clang-format on

  // | --------------- Kalman filter intialization -------------- |
  const x_t        x0 = x_t::Zero();
  const P_t        P0 = 1e3 * P_t::Identity();
  const statecov_t sc0({x0, P0});
  
  lkf_x_.sc_ = sc0;
  lkf_y_.sc_ = sc0;
  lkf_z_.sc_ = sc0;
  lkf_yaw_.sc_ = sc0;

  lkf_x_.lkf_ = std::make_unique<lkf_t>(lkf_x_.A_, lkf_x_.B_, lkf_x_.H_);
  lkf_y_.lkf_ = std::make_unique<lkf_t>(lkf_y_.A_, lkf_y_.B_, lkf_y_.H_);
  lkf_z_.lkf_ = std::make_unique<lkf_t>(lkf_z_.A_, lkf_z_.B_, lkf_z_.H_);
  lkf_yaw_.lkf_ = std::make_unique<lkf_t>(lkf_yaw_.A_, lkf_yaw_.B_, lkf_yaw_.H_);

  // | ----------------- initialize subscribers ----------------- |
  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "VinsDriftEstimator";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 5;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_drifting_odom_         = mrs_lib::SubscribeHandler<nav_msgs::Odometry>(shopts, "drifting_odom_in", &VinsDriftEstimator::callbackDriftingOdom, this);
  sh_drift_correction_pose_ = mrs_lib::SubscribeHandler<geometry_msgs::PoseWithCovarianceStamped>(shopts, "drift_correction_pose_in", &VinsDriftEstimator::callbackDriftCorrectionPose, this);
  sh_orientation_           = mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped>(shopts, "orientation_in", &VinsDriftEstimator::callbackOrientation, this);
  sh_angular_velocity_      = mrs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped>(shopts, "angular_velocity_in", &VinsDriftEstimator::callbackAngularVelocity, this);
  sh_control_manager_diagnostics_ = mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>(shopts, "control_manager_diagnostics_in", &VinsDriftEstimator::callbackControlManagerDiagnostics, this);

  // | ------------------ initialize publishers ----------------- |
  ph_drifting_odom_           = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, "drifting_odom_out", 10);
  ph_estimated_drift_odom_    = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, "estimated_drift_out", 10);
  ph_internal_estimated_drift_odom_ = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, "internal_estimated_drift_out", 10);
  ph_corrected_odom_          = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, "corrected_odom_out", 10);
  ph_corrected_internal_odom_ = mrs_lib::PublisherHandler<nav_msgs::Odometry>(nh_, "corrected_internal_odom_out", 10);
  ph_debug_msg_               = mrs_lib::PublisherHandler<vins_drift_estimator::debug_msg>(nh_, "debug_msg_out", 10);
  ph_mrs_uav_status_          = mrs_lib::PublisherHandler<std_msgs::String>(nh_, "mrs_uav_status", 10);

  // | -------------------- initialize timers ------------------- |
  mrs_uav_status_timer_ = nh_.createTimer(ros::Duration(mrs_uav_status_timer_duration), &VinsDriftEstimator::mrsUavStatusTimer, this);
  main_timer_ = nh_.createTimer(ros::Rate(main_timer_rate), &VinsDriftEstimator::mainTimer, this);

  // | ---------- initialize dynamic reconfigure server --------- |
  reconfigure_server_.reset(new ReconfigureServer(mutex_dynamic_reconfigure_, nh_));
  ReconfigureServer::CallbackType f = boost::bind(&VinsDriftEstimator::callbackDynamicReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);
 
  /* set the default values of dynamic reconfigure server //{*/
    // group: Process noise 
    last_drs_config_.process_noise_position_x = pos_noise_x;
    last_drs_config_.process_noise_position_y = pos_noise_y;
    last_drs_config_.process_noise_position_z = pos_noise_z;
    last_drs_config_.process_noise_yaw = yaw_noise;
    last_drs_config_.process_noise_velocity_x = vel_noise_x;
    last_drs_config_.process_noise_velocity_y = vel_noise_y;
    last_drs_config_.process_noise_velocity_z = vel_noise_z;
    last_drs_config_.process_noise_yaw_rate = yaw_rate_noise;
    
    // group: Check measurement validity 
    last_drs_config_.gate_size_threshold = gate_size_threshold_;
    
    // group: Corrections noise 
    last_drs_config_.correction_noise_method = correction_noise_method_;
    last_drs_config_.correction_noise_single_value_x = correction_noise_params_x_.correction_noise_single_value_;
    last_drs_config_.correction_noise_single_value_y = correction_noise_params_y_.correction_noise_single_value_;
    last_drs_config_.correction_noise_single_value_z = correction_noise_params_z_.correction_noise_single_value_;
    last_drs_config_.correction_noise_single_value_yaw = correction_noise_params_yaw_.correction_noise_single_value_;
    
    // group: Covariance coefficient 
    last_drs_config_.confidence_threshold = confidence_threshold_;
    
    // group: Correction saturation values
    last_drs_config_.enable_correction_saturation = enable_correction_saturation_;
    last_drs_config_.saturation_method = saturation_method_;
    last_drs_config_.single_value_saturation_x = saturation_params_x_.saturate_single_value_;
    last_drs_config_.single_value_saturation_y = saturation_params_y_.saturate_single_value_;
    last_drs_config_.single_value_saturation_z = saturation_params_z_.saturate_single_value_;
    last_drs_config_.single_value_saturation_yaw = saturation_params_yaw_.saturate_single_value_;

    // group: Correction skipping 
    last_drs_config_.use_correction_skipping = use_correction_skipping_;
    last_drs_config_.max_value_x = tmp_max_corr_size_x;
    last_drs_config_.max_value_y = tmp_max_corr_size_y;
    last_drs_config_.max_value_z = tmp_max_corr_size_z;
    last_drs_config_.max_value_yaw = tmp_max_corr_size_yaw;
    
    reconfigure_server_->updateConfig(last_drs_config_);
  //}

  // | ----------------------- transformer ---------------------- |
  tfr = mrs_lib::Transformer(nh_);
  output_frame_ = uav_name_ + "/" + output_frame;
  output_frame_for_drifting_odom_ = uav_name_ + "/" + output_frame_for_drifting_odom;
  ns_drift_corrected_frame_id_ = uav_name_ + "/" + drift_corrected_frame_id;

  // | ------------------------ git info ------------------------ |
  std::tie(commit_hash_, git_changes_) = git_info::getGitMetadata(__FILE__);
  ROS_INFO_STREAM("[VinsDriftEstimator]: commit hash: " << commit_hash_);
  ROS_INFO_STREAM("[VinsDriftEstimator]: git changes: " << git_changes_);

  // | ----------------- initializaion finished ----------------- |
  is_initialized_ = true;
  ROS_INFO("[VinsDriftEstimator]: initialized");
}
//}


// --------------------------------------------------------------
// |                          callbacks                         |
// --------------------------------------------------------------

/* mrsUavStatusTimer()  //{*/
void VinsDriftEstimator::mrsUavStatusTimer([[maybe_unused]] const ros::TimerEvent& event)
{
  // do not continue if the nodelet is not initialized 
  if (!is_initialized_) {
    return;
  }
   
  vins_drift_estimator::debug_msg last_debug_msg = mrs_lib::get_mutexed(mutex_debug_msg_, last_debug_msg_);

  std::ostringstream string_to_show;
  string_to_show << std::fixed << std::setprecision(2);

  if (last_debug_msg.correction_noise_x < 0) { 
    string_to_show  << "-r ";
  }
  else {
    if (last_debug_msg.confidence < 0.8) { 
      string_to_show  << "-y ";
    }
    else {
      string_to_show  << "-g ";
    }
  }
 
  string_to_show << "(" << last_debug_msg.innovation_x << ", ";
  string_to_show  << last_debug_msg.innovation_y << ", ";
  string_to_show  << last_debug_msg.innovation_z << ", ";
  string_to_show  << last_debug_msg.innovation_yaw << "): ";
  string_to_show  << last_debug_msg.confidence;

  // define message with created string
  std_msgs::String msg; 
  msg.data = string_to_show.str();
  
  // publish string message
  try {
    ph_mrs_uav_status_.publish(msg);
  }
  catch (...) {
    ROS_ERROR_THROTTLE(0.25, "[VinsDriftEstimator] Exception caught during publishing mrs uav status topic.");
  }
}
//}

/* mainTimer()  //{*/
void VinsDriftEstimator::mainTimer([[maybe_unused]] const ros::TimerEvent& event)
{
  // do not continue if the nodelet is not initialized 
  if (!is_initialized_) {
    return;
  }
  
  if (!sh_drifting_odom_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[VinsDriftEstimator]: No drifting_odom messages received");
    return;
  }
  
  if (first_correction_) {
    first_correction_  = false;
    t_last_prediction_ = ros::Time::now();
    ROS_INFO("[VinsDriftEstimator]: first main timer callback");
    return;
  }
  
  // obtain dt for state prediction
  const ros::Time t_now = ros::Time::now();
  const double    dt    = (t_now - t_last_prediction_).toSec();
  if (dt <= 0.0) {  // sometimes the timer ticks twice simultaneously in simulation - we ignore the second tick, in case of stopping and starting
                    // the timer, the last_real is 0
    ROS_INFO_THROTTLE(0.25, "[VinsDriftEstimator]: invalid dt %.2f", dt);
    return;
  }
  t_last_prediction_ = t_now;
  
  setDt(dt);
  
  // get current state, covariance, and process noise
  statecov_t sc_x = mrs_lib::get_mutexed(lkf_x_.mutex_sc_, lkf_x_.sc_);
  Q_t        Q_x  = mrs_lib::get_mutexed(lkf_x_.mutex_Q_, lkf_x_.Q_);
  statecov_t sc_y = mrs_lib::get_mutexed(lkf_y_.mutex_sc_, lkf_y_.sc_);
  Q_t        Q_y  = mrs_lib::get_mutexed(lkf_y_.mutex_Q_, lkf_y_.Q_);
  statecov_t sc_z = mrs_lib::get_mutexed(lkf_z_.mutex_sc_, lkf_z_.sc_);
  Q_t        Q_z  = mrs_lib::get_mutexed(lkf_z_.mutex_Q_, lkf_z_.Q_);
  statecov_t sc_yaw = mrs_lib::get_mutexed(lkf_yaw_.mutex_sc_, lkf_yaw_.sc_);
  Q_t        Q_yaw  = mrs_lib::get_mutexed(lkf_yaw_.mutex_Q_, lkf_yaw_.Q_);
  
  /* prediction step - x //{ */
  try {
    // Apply the prediction step
    std::scoped_lock lock(lkf_x_.mutex_lkf_);
    u_t              u;
    sc_x = lkf_x_.lkf_->predict(sc_x, u, Q_x, dt);
  }
  catch (const std::exception &e) {
    ROS_ERROR("[VinsDriftEstimator]: LKF for x prediction failed: %s", e.what());
  }
  //}
  
  /* prediction step - y //{ */
  try {
    // Apply the prediction step
    std::scoped_lock lock(lkf_y_.mutex_lkf_);
    u_t              u;
    sc_y = lkf_y_.lkf_->predict(sc_y, u, Q_y, dt);
  }
  catch (const std::exception &e) {
    ROS_ERROR("[VinsDriftEstimator]: LKF for y prediction failed: %s", e.what());
  }
  //}
  
  /* prediction step - z //{ */
  try {
    // Apply the prediction step
    std::scoped_lock lock(lkf_z_.mutex_lkf_);
    u_t              u;
    sc_z = lkf_z_.lkf_->predict(sc_z, u, Q_z, dt);
  }
  catch (const std::exception &e) {
    ROS_ERROR("[VinsDriftEstimator]: LKF for z prediction failed: %s", e.what());
  }
  //}
  
  /* prediction step - yaw //{ */
  try {
    // Apply the prediction step
    std::scoped_lock lock(lkf_yaw_.mutex_lkf_);
    u_t              u;
    sc_yaw = lkf_yaw_.lkf_->predict(sc_yaw, u, Q_yaw, dt);
  }
  catch (const std::exception &e) {
    ROS_ERROR("[VinsDriftEstimator]: LKF for yaw prediction failed: %s", e.what());
  }
  //}
  
  mrs_lib::set_mutexed(lkf_x_.mutex_sc_, sc_x, lkf_x_.sc_);
  mrs_lib::set_mutexed(lkf_y_.mutex_sc_, sc_y, lkf_y_.sc_);
  mrs_lib::set_mutexed(lkf_z_.mutex_sc_, sc_z, lkf_z_.sc_);
  mrs_lib::set_mutexed(lkf_yaw_.mutex_sc_, sc_yaw, lkf_yaw_.sc_);
 
  if (!sh_drifting_odom_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[VinsDriftEstimator]: Main timer: No drifting_odom messages received");
    return;
  }
  nav_msgs::Odometry::ConstPtr drifting_odom = sh_drifting_odom_.getMsg(); 
 
  // publish 
  publishCorrectedOdom(drifting_odom);
  publishEstimatedDrift();
  publishDriftingOdom(drifting_odom);
}
//}

/* callbackDriftingOdom() //{*/
void VinsDriftEstimator::callbackDriftingOdom(const nav_msgs::Odometry::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }
  
  ROS_INFO_ONCE("[VinsDriftEstimator]: received first drifting odom message");

  // save the drifting odom
  { 
    std::scoped_lock lock(mutex_drifting_odom_buffer_);
    drifting_odom_buffer_.push_back(*msg);
    if (int(drifting_odom_buffer_.size()) > drifting_odom_buffer_size_) {
      drifting_odom_buffer_.pop_front();
    }
  }

}
//}

/* callbackDriftCorrectionPose() //{*/
void VinsDriftEstimator::callbackDriftCorrectionPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  if (!sh_drifting_odom_.hasMsg()) {
    ROS_WARN_THROTTLE(1.0, "[VinsDriftEstimator]: No drifting_odom messages received");
    return;
  }

  ROS_INFO_THROTTLE(1.0, "[VinsDriftEstimator]: Receiving correction");

  // // obtain dt for state prediction
  const ros::Time t_now = ros::Time::now();
 
  // get drifting odom
  std::optional<nav_msgs::Odometry> drifting_odom_opt = findClosestDriftingOdomMessage(msg->header.stamp); 
  nav_msgs::Odometry drifting_odom;
  if (drifting_odom_opt.has_value()) {
    drifting_odom = drifting_odom_opt.value();
  } else {
    ROS_ERROR_THROTTLE(0.25, "[VinsDriftEstimator]: Cannot find closest drifting odom message, probably the buffer is empty");
    return;
  }
 
  // transform msg to internal frame
  std::string output_frame = drifting_odom.header.frame_id;
  geometry_msgs::PoseWithCovarianceStamped correction_msg_transformed;
  geometry_msgs::PoseWithCovarianceStamped correction_msg;
  correction_msg.header = msg->header;
  correction_msg.pose.pose = msg->pose.pose;
  correction_msg.pose.covariance = msg->pose.covariance; 
  const auto tf_opt = tfr.getTransform(correction_msg.header.frame_id, output_frame);
  if (tf_opt.has_value()) {
    const auto tf = tf_opt.value();
    const auto tfd_opt = tfr.transform(correction_msg, tf);
    if (tfd_opt.has_value()) {
      correction_msg_transformed.pose.pose = tfd_opt.value().pose.pose;
      correction_msg_transformed.header.frame_id = output_frame;
    } else {
      ROS_ERROR_THROTTLE(0.5, "[VinsDriftEstimator]: Cannot transform correction pose from %s to %s", correction_msg.header.frame_id.c_str(), output_frame.c_str());
      return; 
    }
  } else { 
    ROS_ERROR_THROTTLE(0.5, "[VinsDriftEstimator]: Cannot get transformation from %s to %s for correction pose", correction_msg.header.frame_id.c_str(), output_frame.c_str());
    return; 
  }

  // get current state, covariance, and process noise
  statecov_t sc_x = mrs_lib::get_mutexed(lkf_x_.mutex_sc_, lkf_x_.sc_);
  statecov_t sc_y = mrs_lib::get_mutexed(lkf_y_.mutex_sc_, lkf_y_.sc_);
  statecov_t sc_z = mrs_lib::get_mutexed(lkf_z_.mutex_sc_, lkf_z_.sc_);
  statecov_t sc_yaw = mrs_lib::get_mutexed(lkf_yaw_.mutex_sc_, lkf_yaw_.sc_);
 
  // confidence [0-1]
  const double confidence = msg->pose.covariance[0];

  // mean of projection errors
  const double mean_projection_error = msg->pose.covariance[1];

  // number of points that are matched 
  // const double number_of_matched_points = msg->pose.covariance[2];
  
  // number of gates 
  const double number_of_gates = msg->pose.covariance[3];

  // length of the largest gate
  const double largest_gate_size = msg->pose.covariance[4];
  
  // comp time 
  // const double comp_time = msg->pose.covariance[5];
  
  // are gates coplanar?
  const double are_gates_coplanar = msg->pose.covariance[6];
  
  double confidence_threshold = mrs_lib::get_mutexed(mutex_confidence_threshold_, confidence_threshold_); 
  if (confidence <= confidence_threshold) {
    ROS_WARN_THROTTLE(0.25, "[VinsDriftEstimator]: discarding correction: confidence: %.4f, confidence threshold: %.4f", confidence, confidence_threshold);
    publishDebugMsg(
        msg->header.stamp, t_now, ros::Time::now(),
        -1, -1, -1, -1, 
        -1, -1, -1, -1, 
        0, 0, 0, 0, 
        0, 0, 0, 0, 
        true, false, confidence, confidence_threshold,
        mean_projection_error, number_of_gates, largest_gate_size,
        false, false, false, false 
    );
    return;
  }
  
  if (!checkIfMeasurementIsValid(confidence, mean_projection_error, number_of_gates, largest_gate_size)) {
    ROS_WARN_THROTTLE(0.25, "[VinsDriftEstimator]: discarding correction: measurement is not valid! confidence: %.4f, mean projection error: %.4f, number of gates: %.4f, largest gate size: %.4f", confidence, mean_projection_error, number_of_gates, largest_gate_size);
    publishDebugMsg(
        msg->header.stamp, t_now, ros::Time::now(),
        -1, -1, -1, -1, 
        -1, -1, -1, -1, 
        0, 0, 0, 0, 
        0, 0, 0, 0, 
        false, true, confidence, confidence_threshold,
        mean_projection_error, number_of_gates, largest_gate_size,
        false, false, false, false 
    );
    return;
  }

  // bool flags for correction skipping and correction saturation
  bool use_correction_skipping = mrs_lib::get_mutexed(mutex_use_correction_skipping_, use_correction_skipping_);
  bool enable_correction_saturation = mrs_lib::get_mutexed(mutex_enable_correction_saturation_, enable_correction_saturation_);

  /* correction step - x //{ */
  double noise_x = scaleCorrectionNoise(confidence, mrs_lib::get_mutexed(mutex_correction_noise_params_x_, correction_noise_params_x_));
  bool   skipping_correction_x = false;
  double saturate_val_x = -1.0;

  // fix correction noise for x if only one gate and this gate is large enough
  if(number_of_gates < 2 && largest_gate_size > mrs_lib::get_mutexed(mutex_gate_size_threshold_xy_correction_noise_, gate_size_threshold_xy_correction_noise_)) {
    if (confidence < confidence_x_passthrough_value_) {
      noise_x = scaleCorrectionNoise(confidence_x_passthrough_value_, mrs_lib::get_mutexed(mutex_correction_noise_params_x_, correction_noise_params_x_));
    } 
  }

  z_t z_x;
  z_x(0) = drifting_odom.pose.pose.position.x - correction_msg_transformed.pose.pose.position.x;
  double innovation_x = 0;

  // lkf per x axis
  if (use_correction_skipping && isCorrectionTooLarge(z_x, sc_x.x, mrs_lib::get_mutexed(mutex_max_corr_size_sq_x_, max_corr_size_sq_x_))) {
    skipping_correction_x = true;
    ROS_WARN_THROTTLE(0.25, "[VinsDriftEstimator]: skipping too large correction for x: %.2f", z_x(0));
  }
  else{
    if (enable_correction_saturation)
    {
      std::tie(z_x, saturate_val_x) = saturate(z_x, sc_x.x, confidence, mrs_lib::get_mutexed(mutex_saturation_params_x_, saturation_params_x_));
    }

    innovation_x = z_x(0)-sc_x.x(0);
    try {
      // Apply the correction step
      std::scoped_lock lock(lkf_x_.mutex_lkf_);
      sc_x = lkf_x_.lkf_->correct(sc_x, z_x, R_t::Ones() * noise_x);
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("[VinsDriftEstimator]: LKF correction for x failed: %s", e.what());
    }
  }
  //}
  
  /* correction step - y //{ */
  double noise_y = scaleCorrectionNoise(confidence, mrs_lib::get_mutexed(mutex_correction_noise_params_y_, correction_noise_params_y_));
  bool   skipping_correction_y = false;
  double saturate_val_y = -1.0;
  
  // fix correction noise for y if only one gate and this gate is large enough
  if(number_of_gates < 2 && largest_gate_size > mrs_lib::get_mutexed(mutex_gate_size_threshold_xy_correction_noise_, gate_size_threshold_xy_correction_noise_)) {
    if (confidence < confidence_y_passthrough_value_) {
      noise_y = scaleCorrectionNoise(confidence_y_passthrough_value_, mrs_lib::get_mutexed(mutex_correction_noise_params_y_, correction_noise_params_y_));
    } 
  }
  
  z_t z_y;
  z_y(0) = drifting_odom.pose.pose.position.y - correction_msg_transformed.pose.pose.position.y;
  double innovation_y = 0;

  // lkf per axis
  if (use_correction_skipping && isCorrectionTooLarge(z_y, sc_y.x, mrs_lib::get_mutexed(mutex_max_corr_size_sq_y_, max_corr_size_sq_y_))) {
    skipping_correction_y = true; 
    ROS_WARN_THROTTLE(0.25, "[VinsDriftEstimator]: skipping too large correction for y: %.2f", z_y(0));
  }
  else{
    if (enable_correction_saturation)
    {
      std::tie(z_y, saturate_val_y) = saturate(z_y, sc_y.x, confidence, mrs_lib::get_mutexed(mutex_saturation_params_y_, saturation_params_y_));
    }

    innovation_y = z_y(0)-sc_y.x(0);
    try {
      // Apply the correction step
      std::scoped_lock lock(lkf_y_.mutex_lkf_);
      sc_y = lkf_y_.lkf_->correct(sc_y, z_y, R_t::Ones() * noise_y);
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("[VinsDriftEstimator]: LKF correction for y failed: %s", e.what());
    }
  }
  //}
  
  /* correction step - z //{ */
  const double noise_z = scaleCorrectionNoise(confidence, mrs_lib::get_mutexed(mutex_correction_noise_params_z_, correction_noise_params_z_));
  bool   skipping_correction_z = false;
  double saturate_val_z = -1.0;

  z_t z_z;
  z_z(0) = drifting_odom.pose.pose.position.z - correction_msg_transformed.pose.pose.position.z;
  double innovation_z = 0;

  if (use_correction_skipping && isCorrectionTooLarge(z_z, sc_z.x, mrs_lib::get_mutexed(mutex_max_corr_size_sq_z_, max_corr_size_sq_z_))) {
    skipping_correction_z = true;
    ROS_WARN_THROTTLE(0.25, "[VinsDriftEstimator]: skipping too large correction for z: %.2f", z_z(0));
  }
  else if (correction_msg_transformed.pose.pose.position.z < minimal_fusable_z_) {
    skipping_correction_z = true;
    ROS_WARN_THROTTLE(0.25, "[VinsDriftEstimator]: z is below minimal fusable z (%.2f): %.2f", minimal_fusable_z_, correction_msg_transformed.pose.pose.position.z);
  }
  else if (are_gates_coplanar > 0.0 && largest_gate_size < mrs_lib::get_mutexed(mutex_gate_size_threshold_, gate_size_threshold_)) {
    skipping_correction_z = true;
    ROS_WARN_THROTTLE(0.25, "[VinsDriftEstimator]: gates are coplanar and largest gate size smaller than threshold, stop fusing z");
  }
  else{
    if (enable_correction_saturation)
    {
      std::tie(z_z, saturate_val_z) = saturate(z_z, sc_z.x, confidence, mrs_lib::get_mutexed(mutex_saturation_params_z_, saturation_params_z_));
    }
    
    innovation_z = z_z(0)-sc_z.x(0);
    try {
      // Apply the correction step
      std::scoped_lock lock(lkf_z_.mutex_lkf_);
      sc_z = lkf_z_.lkf_->correct(sc_z, z_z, R_t::Ones() * noise_z);
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("[VinsDriftEstimator]: LKF correction for z failed: %s", e.what());
    }
  }
  //}
  
  /* correction step - yaw //{ */
  const double noise_yaw = scaleCorrectionNoise(confidence, mrs_lib::get_mutexed(mutex_correction_noise_params_yaw_, correction_noise_params_yaw_));
  bool   skipping_correction_yaw = false;
  double saturate_val_yaw = -1.0;
  
  geometry_msgs::QuaternionStamped internal_orientation;
  if (sh_orientation_.hasMsg() && sh_angular_velocity_.hasMsg()) {
    internal_orientation.quaternion = sh_orientation_.getMsg()->quaternion;
  } else {
    internal_orientation.quaternion = drifting_odom.pose.pose.orientation;
    ROS_ERROR_THROTTLE(0.1, "[VinsDriftEstimator]: orientation or angular velocity not received, using orientation and angular velocity from drifting odom");
  }
  double drifting_odom_yaw = mrs_lib::AttitudeConverter(drifting_odom.pose.pose.orientation).getYaw(); 

  double msg_yaw = mrs_lib::AttitudeConverter(correction_msg_transformed.pose.pose.orientation).getYaw(); 
  double yaw_diff = mrs_lib::geometry::radians::unwrap(drifting_odom_yaw, msg_yaw) - msg_yaw;
   
  z_t z_yaw;
  z_yaw(0) = mrs_lib::geometry::sradians::unwrap(yaw_diff, sc_yaw.x(0));
  double innovation_yaw = 0;

  if (use_correction_skipping && isCorrectionTooLarge(z_yaw, sc_yaw.x, mrs_lib::get_mutexed(mutex_max_corr_size_sq_yaw_, max_corr_size_sq_yaw_))) {
    skipping_correction_yaw = true;
    ROS_WARN_THROTTLE(0.25, "[VinsDriftEstimator]: skipping too large correction for yaw: %.2f", z_yaw(0));
  }
  else if (!isYawFusable(are_gates_coplanar)) {
    skipping_correction_yaw = true;
    ROS_WARN_THROTTLE(0.25, "[VinsDriftEstimator]: Yaw measurement is not fusable! Skipping yaw correction!"); 
  }
  else{
    if (enable_correction_saturation)
    {
      std::tie(z_yaw, saturate_val_yaw) = saturate(z_yaw, sc_yaw.x, confidence, mrs_lib::get_mutexed(mutex_saturation_params_yaw_, saturation_params_yaw_));
    }

    innovation_yaw = z_yaw(0)-sc_yaw.x(0);
    try {
      // Apply the correction step
      std::scoped_lock lock(lkf_yaw_.mutex_lkf_);
      sc_yaw = lkf_yaw_.lkf_->correct(sc_yaw, z_yaw, R_t::Ones() * noise_yaw);
      sc_yaw.x(0) = mrs_lib::geometry::sradians::wrap(sc_yaw.x(0));
    }
    catch (const std::exception &e) {
      // In case of error, alert the user
      ROS_ERROR("[VinsDriftEstimator]: LKF correction for yaw failed: %s", e.what());
    }
  }
  //}
  
  mrs_lib::set_mutexed(lkf_x_.mutex_sc_, sc_x, lkf_x_.sc_);
  mrs_lib::set_mutexed(lkf_y_.mutex_sc_, sc_y, lkf_y_.sc_);
  mrs_lib::set_mutexed(lkf_z_.mutex_sc_, sc_z, lkf_z_.sc_);
  mrs_lib::set_mutexed(lkf_yaw_.mutex_sc_, sc_yaw, lkf_yaw_.sc_);

  publishDebugMsg(
      msg->header.stamp, t_now, ros::Time::now(),
      noise_x, noise_y, noise_z, noise_yaw, 
      saturate_val_x, saturate_val_y, saturate_val_z, saturate_val_yaw, 
      z_x(0), z_y(0), z_z(0), z_yaw(0),
      innovation_x, innovation_y, innovation_z, innovation_yaw,
      false, false, confidence, confidence_threshold,
      mean_projection_error, number_of_gates, largest_gate_size,
      skipping_correction_x, skipping_correction_y, skipping_correction_z, skipping_correction_yaw
  );
}
//}

/* callbackDynamicReconfigure() //{*/
void VinsDriftEstimator::callbackDynamicReconfigure([[maybe_unused]] Config& config, [[maybe_unused]] uint32_t level) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO("[VinsDriftEstimator]: Reconfigure Request");
    
  mrs_lib::set_mutexed(mutex_dynparam_, config, last_drs_config_);

  // set group: Process noise
  setQ(config.process_noise_position_x, config.process_noise_position_y, config.process_noise_position_z, config.process_noise_velocity_x, config.process_noise_velocity_y, config.process_noise_velocity_z, config.process_noise_yaw, config.process_noise_yaw_rate); 

  // set group: Check valid measurement 
  mrs_lib::set_mutexed(mutex_gate_size_threshold_, config.gate_size_threshold, gate_size_threshold_);

  // set group: Correction noise
  mrs_lib::set_mutexed(mutex_correction_noise_method_, config.correction_noise_method, correction_noise_method_);
  mrs_lib::set_mutexed(mutex_correction_noise_params_x_, config.correction_noise_single_value_x, correction_noise_params_x_.correction_noise_single_value_);
  mrs_lib::set_mutexed(mutex_correction_noise_params_y_, config.correction_noise_single_value_y, correction_noise_params_y_.correction_noise_single_value_);
  mrs_lib::set_mutexed(mutex_correction_noise_params_z_, config.correction_noise_single_value_z, correction_noise_params_z_.correction_noise_single_value_);
  mrs_lib::set_mutexed(mutex_correction_noise_params_yaw_, config.correction_noise_single_value_yaw, correction_noise_params_yaw_.correction_noise_single_value_);
  
  // set group: Covariance coefficient
  mrs_lib::set_mutexed(mutex_confidence_threshold_, config.confidence_threshold, confidence_threshold_); 
  
  // set group: Corrections saturation values
  mrs_lib::set_mutexed(mutex_enable_correction_saturation_, config.enable_correction_saturation, enable_correction_saturation_);
  mrs_lib::set_mutexed(mutex_saturation_method_, config.saturation_method, saturation_method_);
  mrs_lib::set_mutexed(mutex_saturation_params_x_, config.single_value_saturation_x, saturation_params_x_.saturate_single_value_);
  mrs_lib::set_mutexed(mutex_saturation_params_y_, config.single_value_saturation_y, saturation_params_y_.saturate_single_value_);
  mrs_lib::set_mutexed(mutex_saturation_params_z_, config.single_value_saturation_z, saturation_params_z_.saturate_single_value_);
  mrs_lib::set_mutexed(mutex_saturation_params_yaw_, config.single_value_saturation_yaw, saturation_params_yaw_.saturate_single_value_);
 
  // Correction skipping
  mrs_lib::set_mutexed(mutex_use_correction_skipping_, config.use_correction_skipping, use_correction_skipping_);
  mrs_lib::set_mutexed(mutex_max_corr_size_sq_x_, config.max_value_x*config.max_value_x, max_corr_size_sq_x_);
  mrs_lib::set_mutexed(mutex_max_corr_size_sq_y_, config.max_value_y*config.max_value_y, max_corr_size_sq_y_);
  mrs_lib::set_mutexed(mutex_max_corr_size_sq_z_, config.max_value_z*config.max_value_z, max_corr_size_sq_z_);
  mrs_lib::set_mutexed(mutex_max_corr_size_sq_yaw_, config.max_value_yaw*config.max_value_yaw, max_corr_size_sq_yaw_);
  
}
//}

/* callbackOrientation() //{*/
void VinsDriftEstimator::callbackOrientation([[maybe_unused]] const geometry_msgs::QuaternionStamped::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[VinsDriftEstimator]: received first orientation message");
}
//}

/* callbackAngularVelocity() //{*/
void VinsDriftEstimator::callbackAngularVelocity([[maybe_unused]] const geometry_msgs::Vector3Stamped::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[VinsDriftEstimator]: received first angular velocity message");
}
//}

/* callbackControlManagerDiagnostics() //{*/
void VinsDriftEstimator::callbackControlManagerDiagnostics(mrs_msgs::ControlManagerDiagnostics::ConstPtr msg) {

  if (!is_initialized_) {
    return;
  }
  
  ROS_INFO_ONCE("[VinsDriftEstimator]: received first control manager diagnostics");

  if (mrs_lib::get_mutexed(mutex_use_velocity_in_drift_model_, use_velocity_in_drift_model_)) {
  
    if(mrs_lib::get_mutexed(mutex_disable_velocity_before_tracker_activation_, disable_velocity_before_tracker_activation_)) {
      ROS_INFO_ONCE("[VinsDriftEstimator]: velocity in drift model will be enabled upon tracker activation");
    } 
    else {
      ROS_INFO_ONCE("[VinsDriftEstimator]: velocity in drift model is enabled");
      return; 
    }
  }

  if (msg->active_tracker == "CsvTracker") {
    ROS_INFO_ONCE("[VinsDriftEstimator]: CsvTracker is active");
    /* mrs_lib::set_mutexed(mutex_use_velocity_in_drift_model_, true, use_velocity_in_drift_model_); */
    if(mrs_lib::get_mutexed(mutex_disable_velocity_before_tracker_activation_, disable_velocity_before_tracker_activation_)) {
      mrs_lib::set_mutexed(mutex_disable_velocity_before_tracker_activation_, false, disable_velocity_before_tracker_activation_);
      ROS_INFO_ONCE("[VinsDriftEstimator]: velocity in drift model is enabled (Tracker has been activated!)");
      return;
    } 
  }
  
  ROS_INFO_THROTTLE(1.0, "[VinsDriftEstimator]: velocities in drift estimator are not used");

}
//}


// --------------------------------------------------------------
// |                        core methods                        |
// --------------------------------------------------------------

/* setDt() //{*/
void VinsDriftEstimator::setDt(const double dt) {
  generateA(dt);
  /* generateB(); */
 
  {
    std::scoped_lock lock(lkf_x_.mutex_lkf_);
    lkf_x_.lkf_->A = lkf_x_.A_;
    lkf_x_.lkf_->B = lkf_x_.B_;
  }
  
  {   
    std::scoped_lock lock(lkf_y_.mutex_lkf_);
    lkf_y_.lkf_->A = lkf_y_.A_;
    lkf_y_.lkf_->B = lkf_y_.B_;
  } 

  {   
    std::scoped_lock lock(lkf_z_.mutex_lkf_);
    lkf_z_.lkf_->A = lkf_z_.A_;
    lkf_z_.lkf_->B = lkf_z_.B_;
  } 
  
  {   
    std::scoped_lock lock(lkf_yaw_.mutex_lkf_);
    lkf_yaw_.lkf_->A = lkf_yaw_.A_;
    lkf_yaw_.lkf_->B = lkf_yaw_.B_;
  } 
}
//}

/* generateA() //{*/
void VinsDriftEstimator::generateA(double dt) {

  if(!mrs_lib::get_mutexed(mutex_use_velocity_in_drift_model_, use_velocity_in_drift_model_)) {
    dt = 0.0;
  }
  if(mrs_lib::get_mutexed(mutex_disable_velocity_before_tracker_activation_, disable_velocity_before_tracker_activation_)) {
    dt = 0.0;
  }
  // clang-format off
    lkf_x_.A_ <<
      1, dt,
      0, 1-x_friction_;
    lkf_y_.A_ <<
      1, dt,
      0, 1-y_friction_;
    lkf_z_.A_ <<
      1, dt,
      0, 1-z_friction_;
    lkf_yaw_.A_ <<
      1, dt,
      0, 1-yaw_friction_;
  // clang-format on
}
//}

/* generateB() //{*/
void VinsDriftEstimator::generateB() {

  // clang-format off
    lkf_x_.B_ <<
      0,
      0;
    lkf_y_.B_ <<
      0,
      0;
    lkf_z_.B_ <<
      0,
      0;
    lkf_yaw_.B_ <<
      0,
      0;
  // clang-format on
}
//}

/* setQ() //{*/
void VinsDriftEstimator::setQ(const double pos_noise_x, const double pos_noise_y, const double pos_noise_z, const double vel_noise_x, const double vel_noise_y, const double vel_noise_z, const double yaw_noise, const double yaw_rate_noise) {
  { 
    std::scoped_lock lock(lkf_x_.mutex_Q_);
    lkf_x_.Q_(0, 0) = pos_noise_x;
    lkf_x_.Q_(1, 1) = vel_noise_x;
  }
  
  { 
    std::scoped_lock lock(lkf_y_.mutex_Q_);
    lkf_y_.Q_(0, 0) = pos_noise_y;
    lkf_y_.Q_(1, 1) = vel_noise_y;
  }
  
  { 
    std::scoped_lock lock(lkf_z_.mutex_Q_);
    lkf_z_.Q_(0, 0) = pos_noise_z;
    lkf_z_.Q_(1, 1) = vel_noise_z;
  }
  
  { 
    std::scoped_lock lock(lkf_yaw_.mutex_Q_);
    lkf_yaw_.Q_(0, 0) = yaw_noise;
    lkf_yaw_.Q_(1, 1) = yaw_rate_noise;
  }
}
//}

/* publishDebugMsg() //{*/
void VinsDriftEstimator::publishDebugMsg(
    const ros::Time correction_pose_stamp, const ros::Time correction_start_stamp, const ros::Time correction_end_stamp,
    const double noise_x, const double noise_y, const double noise_z, const double noise_yaw,
    const double saturate_val_x, const double saturate_val_y, const double saturate_val_z, const double saturate_val_yaw,
    const double z_x, const double z_y, const double z_z, const double z_yaw,
    const double innovation_x, const double innovation_y, const double innovation_z, const double innovation_yaw, 
    const bool discarding_correction_due_confidence, const bool discarding_correction_due_invalid_measurement, const double confidence, const double confidence_threshold,
    const double mean_projection_error, const double number_of_gates, const double largest_gate_size, 
    const bool skipping_correction_x, const bool skipping_correction_y, const bool skipping_correction_z, const bool skipping_correction_yaw) {
  
  vins_drift_estimator::debug_msg msg;
  msg.header.frame_id = ns_drift_corrected_frame_id_;
  msg.header.stamp    = ros::Time::now();

  msg.commit_hash = commit_hash_;

  msg.gate_pose_stamp = correction_pose_stamp;
  msg.correction_duration = correction_end_stamp - correction_start_stamp;
  msg.from_image_to_correction_duration = correction_end_stamp - correction_pose_stamp;

  msg.correction_noise_method = mrs_lib::get_mutexed(mutex_correction_noise_method_, correction_noise_method_);
  msg.correction_noise_x = noise_x;
  msg.correction_noise_y = noise_y;
  msg.correction_noise_z = noise_z;
  msg.correction_noise_yaw = noise_yaw;

  msg.saturation_method = mrs_lib::get_mutexed(mutex_saturation_method_, saturation_method_);
  msg.saturate_value_x = saturate_val_x;
  msg.saturate_value_y = saturate_val_y;
  msg.saturate_value_z = saturate_val_z;
  msg.saturate_value_yaw = saturate_val_yaw;
  
  msg.z_x = z_x;
  msg.z_y = z_y;
  msg.z_z = z_z;
  msg.z_yaw = z_yaw;
  
  msg.innovation_x = innovation_x;
  msg.innovation_y = innovation_y;
  msg.innovation_z = innovation_z;
  msg.innovation_yaw = innovation_yaw;
  
  msg.discarding_correction_due_confidence = discarding_correction_due_confidence;
  msg.discarding_correction_due_invalid_measurement = discarding_correction_due_invalid_measurement;
  msg.confidence = confidence;
  msg.confidence_threshold = confidence_threshold;

  msg.skipping_correction_x = skipping_correction_x;
  msg.skipping_correction_y = skipping_correction_y;
  msg.skipping_correction_z = skipping_correction_z;
  msg.skipping_correction_yaw = skipping_correction_yaw;

  msg.mean_projection_error = mean_projection_error;
  msg.number_of_gates = number_of_gates;
  msg.largest_gate_size = largest_gate_size; 

  mrs_lib::set_mutexed(mutex_debug_msg_, msg, last_debug_msg_);

  ph_debug_msg_.publish(msg);
}
//}

/* publishEstimatedDrift() //{*/
void VinsDriftEstimator::publishEstimatedDrift() {
  nav_msgs::Odometry odom_out;
  odom_out.header.frame_id         = ns_drift_corrected_frame_id_;
  odom_out.header.stamp            = ros::Time::now();
  
  odom_out.pose.pose.position.x    = mrs_lib::get_mutexed(lkf_x_.mutex_sc_, lkf_x_.sc_).x(0);
  odom_out.pose.pose.position.y    = mrs_lib::get_mutexed(lkf_y_.mutex_sc_, lkf_y_.sc_).x(0);
  odom_out.pose.pose.position.z    = mrs_lib::get_mutexed(lkf_z_.mutex_sc_, lkf_z_.sc_).x(0);
  
  double yaw = mrs_lib::get_mutexed(lkf_yaw_.mutex_sc_, lkf_yaw_.sc_).x(0);
  mrs_lib::AttitudeConverter yaw_ac(0, 0, 0, mrs_lib::RPY_convention_t::RPY_EXTRINSIC);
  yaw_ac = yaw_ac.setYaw(yaw);
  odom_out.pose.pose.orientation = yaw_ac;
  
  odom_out.twist.twist.linear.x    = mrs_lib::get_mutexed(lkf_x_.mutex_sc_, lkf_x_.sc_).x(1);
  odom_out.twist.twist.linear.y    = mrs_lib::get_mutexed(lkf_y_.mutex_sc_, lkf_y_.sc_).x(1);
  odom_out.twist.twist.linear.z    = mrs_lib::get_mutexed(lkf_z_.mutex_sc_, lkf_z_.sc_).x(1);
  
  odom_out.twist.twist.angular.z    = mrs_lib::get_mutexed(lkf_yaw_.mutex_sc_, lkf_yaw_.sc_).x(1);
  
  Eigen::MatrixXd pose_covariance  = composePoseCovarianceMatrix();
  Eigen::MatrixXd twist_covariance = composeTwistCovarianceMatrix();
  
  odom_out.pose.covariance  = convertEigenToBoostArray(pose_covariance);
  odom_out.twist.covariance = convertEigenToBoostArray(twist_covariance);

  ph_internal_estimated_drift_odom_.publish(odom_out);

  ROS_INFO_THROTTLE(1.0, "[VinsDriftEstimator]: estimated drift x: %.2f y: %.2f z: %.2f yaw: %.3f", odom_out.pose.pose.position.x, odom_out.pose.pose.position.y,
                    odom_out.pose.pose.position.z, yaw);
  
  const auto tf_opt = tfr.getTransform(odom_out.header.frame_id, output_frame_);
  if (tf_opt.has_value()) {
    const auto tf = tf_opt.value();
    Eigen::Vector3d vec_odom_pose_out = {odom_out.pose.pose.position.x, odom_out.pose.pose.position.y, odom_out.pose.pose.position.z};
    Eigen::Vector3d vec_odom_vel_out = {odom_out.twist.twist.linear.x, odom_out.twist.twist.linear.y, odom_out.twist.twist.linear.z};
    const auto tfd_pose_opt = tfr.transformAsVector(vec_odom_pose_out, tf);
    const auto tfd_vel_opt = tfr.transformAsVector(vec_odom_vel_out, tf);
    if (tfd_pose_opt.has_value() && tfd_vel_opt.has_value()) {
      odom_out.header.frame_id = output_frame_;
      Eigen::Vector3d transformed_vec_odom_pose_out = tfd_pose_opt.value();
      odom_out.pose.pose.position.x = transformed_vec_odom_pose_out.x();
      odom_out.pose.pose.position.y = transformed_vec_odom_pose_out.y();
      odom_out.pose.pose.position.z = transformed_vec_odom_pose_out.z();
      Eigen::Vector3d transformed_vec_odom_vel_out = tfd_vel_opt.value();
      odom_out.twist.twist.linear.x = transformed_vec_odom_vel_out.x();
      odom_out.twist.twist.linear.y = transformed_vec_odom_vel_out.y();
      odom_out.twist.twist.linear.z = transformed_vec_odom_vel_out.z();
    } else {
      ROS_ERROR_THROTTLE(0.5, "[VinsDriftEstimator]: Cannot transform drift from %s to %s", odom_out.header.frame_id.c_str(), output_frame_.c_str());
      return; 
    }
  } else { 
    ROS_ERROR_THROTTLE(0.5, "[VinsDriftEstimator]: Cannot get transformation from %s to %s for corrected odom", odom_out.header.frame_id.c_str(), output_frame_.c_str());
    return;
  }
  ph_estimated_drift_odom_.publish(odom_out);
}
//}

/* publishCorrectedOdom() //{*/
void VinsDriftEstimator::publishCorrectedOdom(const nav_msgs::Odometry::ConstPtr &drifting_odom) {
  nav_msgs::Odometry odom_out;
  
  odom_out.header.frame_id       = drifting_odom->header.frame_id;
  // odom_out.header.stamp          = drifting_odom->header.stamp;
  odom_out.header.stamp          = ros::Time::now();
  odom_out.child_frame_id        = drifting_odom->child_frame_id;
  double dt = (odom_out.header.stamp - drifting_odom->header.stamp).toSec();
  
  // orientation and angular velocities
  geometry_msgs::QuaternionStamped internal_orientation;
  if (mrs_lib::get_mutexed(mutex_stop_using_hw_api_, stop_using_hw_api_)) {
    internal_orientation.quaternion = drifting_odom->pose.pose.orientation;
    odom_out.twist.twist.angular    = drifting_odom->twist.twist.angular;
    ROS_INFO_ONCE("[VinsDriftEstimator]: intentionally using orientation and angular velocity from drifting odom");
  } else if(sh_orientation_.hasMsg() && sh_angular_velocity_.hasMsg()) {
    internal_orientation.quaternion = sh_orientation_.getMsg()->quaternion;
    odom_out.twist.twist.angular    = sh_angular_velocity_.getMsg()->vector;  
  }
  else {
    internal_orientation.quaternion = drifting_odom->pose.pose.orientation;
    odom_out.twist.twist.angular    = drifting_odom->twist.twist.angular;
    ROS_ERROR_THROTTLE(0.1, "[VinsDriftEstimator]: orientation or angular velocity not received, using orientation and angular velocity from drifting odom");
  }
  mrs_lib::AttitudeConverter internal_odom_ac = mrs_lib::AttitudeConverter(internal_orientation.quaternion); 

  Eigen::Quaterniond q_drifting = mrs_lib::AttitudeConverter(drifting_odom->pose.pose.orientation);
  double yaw                    = mrs_lib::AttitudeConverter(q_drifting).getYaw(); 
  double yaw_drift              = mrs_lib::get_mutexed(lkf_yaw_.mutex_sc_, lkf_yaw_.sc_).x(0); 
  /* double final_yaw              = mrs_lib::geometry::radians::unwrap(yaw, yaw_drift) - yaw_drift + dt*odom_out.twist.twist.angular.z; */
  /* double delta_yaw              = mrs_lib::geometry::radians::unwrap(final_yaw, yaw) - yaw; */
  double delta_yaw              = yaw_drift + dt*mrs_lib::get_mutexed(lkf_yaw_.mutex_sc_, lkf_yaw_.sc_).x(1);

  Eigen::Quaterniond q_correction(Eigen::AngleAxisd(delta_yaw, Eigen::Vector3d::UnitZ()));
  Eigen::Quaterniond q_yaw_corrected = (q_correction.inverse() * q_drifting).normalized();

  double corrected_yaw = mrs_lib::AttitudeConverter(q_yaw_corrected).getYaw();
  double roll  = internal_odom_ac.getRoll(); 
  double pitch = internal_odom_ac.getPitch(); 
  mrs_lib::AttitudeConverter new_orientation_ac(roll, pitch, corrected_yaw);
  odom_out.pose.pose.orientation = new_orientation_ac;

  // position
  odom_out.pose.pose.position.x  = drifting_odom->pose.pose.position.x - mrs_lib::get_mutexed(lkf_x_.mutex_sc_, lkf_x_.sc_).x(0);
  odom_out.pose.pose.position.y  = drifting_odom->pose.pose.position.y - mrs_lib::get_mutexed(lkf_y_.mutex_sc_, lkf_y_.sc_).x(0);
  odom_out.pose.pose.position.z  = drifting_odom->pose.pose.position.z - mrs_lib::get_mutexed(lkf_z_.mutex_sc_, lkf_z_.sc_).x(0);
  
  // linear velocities
  odom_out.twist.twist.linear.x = drifting_odom->twist.twist.linear.x;
  odom_out.twist.twist.linear.y = drifting_odom->twist.twist.linear.y;
  odom_out.twist.twist.linear.z = drifting_odom->twist.twist.linear.z;

  // transform velocity drift from map frame to body frame
  tf2::Quaternion tf_quat;
  tf2::fromMsg(odom_out.pose.pose.orientation, tf_quat);

  geometry_msgs::Vector3 vec_in;
  vec_in.x = mrs_lib::get_mutexed(lkf_x_.mutex_sc_, lkf_x_.sc_).x(1);
  vec_in.y = mrs_lib::get_mutexed(lkf_y_.mutex_sc_, lkf_y_.sc_).x(1);
  vec_in.z = mrs_lib::get_mutexed(lkf_z_.mutex_sc_, lkf_z_.sc_).x(1);
  geometry_msgs::Vector3 vec_out = rotateVector(vec_in, mrs_lib::AttitudeConverter(tf_quat.inverse()));

  // correct velocity
  if(do_velocity_correction_x_) {
    odom_out.twist.twist.linear.x -= vec_out.x;
  }
  if(do_velocity_correction_y_) {
    odom_out.twist.twist.linear.y -= vec_out.y;
  }
  if(do_velocity_correction_z_) {
    odom_out.twist.twist.linear.z -= vec_out.z;
  }
  
  // compute position deltas
  geometry_msgs::Vector3 vel_body_frame;
  vel_body_frame.x = drifting_odom->twist.twist.linear.x;
  vel_body_frame.y = drifting_odom->twist.twist.linear.y;
  vel_body_frame.z = drifting_odom->twist.twist.linear.z;
  geometry_msgs::Vector3 vel_map_frame = rotateVector(vel_body_frame, mrs_lib::AttitudeConverter(tf_quat));

  double dx = 0;
  double dy = 0; 
  double dz = 0; 
  if(dt > 0) {
    dx = dt*vel_map_frame.x;
    dy = dt*vel_map_frame.y;
    dz = dt*vel_map_frame.z;
  }
  odom_out.pose.pose.position.x  = odom_out.pose.pose.position.x + dx;
  odom_out.pose.pose.position.y  = odom_out.pose.pose.position.y + dy;
  odom_out.pose.pose.position.z  = odom_out.pose.pose.position.z + dz;

  // covariances
  Eigen::MatrixXd pose_covariance  = composePoseCovarianceMatrix();
  Eigen::MatrixXd twist_covariance = composeTwistCovarianceMatrix(); 
  odom_out.pose.covariance  = convertEigenToBoostArray(pose_covariance);
  odom_out.twist.covariance = convertEigenToBoostArray(twist_covariance);
 
  // publish corrected internal odom
  ph_corrected_internal_odom_.publish(odom_out);

  bool publish_odom = true;
  // transform to desired frame
  if(transform_corrected_odom_) {
    geometry_msgs::PoseWithCovarianceStamped odom_out_pose;
    odom_out_pose.header = odom_out.header;
    odom_out_pose.pose.pose = odom_out.pose.pose;
    odom_out_pose.pose.covariance = odom_out.pose.covariance; 
    const auto tf_opt = tfr.getTransform(odom_out_pose.header.frame_id, output_frame_);
    if (tf_opt.has_value()) {
      const auto tf = tf_opt.value();
      const auto tfd_opt = tfr.transform(odom_out_pose, tf);
      if (tfd_opt.has_value()) {
        odom_out.pose = tfd_opt.value().pose;
        odom_out.header.frame_id = output_frame_;
      } else {
        publish_odom = false;
        ROS_ERROR_THROTTLE(0.5, "[VinsDriftEstimator]: Cannot transform corrected odom from %s to %s", odom_out_pose.header.frame_id.c_str(), output_frame_.c_str());
      }
    } else { 
      publish_odom = false;
      ROS_ERROR_THROTTLE(0.5, "[VinsDriftEstimator]: Cannot get transformation from %s to %s for corrected odom", odom_out_pose.header.frame_id.c_str(), output_frame_.c_str());
    }
  }

  // publish corrected odom in desired frame
  if(publish_odom) {
    ph_corrected_odom_.publish(odom_out);
    ROS_INFO_ONCE("[VinsDriftEstimator]: publishing corrected odom");
  }
  else {
    ROS_WARN_THROTTLE(1.0, "[VinsDriftEstimator]: Cannot publish corrected odom, probably cannot transform odom to desired frame!");
  }

}
//}

/* publishDriftingOdom() //{*/
void VinsDriftEstimator::publishDriftingOdom(const nav_msgs::Odometry::ConstPtr &drifting_odom) {
  nav_msgs::Odometry odom_out;
  
  odom_out.header.frame_id = drifting_odom->header.frame_id;
  odom_out.header.stamp    = drifting_odom->header.stamp;
  odom_out.child_frame_id  = drifting_odom->child_frame_id;

  // position
  odom_out.pose.pose.position.x  = drifting_odom->pose.pose.position.x;
  odom_out.pose.pose.position.y  = drifting_odom->pose.pose.position.y;
  odom_out.pose.pose.position.z  = drifting_odom->pose.pose.position.z;
 
  // orientation
  odom_out.pose.pose.orientation.x = drifting_odom->pose.pose.orientation.x;
  odom_out.pose.pose.orientation.y = drifting_odom->pose.pose.orientation.y;
  odom_out.pose.pose.orientation.z = drifting_odom->pose.pose.orientation.z;
  odom_out.pose.pose.orientation.w = drifting_odom->pose.pose.orientation.w;
  
  // linear velocity
  odom_out.twist.twist.linear.x  = drifting_odom->twist.twist.linear.x;
  odom_out.twist.twist.linear.y  = drifting_odom->twist.twist.linear.y;
  odom_out.twist.twist.linear.z  = drifting_odom->twist.twist.linear.z;
 
  // angular velocity
  odom_out.twist.twist.angular.x = drifting_odom->twist.twist.angular.x;
  odom_out.twist.twist.angular.y = drifting_odom->twist.twist.angular.y;
  odom_out.twist.twist.angular.z = drifting_odom->twist.twist.angular.z;
 
  // covariances
  odom_out.pose.covariance  = drifting_odom->pose.covariance;
  odom_out.twist.covariance = drifting_odom->twist.covariance;
   
  // transform to desired frame
  geometry_msgs::PoseWithCovarianceStamped odom_out_pose;
  odom_out_pose.header = odom_out.header;
  odom_out_pose.pose.pose = odom_out.pose.pose;
  odom_out_pose.pose.covariance = odom_out.pose.covariance; 
  const auto tf_opt = tfr.getTransform(odom_out_pose.header.frame_id, output_frame_for_drifting_odom_);
  if (tf_opt.has_value()) {
    const auto tf = tf_opt.value();
    const auto tfd_opt = tfr.transform(odom_out_pose, tf);
    if (tfd_opt.has_value()) {
      odom_out.pose = tfd_opt.value().pose;
      odom_out.header.frame_id = output_frame_for_drifting_odom_;
    } else {
      ROS_ERROR_THROTTLE(0.5, "[VinsDriftEstimator]: Cannot transform drifting odom from %s to %s", odom_out_pose.header.frame_id.c_str(), output_frame_for_drifting_odom_.c_str());
      return; 
    }
  } else { 
    ROS_ERROR_THROTTLE(0.5, "[VinsDriftEstimator]: Cannot get transformation from %s to %s for drifting odom", odom_out_pose.header.frame_id.c_str(), output_frame_for_drifting_odom_.c_str());
    return; 
  }
 
  // publish drifting odom
  ph_drifting_odom_.publish(odom_out);

  ROS_INFO_ONCE("[VinsDriftEstimator]: publishing drifitng odom in frame %s", output_frame_for_drifting_odom_.c_str());
}
//}

/* composePoseCovarianceMatrix() //{*/
Eigen::MatrixXd VinsDriftEstimator::composePoseCovarianceMatrix() {
  Eigen::MatrixXd lkf_P = Eigen::MatrixXd::Zero(6,6);
 
  Eigen::MatrixXd lkf_x_P = mrs_lib::get_mutexed(lkf_x_.mutex_sc_, lkf_x_.sc_.P);
  Eigen::MatrixXd lkf_y_P = mrs_lib::get_mutexed(lkf_y_.mutex_sc_, lkf_y_.sc_.P);
  Eigen::MatrixXd lkf_z_P = mrs_lib::get_mutexed(lkf_z_.mutex_sc_, lkf_z_.sc_.P);
  Eigen::MatrixXd lkf_yaw_P = mrs_lib::get_mutexed(lkf_yaw_.mutex_sc_, lkf_yaw_.sc_.P);

  lkf_P(0,0) = lkf_x_P(0);
  lkf_P(1,1) = lkf_y_P(0);
  lkf_P(2,2) = lkf_z_P(0);
  lkf_P(5,5) = lkf_yaw_P(0);
  
  return lkf_P;
}
//}

/* composeTwistCovarianceMatrix() //{*/
Eigen::MatrixXd VinsDriftEstimator::composeTwistCovarianceMatrix() {
  Eigen::MatrixXd lkf_P = Eigen::MatrixXd::Zero(6,6);
  
  Eigen::MatrixXd lkf_x_P = mrs_lib::get_mutexed(lkf_x_.mutex_sc_, lkf_x_.sc_.P);
  Eigen::MatrixXd lkf_y_P = mrs_lib::get_mutexed(lkf_y_.mutex_sc_, lkf_y_.sc_.P);
  Eigen::MatrixXd lkf_z_P = mrs_lib::get_mutexed(lkf_z_.mutex_sc_, lkf_z_.sc_.P);
  Eigen::MatrixXd lkf_yaw_P = mrs_lib::get_mutexed(lkf_yaw_.mutex_sc_, lkf_yaw_.sc_.P);

  lkf_P(0,0) = lkf_x_P(1);
  lkf_P(1,1) = lkf_y_P(1);
  lkf_P(2,2) = lkf_z_P(1);
  lkf_P(5,5) = lkf_yaw_P(1);
  
  return lkf_P;
}
//}

/* convertEigenToBoostArray() //{*/
boost::array<double, 3 * drift_estimator_model::n_states * 3 * drift_estimator_model::n_states> VinsDriftEstimator::convertEigenToBoostArray(
    const Eigen::MatrixXd &eigen_matrix) {

  int rows = eigen_matrix.rows();
  int cols = eigen_matrix.cols();

  // Create a boost::array to hold the 1D representation
  boost::array<double, 3 * drift_estimator_model::n_states * 3 * drift_estimator_model::n_states> boost_array;

  // Copy the values from the Eigen matrix into the Boost array
  int idx = 0;
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      boost_array[idx++] = eigen_matrix(i, j);
    }
  }

  return boost_array;
}
//}

/* isCorrectionTooLarge() //{*/
bool VinsDriftEstimator::isCorrectionTooLarge(const z_t &z, const x_t &x, const double max_corr_size_sq) {
  return std::pow(z(0) - x(0), 2) > max_corr_size_sq;
}
//}

/* saturate() //{*/
std::tuple<VinsDriftEstimator::z_t, double> VinsDriftEstimator::saturate(const z_t &z, const x_t &x, const double confidence, const saturation_params_t saturation_params) {
  int saturation_method = mrs_lib::get_mutexed(mutex_saturation_method_, saturation_method_); 
  
  if (saturation_method == saturation_method_ns::sigmoid) {
    double saturate_val = sigmoid(confidence, saturation_params.sigmoid_params.max_saturate_value_-saturation_params.sigmoid_params.min_saturate_value_, saturation_params.sigmoid_params.saturate_function_steepness_, saturation_params.sigmoid_params.saturate_function_center_, saturation_params.sigmoid_params.min_saturate_value_);
    return std::make_tuple(computeSaturation(z, x, saturate_val), saturate_val);
  } 
  else if (saturation_method == saturation_method_ns::single_value) {
    return std::make_tuple(computeSaturation(z, x, saturation_params.saturate_single_value_), saturation_params.saturate_single_value_);
  }
  else if (saturation_method == saturation_method_ns::no_saturation) {
    return std::make_tuple(z, -1.0); 
  }
  else {
    ROS_ERROR_THROTTLE(0.25, "[VinsDriftEstimator]: Unknown saturation method, using sigmoid.");
    double saturate_val = sigmoid(confidence, saturation_params.sigmoid_params.max_saturate_value_-saturation_params.sigmoid_params.min_saturate_value_, saturation_params.sigmoid_params.saturate_function_steepness_, saturation_params.sigmoid_params.saturate_function_center_, saturation_params.sigmoid_params.min_saturate_value_);
    return std::make_tuple(computeSaturation(z, x, saturate_val), saturate_val);
  }
 
  ROS_ERROR_THROTTLE(0.25, "[VinsDriftEstimator]: Unknown saturation method, saturation is not used.");
  return std::make_tuple(z, -1.0); 
}
//}

/* computeSaturation() //{*/
VinsDriftEstimator::z_t VinsDriftEstimator::computeSaturation(const z_t &z, const x_t &x, const double saturate_val) { 
  z_t z_sat;
  for (int i = 0; i < drift_estimator_model::n_states; i++) {
    if (z(i) - x(i) > saturate_val) {
      z_sat(i) = x(i) + saturate_val;
    } else if (z(i) - x(i) < -saturate_val) {
      z_sat(i) = x(i) - saturate_val;
    } else {
      z_sat(i) = z(i);
    }
  }
  return z_sat;
}
//}

/* scaleCorrectionNoise() //{*/
double VinsDriftEstimator::scaleCorrectionNoise(const double confidence, const correction_noise_params_t correction_noise_params) {
  int correction_noise_method = mrs_lib::get_mutexed(mutex_correction_noise_method_, correction_noise_method_); 
  
  if (correction_noise_method == correction_noise_method_ns::sigmoid) {
    return sigmoid(confidence, correction_noise_params.sigmoid_params.max_saturate_value_-correction_noise_params.sigmoid_params.min_saturate_value_, correction_noise_params.sigmoid_params.saturate_function_steepness_, correction_noise_params.sigmoid_params.saturate_function_center_, correction_noise_params.sigmoid_params.min_saturate_value_);
  } 
  else if (correction_noise_method == correction_noise_method_ns::single_value) {
    return correction_noise_params.correction_noise_single_value_;
  }
  else if (correction_noise_method == correction_noise_method_ns::fraction) {
    return fraction_function(confidence, correction_noise_params.frac_params.correction_scaling, correction_noise_params.frac_params.correction_confidence);
  }
  else {
    ROS_ERROR_THROTTLE(0.25, "[VinsDriftEstimator]: Unknown correction noise method, single value %.2f is used.", correction_noise_params.correction_noise_single_value_);
    return correction_noise_params.correction_noise_single_value_; 
  }
 
  ROS_ERROR_THROTTLE(0.25, "[VinsDriftEstimator]: Unknown correction noise method, single value %.2f is used.", correction_noise_params.correction_noise_single_value_);
  return correction_noise_params.correction_noise_single_value_; 
}
//}

/* findClosestDriftingOdomMessage() //{*/
std::optional<nav_msgs::Odometry> VinsDriftEstimator::findClosestDriftingOdomMessage(const ros::Time target_timestamp) {
    
    std::scoped_lock lock(mutex_drifting_odom_buffer_);
    
    if (drifting_odom_buffer_.empty()) {
        ROS_WARN("[VinsDriftEstimator] No data in drifting odom buffer to match with correction pose.");
        return std::nullopt;
    }  

    // Find the first element with timestamp >= target_timestamp
    auto after = std::lower_bound(drifting_odom_buffer_.begin(), drifting_odom_buffer_.end(), target_timestamp,
        [](const nav_msgs::Odometry& odom, const ros::Time& t) {
            return odom.header.stamp < t;
        });

    // Target is outside the buffer range, return boundary message
    if (after == drifting_odom_buffer_.end()) {
        ROS_WARN("[VinsDriftEstimator] Target timestamp is after all buffered odom messages, returning latest.");
        return drifting_odom_buffer_.back();
    }
    if (after == drifting_odom_buffer_.begin()) {
        ROS_WARN("[VinsDriftEstimator] Target timestamp is before all buffered odom messages, returning earliest.");
        return drifting_odom_buffer_.front();
    }

    auto before = std::prev(after);

    const double t0 = before->header.stamp.toSec();
    const double t1 = after->header.stamp.toSec();
    const double t  = target_timestamp.toSec();
    const double alpha = (t - t0) / (t1 - t0);

    nav_msgs::Odometry interpolated;
    interpolated.header.stamp    = target_timestamp;
    interpolated.header.frame_id = before->header.frame_id;
    interpolated.child_frame_id  = before->child_frame_id;

    // Linearly interpolate position
    const auto& p0 = before->pose.pose.position;
    const auto& p1 = after->pose.pose.position;
    interpolated.pose.pose.position.x = p0.x + alpha * (p1.x - p0.x);
    interpolated.pose.pose.position.y = p0.y + alpha * (p1.y - p0.y);
    interpolated.pose.pose.position.z = p0.z + alpha * (p1.z - p0.z);

    // Spherically interpolate (SLERP) orientation
    const auto& q0 = before->pose.pose.orientation;
    const auto& q1 = after->pose.pose.orientation;
    tf2::Quaternion tq0(q0.x, q0.y, q0.z, q0.w);
    tf2::Quaternion tq1(q1.x, q1.y, q1.z, q1.w);
    tf2::Quaternion tq_interp = tq0.slerp(tq1, alpha);
    interpolated.pose.pose.orientation.x = tq_interp.x();
    interpolated.pose.pose.orientation.y = tq_interp.y();
    interpolated.pose.pose.orientation.z = tq_interp.z();
    interpolated.pose.pose.orientation.w = tq_interp.w();

    // Linearly interpolate twist (velocities)
    const auto& lv0 = before->twist.twist.linear;
    const auto& lv1 = after->twist.twist.linear;
    interpolated.twist.twist.linear.x = lv0.x + alpha * (lv1.x - lv0.x);
    interpolated.twist.twist.linear.y = lv0.y + alpha * (lv1.y - lv0.y);
    interpolated.twist.twist.linear.z = lv0.z + alpha * (lv1.z - lv0.z);

    const auto& av0 = before->twist.twist.angular;
    const auto& av1 = after->twist.twist.angular;
    interpolated.twist.twist.angular.x = av0.x + alpha * (av1.x - av0.x);
    interpolated.twist.twist.angular.y = av0.y + alpha * (av1.y - av0.y);
    interpolated.twist.twist.angular.z = av0.z + alpha * (av1.z - av0.z);

    return interpolated;
}
//}

/* checkIfMeasurementIsValid() //{*/
bool VinsDriftEstimator::checkIfMeasurementIsValid(double confidence, double mean_projection_error, double number_of_gates, double largest_gate_size) { 
  if (number_of_gates < 2 && largest_gate_size < mrs_lib::get_mutexed(mutex_gate_size_threshold_, gate_size_threshold_)) {
    ROS_WARN_THROTTLE(0.5, "[VinsDriftEstimator]: Only one gate and its size is not large enough. Confidence: %.2f, mean projection error: %.2f, number of gates: %.2f, largest gate size: %.2f", confidence, mean_projection_error, number_of_gates, largest_gate_size); 
    return false;
  }
  return true;
}
//}

/* isYawFusable() //{*/
bool VinsDriftEstimator::isYawFusable(double are_gates_coplanar) { 
  if (are_gates_coplanar > 0.0) {
    ROS_WARN_THROTTLE(0.5, "[VinsDriftEstimator]: Gates are coplanar! Yaw drift is not corrected now."); 
    return false;
  }
  return true;
}
//}

// --------------------------------------------------------------
// |                       support methods                      |
// --------------------------------------------------------------

/* rotateVector() //{*/
geometry_msgs::Vector3 VinsDriftEstimator::rotateVector(const geometry_msgs::Vector3& vec_in, const geometry_msgs::Quaternion& q_in) {
  try {
    Eigen::Matrix3d        R = mrs_lib::AttitudeConverter(q_in);
    Eigen::Vector3d        vec_in_eigen(vec_in.x, vec_in.y, vec_in.z);
    Eigen::Vector3d        vec_eigen_rotated = R * vec_in_eigen;
    geometry_msgs::Vector3 vec_out;
    vec_out.x = vec_eigen_rotated[0];
    vec_out.y = vec_eigen_rotated[1];
    vec_out.z = vec_eigen_rotated[2];
    return vec_out;
  }
  catch (...) {
    ROS_ERROR_THROTTLE(1.0, "[VinsDriftEstimator]: RotateVector() failed");
    return vec_in;
  }
}
//}

/* normalizeQuaternion() //{*/
geometry_msgs::Quaternion VinsDriftEstimator::normalizeQuaternion(const geometry_msgs::Quaternion& q_msg) {
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    tf2::Quaternion q_tf2_normalized = q.normalized();
    geometry_msgs::Quaternion q_normalized;
    q_normalized.x = q_tf2_normalized.x();
    q_normalized.y = q_tf2_normalized.y();
    q_normalized.z = q_tf2_normalized.z();
    q_normalized.w = q_tf2_normalized.w();
    return q_normalized;
}
//}

/* sigmoid() //{*/
double VinsDriftEstimator::sigmoid(const double x, const double a, const double b, const double c, const double d) {
  return a * (1 / (1 + std::exp(-b * (x - c)))) + d;
}
//}

/* fraction_function() //{*/
double VinsDriftEstimator::fraction_function(const double x, const double a, const double b) {
  if (x < 0.0000001) {
    return 10000000;
  }
  return a * b / x;
}
//}

/* mapSaturationMethodString() //{*/
int VinsDriftEstimator::mapSaturationMethodString(const std::string method_name) {
  if (method_name == "sigmoid") {
    return saturation_method_ns::sigmoid;
  } else if (method_name == "single_value") {
    return saturation_method_ns::single_value;
  } else if (method_name == "no_saturation") {
    return saturation_method_ns::no_saturation;
  } else {
    ROS_ERROR("[VinsDriftEstimator]: Unknown saturation method.");
    return -1; 
  }
}
//}

/* mapCorrectionNoiseMethodName() //{*/
int VinsDriftEstimator::mapCorrectionNoiseMethodName(const std::string method_name) {
  if (method_name == "sigmoid") {
    return correction_noise_method_ns::sigmoid;
  } else if (method_name == "single_value") {
    return correction_noise_method_ns::single_value;
  } else if (method_name == "fraction") {
    return correction_noise_method_ns::fraction;
  } else {
    ROS_ERROR("[VinsDriftEstimator]: Unknown correction noise method.");
    return -1; 
  }
}
//}

}  // namespace vins_drift_estimator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(vins_drift_estimator::VinsDriftEstimator, nodelet::Nodelet)

