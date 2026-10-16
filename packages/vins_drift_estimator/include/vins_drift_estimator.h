#ifndef VINS_DRIFT_ESTIMATOR_H
#define VINS_DRIFT_ESTIMATOR_H

/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <boost/array.hpp>

#include <dynamic_reconfigure/server.h>

#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/ControlManagerDiagnostics.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/lkf.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/transformer.h>

#include <vins_drift_estimator/debug_msg.h>
#include <vins_drift_estimator/dynparamConfig.h>

#include <deque>

//}

namespace vins_drift_estimator
{

namespace drift_estimator_model
{
  const int n_axes         = 1;
  const int n_states       = 2;
  const int n_inputs       = 0;
  const int n_measurements = 1;
}

namespace saturation_method_ns
{
  const int sigmoid       = 0;
  const int single_value  = 1;
  const int no_saturation = 2;
}

namespace correction_noise_method_ns
{
  const int sigmoid       = 0;
  const int single_value  = 1;
  const int fraction      = 2;
}

class VinsDriftEstimator : public nodelet::Nodelet {

  using lkf_t      = mrs_lib::LKF<drift_estimator_model::n_states, drift_estimator_model::n_inputs, drift_estimator_model::n_measurements>;
  using A_t        = lkf_t::A_t;
  using B_t        = lkf_t::B_t;
  using H_t        = lkf_t::H_t;
  using Q_t        = lkf_t::Q_t;
  using x_t        = lkf_t::x_t;
  using P_t        = lkf_t::P_t;
  using u_t        = lkf_t::u_t;
  using z_t        = lkf_t::z_t;
  using R_t        = lkf_t::R_t;
  using statecov_t = lkf_t::statecov_t;

private:

  // | --------------------- typedef struct --------------------- |
  typedef struct
  {
      A_t                    A_;
      B_t                    B_;
      H_t                    H_;
      Q_t                    Q_;
      mutable std::mutex     mutex_Q_;
      std::unique_ptr<lkf_t> lkf_;
      mutable std::mutex     mutex_lkf_;
      statecov_t             sc_;
      mutable std::mutex     mutex_sc_;
  } lkf_state_t;
  
  typedef struct
  {
      double min_saturate_value_;
      double max_saturate_value_;
      double saturate_function_center_;
      double saturate_function_steepness_;
  } sigmoid_params_t;
  
  typedef struct
  {
      double correction_scaling;
      double correction_confidence;
  } frac_params_t;
  
  typedef struct
  {
      sigmoid_params_t sigmoid_params;
      double correction_noise_single_value_;
      frac_params_t frac_params;
  } correction_noise_params_t;
  
  typedef struct
  {
      sigmoid_params_t sigmoid_params;
      double saturate_single_value_;
  } saturation_params_t;

  // | ------------------- general parameters ------------------- |
  ros::NodeHandle nh_;
  
  mrs_lib::Transformer tfr;

  std::deque<nav_msgs::Odometry> drifting_odom_buffer_;
  int drifting_odom_buffer_size_ = 100;
  mutable std::mutex mutex_drifting_odom_buffer_;

  bool is_initialized_ = false;

  bool transform_corrected_odom_ = false;
  
  bool disable_velocity_before_tracker_activation_ = true;
  bool use_velocity_in_drift_model_ = false;
  mutable std::mutex mutex_use_velocity_in_drift_model_;
  mutable std::mutex mutex_disable_velocity_before_tracker_activation_;
  
  bool do_velocity_correction_x_ = false;
  bool do_velocity_correction_y_ = false;
  bool do_velocity_correction_z_ = false;
  
  bool stop_using_hw_api_ = false;
  mutable std::mutex mutex_stop_using_hw_api_;

  std::string uav_name_;
  std::string output_frame_;
  std::string output_frame_for_drifting_odom_;
  std::string ns_drift_corrected_frame_id_;
  
  std::string commit_hash_;
  std::string git_changes_;

  // | -------------- check if measurement is valid ------------- |
  double minimal_fusable_z_;
  
  double gate_size_threshold_;
  mutable std::mutex mutex_gate_size_threshold_;

  // | --------------- correction noise parameters -------------- |
  int correction_noise_method_ = 0;
  mutable std::mutex mutex_correction_noise_method_;
  
  correction_noise_params_t correction_noise_params_x_;
  correction_noise_params_t correction_noise_params_y_;
  correction_noise_params_t correction_noise_params_z_;
  correction_noise_params_t correction_noise_params_yaw_;
     
  mutable std::mutex mutex_correction_noise_params_x_;
  mutable std::mutex mutex_correction_noise_params_y_;
  mutable std::mutex mutex_correction_noise_params_z_;
  mutable std::mutex mutex_correction_noise_params_yaw_;

  mutable std::mutex mutex_gate_size_threshold_xy_correction_noise_;
  double gate_size_threshold_xy_correction_noise_;
  double confidence_x_passthrough_value_;
  double confidence_y_passthrough_value_;

  // | ------------------- skipping correction ------------------ |
  bool use_correction_skipping_ = false;
  mutable std::mutex mutex_use_correction_skipping_;
  
  double max_corr_size_sq_x_;
  double max_corr_size_sq_y_;
  double max_corr_size_sq_z_;
  double max_corr_size_sq_yaw_;
  
  mutable std::mutex mutex_max_corr_size_sq_x_;
  mutable std::mutex mutex_max_corr_size_sq_y_;
  mutable std::mutex mutex_max_corr_size_sq_z_;
  mutable std::mutex mutex_max_corr_size_sq_yaw_;

  // | ------------------ saturation parameters ----------------- |
  int saturation_method_ = 0;
  mutable std::mutex mutex_saturation_method_;
  bool enable_correction_saturation_ = 0;
  mutable std::mutex mutex_enable_correction_saturation_;
  
  saturation_params_t saturation_params_x_;
  saturation_params_t saturation_params_y_;
  saturation_params_t saturation_params_z_;
  saturation_params_t saturation_params_yaw_;
  
  mutable std::mutex mutex_saturation_params_x_;
  mutable std::mutex mutex_saturation_params_y_;
  mutable std::mutex mutex_saturation_params_z_;
  mutable std::mutex mutex_saturation_params_yaw_;

  // | ------------------ confidence parameters ----------------- | 
  double confidence_threshold_; 
  mutable std::mutex mutex_confidence_threshold_;

  // | -------------------------- debug ------------------------- | 
  vins_drift_estimator::debug_msg last_debug_msg_;
  mutable std::mutex mutex_debug_msg_;

  // | ----------------------- subscribers ---------------------- |
  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_drifting_odom_;
  void callbackDriftingOdom(const nav_msgs::Odometry::ConstPtr msg);

  mrs_lib::SubscribeHandler<geometry_msgs::PoseWithCovarianceStamped> sh_drift_correction_pose_;
  void callbackDriftCorrectionPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
  
  mrs_lib::SubscribeHandler<geometry_msgs::QuaternionStamped> sh_orientation_;
  void callbackOrientation(const geometry_msgs::QuaternionStamped::ConstPtr msg);
  
  mrs_lib::SubscribeHandler<geometry_msgs::Vector3Stamped> sh_angular_velocity_;
  void callbackAngularVelocity(const geometry_msgs::Vector3Stamped::ConstPtr msg);
  
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diagnostics_;
  void callbackControlManagerDiagnostics(mrs_msgs::ControlManagerDiagnostics::ConstPtr msg);

  // | ----------------------- publishers ----------------------- |
  mrs_lib::PublisherHandler<nav_msgs::Odometry>       ph_drifting_odom_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry>       ph_estimated_drift_odom_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry>       ph_internal_estimated_drift_odom_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry>       ph_corrected_odom_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry>       ph_corrected_internal_odom_;
  mrs_lib::PublisherHandler<vins_drift_estimator::debug_msg> ph_debug_msg_;
  mrs_lib::PublisherHandler<std_msgs::String>         ph_mrs_uav_status_;
  mrs_lib::PublisherHandler<nav_msgs::Odometry>         ph_corrected_odom_100hz_;

  // | ------------------------- timers ------------------------- |
  ros::Timer mrs_uav_status_timer_;
  ros::Timer main_timer_;
  void mrsUavStatusTimer([[maybe_unused]] const ros::TimerEvent& event);
  void mainTimer([[maybe_unused]] const ros::TimerEvent& event);

  // | -------------------------- LKFs -------------------------- |
  lkf_state_t lkf_x_;
  lkf_state_t lkf_y_;
  lkf_state_t lkf_z_;
  lkf_state_t lkf_yaw_;

  double x_friction_ = 0;
  double y_friction_ = 0;
  double z_friction_ = 0;
  double yaw_friction_ = 0;

  ros::Time t_last_prediction_;
  bool first_correction_ = true;

  // | ------------------------ functions ----------------------- |
  void   setDt(const double dt);
  void   generateA(double dt);
  void   generateB();
  void   setQ(const double pos_noise_x, const double pos_noise_y, const double pos_noise_z, const double vel_noise_x, const double vel_noise_y, const double vel_noise_z, const double yaw_noise, const double yaw_rate_noise);

  void publishEstimatedDrift();
  void publishCorrectedOdom(const nav_msgs::Odometry::ConstPtr& drifting_odom);
  void publishDriftingOdom(const nav_msgs::Odometry::ConstPtr& drifting_odom);

  void publishDebugMsg(
      const ros::Time correction_pose_stamp, const ros::Time correction_start_stamp, const ros::Time correction_end_stamp,
      const double noise_x, const double noise_y, const double noise_z, const double noise_yaw, 
      const double saturate_val_x, const double saturate_val_y, const double saturate_val_z, const double saturate_val_yaw,
      const double z_x, const double z_y, const double z_z, const double z_yaw, 
      const double innovation_x, const double innovation_y, const double innovation_z, const double innovation_yaw, 
      const bool discarding_correction_due_confidence, const bool discarding_correction_due_invalid_measurement, const double confidence, const double confidence_threshold, 
      const double mean_projection_error, const double number_of_gates, const double largest_gate_size, 
      const bool skipping_correction_x, const bool skipping_correction_y, const bool skipping_correction_z, const bool skipping_correction_yaw
  );

  Eigen::MatrixXd composePoseCovarianceMatrix();
  Eigen::MatrixXd composeTwistCovarianceMatrix();
  boost::array<double, 3 * drift_estimator_model::n_states * 3 * drift_estimator_model::n_states> convertEigenToBoostArray(const Eigen::MatrixXd &eigen_matrix);

  bool isCorrectionTooLarge(const z_t &z, const x_t &x, const double max_corr_size_sq);
  std::tuple<z_t, double>  saturate(const z_t &z, const x_t &x, const double confidence, const saturation_params_t saturation_params);
  z_t  computeSaturation(const z_t &z, const x_t &x, const double saturate_val);
  double scaleCorrectionNoise(const double confidence, const correction_noise_params_t correction_noise_params);

  static geometry_msgs::Vector3 rotateVector(const geometry_msgs::Vector3& vec_in, const geometry_msgs::Quaternion& q_in);
   
  geometry_msgs::Quaternion normalizeQuaternion(const geometry_msgs::Quaternion& q_msg);
  std::optional<nav_msgs::Odometry> findClosestDriftingOdomMessage(const ros::Time timestamp);
  
  double sigmoid(const double x, const double a, const double b, const double c, const double d);
  double fraction_function(const double x, const double a, const double b);

  int mapSaturationMethodString(const std::string method_name);
  int mapCorrectionNoiseMethodName(const std::string method_name);

  bool checkIfMeasurementIsValid(double confidence, double mean_projection_error, double number_of_gates, double largest_gate_size);
  bool isYawFusable(double are_gates_coplanar);

  // | ------------------- dynamic reconfigure ------------------ |
  boost::recursive_mutex mutex_dynamic_reconfigure_;
  typedef vins_drift_estimator::dynparamConfig Config;
  typedef dynamic_reconfigure::Server<vins_drift_estimator::dynparamConfig> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer>  reconfigure_server_;
  void  callbackDynamicReconfigure(Config& config, uint32_t level);
  vins_drift_estimator::dynparamConfig last_drs_config_;
  std::mutex mutex_dynparam_;

public:
  VinsDriftEstimator(){};
  ~VinsDriftEstimator(){};

  void onInit();
};

}  // namespace vins_drift_estimator

#endif

