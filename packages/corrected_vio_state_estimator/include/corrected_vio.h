#ifndef ESTIMATORS_STATE_CORRECTEDVIO_H
#define ESTIMATORS_STATE_CORRECTEDVIO_H

/* includes //{ */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <mrs_lib/lkf.h>
#include <mrs_lib/profiler.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/transformer.h>

#include <mrs_uav_managers/state_estimator.h>

//}

namespace mrs_uav_state_estimators
{

namespace corrected_vio
{
const char name[]         = "corrected_vio";
const char frame_id[]     = "corrected_vio_origin";
const char package_name[] = "corrected_vio_state_estimators";

const bool is_core_plugin = true;

using namespace mrs_uav_managers::estimation_manager;

class CorrectedVio : public mrs_uav_managers::StateEstimator {

  using CommonHandlers_t = mrs_uav_managers::estimation_manager::CommonHandlers_t;

private:
  const std::string package_name_ = "corrected_vio_state_estimator";

  const std::string est_lat_name_ = "lat_corrected_vio";

  const std::string est_alt_name_ = "alt_corrected_vio";

  const std::string est_hdg_name_ = "hdg_corrected_vio";

  const bool is_core_plugin_;

  mrs_lib::SubscribeHandler<nav_msgs::Odometry> sh_corrected_vio_odom_;
  void                                          callbackCorrectedVioOdom(const nav_msgs::Odometry::ConstPtr msg);
  double                                        _critical_timeout_corrected_vio_odom_;
  std::string                                   msg_topic_;

  ros::Timer       timer_check_corrected_vio_odom_hz_;
  void             timerCheckCorrectedVioOdomHz(const ros::TimerEvent &event);
  std::atomic<int> counter_odom_msgs_ = 0;
  ros::Time        t_check_hz_last_;
  double           prev_avg_hz_ = 0;
  bool             kickoff_     = false;
  bool             is_override_frame_id_ = false;
  
  const int n_states = 6;

  ros::Timer                 timer_update_;
  void                       timerUpdate(const ros::TimerEvent &event);
  nav_msgs::OdometryConstPtr prev_msg_;
  bool                       first_iter_ = true;

  bool isConverged();

  void waitForEstimationInitialization();

public:
  CorrectedVio() : StateEstimator(corrected_vio::name, corrected_vio::frame_id, corrected_vio::package_name), is_core_plugin_(is_core_plugin) {
  }

  ~CorrectedVio(void) {
  }

  void initialize(ros::NodeHandle &parent_nh, const std::shared_ptr<CommonHandlers_t> &ch, const std::shared_ptr<PrivateHandlers_t> &ph) override;
  bool start(void) override;
  bool pause(void) override;
  bool reset(void) override;

  void updateUavState();
  bool setUavState(const mrs_msgs::UavState &uav_state) override;
};

}  // namespace corrected_vio 

}  // namespace mrs_uav_state_estimators

#endif  // ESTIMATORS_STATE_CORRECTEDVIO_H

