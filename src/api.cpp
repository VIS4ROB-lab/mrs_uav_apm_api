/* includes //{ */

#include <mrs_uav_hw_api/api.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

/* #include <mrs_modules_msgs/Bestpos.h> */

#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/gps_conversions.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/subscriber_handler.h>

#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <mavros_msgs/msg/actuator_control.hpp>
#include <mavros_msgs/msg/altitude.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <mavros_msgs/msg/gpsraw.hpp>
#include <mavros_msgs/msg/rc_in.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

//}

/* defines //{ */

#define PWM_MIDDLE 1500
#define PWM_MIN 1000
#define PWM_MAX 2000
#define PWM_DEADBAND 200
#define PWM_RANGE PWM_MAX - PWM_MIN

//}

/* typedefs //{ */

#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif

//}

namespace mrs_uav_px4_api {

/* class MrsUavPx4Api //{ */

class MrsUavPx4Api : public mrs_uav_hw_api::MrsUavHwApi {
 public:
  ~MrsUavPx4Api() {};

  void initialize(
      const rclcpp::Node::SharedPtr& node,
      std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers);

  void destroy();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // | --------------------- status methods --------------------- |

  mrs_msgs::msg::HwApiStatus getStatus();
  mrs_msgs::msg::HwApiCapabilities getCapabilities();

  // | --------------------- topic callbacks -------------------- |

  bool callbackActuatorCmd(
      const mrs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr msg);
  bool callbackControlGroupCmd(
      const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg);
  bool callbackAttitudeRateCmd(
      const mrs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg);
  bool callbackAttitudeCmd(
      const mrs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg);
  bool callbackAccelerationHdgRateCmd(
      const mrs_msgs::msg::HwApiAccelerationHdgRateCmd::ConstSharedPtr msg);
  bool callbackAccelerationHdgCmd(
      const mrs_msgs::msg::HwApiAccelerationHdgCmd::ConstSharedPtr msg);
  bool callbackVelocityHdgRateCmd(
      const mrs_msgs::msg::HwApiVelocityHdgRateCmd::ConstSharedPtr msg);
  bool callbackVelocityHdgCmd(
      const mrs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr msg);
  bool callbackPositionCmd(
      const mrs_msgs::msg::HwApiPositionCmd::ConstSharedPtr msg);

  void callbackTrackerCmd(
      const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg);

  // | -------------------- service callbacks ------------------- |

  std::tuple<bool, std::string> callbackArming(const bool& request);
  std::tuple<bool, std::string> callbackOffboard(void);

 private:
  bool is_initialized_ = false;

  std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers_;

  rclcpp::Time last_mavros_state_time_;
  std::mutex mutex_last_mavros_state_time_;

  // | ----------------------- parameters ----------------------- |

  mrs_msgs::msg::HwApiCapabilities _capabilities_;

  std::string _uav_name_;
  std::string _body_frame_name_;
  std::string _world_frame_name_;

  double _mavros_timeout_;
  double _mavros_passable_delay_;

  bool _simulation_;

  double _sim_rtk_utm_x_;
  double _sim_rtk_utm_y_;
  std::string _sim_rtk_utm_zone_;
  double _sim_rtk_amsl_;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<mavros_msgs::srv::CommandLong>
      sch_mavros_command_long_;
  mrs_lib::ServiceClientHandler<mavros_msgs::srv::SetMode> sch_mavros_mode_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry> sh_ground_truth_;
  mrs_lib::SubscriberHandler<mavros_msgs::msg::State> sh_mavros_state_;
  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry> sh_mavros_odometry_local_;
  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry> sh_mavros_odometry_in_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix> sh_mavros_gps_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::Range>
      sh_mavros_distance_sensor_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu> sh_mavros_imu_;
  mrs_lib::SubscriberHandler<std_msgs::msg::Float64>
      sh_mavros_magnetometer_heading_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField>
      sh_mavros_magnetic_field_;
  mrs_lib::SubscriberHandler<mavros_msgs::msg::RCIn> sh_mavros_rc_;
  mrs_lib::SubscriberHandler<mavros_msgs::msg::Altitude> sh_mavros_altitude_;
  mrs_lib::SubscriberHandler<mavros_msgs::msg::GPSRAW> sh_gps_status_raw_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::BatteryState> sh_mavros_battery_;
  /* mrs_lib::SubscriberHandler<mrs_modules_msgs::msg::Bestpos> sh_rtk_; */

  void callbackGroundTruth(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void callbackMavrosState(const mavros_msgs::msg::State::ConstSharedPtr msg);
  void callbackOdometryLocal(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void callbackOdometryIn(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void callbackNavsatFix(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void callbackDistanceSensor(
      const sensor_msgs::msg::Range::ConstSharedPtr msg);
  void callbackImu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void callbackMagnetometer(const std_msgs::msg::Float64::ConstSharedPtr msg);
  void callbackMagneticField(
      const sensor_msgs::msg::MagneticField::ConstSharedPtr msg);
  void callbackRC(const mavros_msgs::msg::RCIn::ConstSharedPtr msg);
  void callbackAltitude(const mavros_msgs::msg::Altitude::ConstSharedPtr msg);
  void callbackGpsStatusRaw(const mavros_msgs::msg::GPSRAW::ConstSharedPtr msg);
  void callbackBattery(
      const sensor_msgs::msg::BatteryState::ConstSharedPtr msg);
  /* void                                                 callbackRTK(const
   * mrs_modules_msgs::msg::Bestpos::ConstSharedPtr msg); */

  void timeoutMavrosState(void);

  double RCChannelToRange(const double& rc_value);

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<mavros_msgs::msg::AttitudeTarget>
      ph_mavros_attitude_target_;
  mrs_lib::PublisherHandler<mavros_msgs::msg::ActuatorControl>
      ph_mavros_actuator_control_;

  // | ------------------------- timers ------------------------- |

  std::shared_ptr<TimerType> timer_main_;

  void timerMain();

  // | ------------------------ variables ----------------------- |

  std::atomic<bool> offboard_ = false;
  std::string mode_;
  std::atomic<bool> armed_ = false;
  std::atomic<bool> connected_ = false;
  std::mutex mutex_status_;
};

//}

// --------------------------------------------------------------
// |                   controller's interface                   |
// --------------------------------------------------------------

/* initialize() //{ */

void MrsUavPx4Api::initialize(
    const rclcpp::Node::SharedPtr& node,
    std::shared_ptr<mrs_uav_hw_api::CommonHandlers_t> common_handlers) {
  node_ = node;
  clock_ = node_->get_clock();
  common_handlers_ = common_handlers;

  _uav_name_ = common_handlers->getUavName();
  _body_frame_name_ = common_handlers->getBodyFrameName();
  _world_frame_name_ = common_handlers->getWorldFrameName();

  _capabilities_.api_name = "Px4Api";

  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  last_mavros_state_time_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader local_param_loader(node_, "MrsUavPx4Api");

  std::vector<std::string> config_files;
  common_handlers_->main_param_loader->loadParamReusable("configs",
                                                         config_files);

  common_handlers_->main_param_loader->loadParam("simulation", _simulation_);

  if (!common_handlers_->main_param_loader->loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  for (auto config_file : config_files) {
    RCLCPP_INFO(node_->get_logger(), "loading config file '%s'",
                config_file.c_str());
    local_param_loader.addYamlFile(config_file);
  }

  local_param_loader.loadParam("mavros_timeout", _mavros_timeout_);
  local_param_loader.loadParam("mavros_passable_delay",
                               _mavros_passable_delay_);

  local_param_loader.loadParam("gnss/utm_x", _sim_rtk_utm_x_);
  local_param_loader.loadParam("gnss/utm_y", _sim_rtk_utm_y_);
  local_param_loader.loadParam("gnss/utm_zone", _sim_rtk_utm_zone_);
  local_param_loader.loadParam("gnss/amsl", _sim_rtk_amsl_);

  local_param_loader.loadParam("input_mode/control_group",
                               (bool&)_capabilities_.accepts_control_group_cmd);
  local_param_loader.loadParam("input_mode/attitude_rate",
                               (bool&)_capabilities_.accepts_attitude_rate_cmd);
  local_param_loader.loadParam("input_mode/attitude",
                               (bool&)_capabilities_.accepts_attitude_cmd);

  local_param_loader.loadParam("outputs/distance_sensor",
                               (bool&)_capabilities_.produces_distance_sensor);
  local_param_loader.loadParam("outputs/gnss",
                               (bool&)_capabilities_.produces_gnss);
  local_param_loader.loadParam("outputs/gnss_status",
                               (bool&)_capabilities_.produces_gnss_status);
  local_param_loader.loadParam("outputs/rtk",
                               (bool&)_capabilities_.produces_rtk);
  local_param_loader.loadParam("outputs/ground_truth",
                               (bool&)_capabilities_.produces_ground_truth);
  local_param_loader.loadParam("outputs/imu",
                               (bool&)_capabilities_.produces_imu);
  local_param_loader.loadParam("outputs/altitude",
                               (bool&)_capabilities_.produces_altitude);
  local_param_loader.loadParam(
      "outputs/magnetometer_heading",
      (bool&)_capabilities_.produces_magnetometer_heading);
  local_param_loader.loadParam("outputs/magnetic_field",
                               (bool&)_capabilities_.produces_magnetic_field);
  local_param_loader.loadParam("outputs/rc_channels",
                               (bool&)_capabilities_.produces_rc_channels);
  local_param_loader.loadParam("outputs/battery_state",
                               (bool&)_capabilities_.produces_battery_state);
  local_param_loader.loadParam("outputs/position",
                               (bool&)_capabilities_.produces_position);
  local_param_loader.loadParam("outputs/orientation",
                               (bool&)_capabilities_.produces_orientation);
  local_param_loader.loadParam("outputs/velocity",
                               (bool&)_capabilities_.produces_velocity);
  local_param_loader.loadParam("outputs/angular_velocity",
                               (bool&)_capabilities_.produces_angular_velocity);
  local_param_loader.loadParam("outputs/odometry",
                               (bool&)_capabilities_.produces_odometry);

  if (!local_param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all parameters!");
    rclcpp::shutdown();
  }

  // | --------------------- service clients -------------------- |

  sch_mavros_command_long_ =
      mrs_lib::ServiceClientHandler<mavros_msgs::srv::CommandLong>(
          node_, "~/mavros_cmd_out", callback_group_);
  sch_mavros_mode_ = mrs_lib::ServiceClientHandler<mavros_msgs::srv::SetMode>(
      node_, "~/mavros_set_mode_out", callback_group_);

  // | ----------------------- subscribers ---------------------- |

  rclcpp::QoS qos_profile(10);
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node = node_;
  shopts.node_name = "MrsHwPx4Api";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe = true;
  shopts.autostart = true;
  shopts.subscription_options.callback_group = callback_group_;
  shopts.qos = qos_profile;

  if (_simulation_) {
    sh_ground_truth_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(
        shopts, "~/ground_truth_in", &MrsUavPx4Api::callbackGroundTruth, this);
  }

  /* if (!_simulation_) { */
  /* sh_rtk_ =
   * mrs_lib::SubscriberHandler<mrs_modules_msgs::msg::Bestpos>(shopts,
   * "rtk_in", &MrsUavPx4Api::callbackRTK, this); */
  /* } */

  sh_mavros_state_ = mrs_lib::SubscriberHandler<mavros_msgs::msg::State>(
      shopts, "~/mavros_state_in", &MrsUavPx4Api::callbackMavrosState, this);

  sh_mavros_odometry_local_ =
      mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(
          shopts, "~/mavros_local_position_in",
          &MrsUavPx4Api::callbackOdometryLocal, this);

  sh_mavros_odometry_in_ = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(
      shopts, "~/mavros_odometry_in", &MrsUavPx4Api::callbackOdometryIn, this);

  sh_mavros_gps_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::NavSatFix>(
      shopts, "~/mavros_global_position_in", &MrsUavPx4Api::callbackNavsatFix,
      this);

  sh_mavros_distance_sensor_ =
      mrs_lib::SubscriberHandler<sensor_msgs::msg::Range>(
          shopts, "~/mavros_garmin_in", &MrsUavPx4Api::callbackDistanceSensor,
          this);

  sh_mavros_imu_ = mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>(
      shopts, "~/mavros_imu_in", &MrsUavPx4Api::callbackImu, this);

  sh_mavros_magnetometer_heading_ =
      mrs_lib::SubscriberHandler<std_msgs::msg::Float64>(
          shopts, "~/mavros_magnetometer_in",
          &MrsUavPx4Api::callbackMagnetometer, this);

  sh_mavros_magnetic_field_ =
      mrs_lib::SubscriberHandler<sensor_msgs::msg::MagneticField>(
          shopts, "~/mavros_magnetic_field_in",
          &MrsUavPx4Api::callbackMagneticField, this);

  sh_mavros_rc_ = mrs_lib::SubscriberHandler<mavros_msgs::msg::RCIn>(
      shopts, "~/mavros_rc_in", &MrsUavPx4Api::callbackRC, this);

  sh_mavros_altitude_ = mrs_lib::SubscriberHandler<mavros_msgs::msg::Altitude>(
      shopts, "~/mavros_altitude_in", &MrsUavPx4Api::callbackAltitude, this);

  sh_gps_status_raw_ = mrs_lib::SubscriberHandler<mavros_msgs::msg::GPSRAW>(
      shopts, "~/mavros_gps_status_raw_in", &MrsUavPx4Api::callbackGpsStatusRaw,
      this);

  sh_mavros_battery_ =
      mrs_lib::SubscriberHandler<sensor_msgs::msg::BatteryState>(
          shopts, "~/mavros_battery_in", &MrsUavPx4Api::callbackBattery, this);

  // | ----------------------- publishers ----------------------- |

  ph_mavros_attitude_target_ =
      mrs_lib::PublisherHandler<mavros_msgs::msg::AttitudeTarget>(
          node_, "~/mavros_attitude_setpoint_out");
  ph_mavros_actuator_control_ =
      mrs_lib::PublisherHandler<mavros_msgs::msg::ActuatorControl>(
          node_, "~/mavros_actuator_control_out");

  // | ----------------------- finish init ---------------------- |

  {
    std::function<void()> callback_fcn =
        std::bind(&MrsUavPx4Api::timerMain, this);

    mrs_lib::TimerHandlerOptions opts;

    opts.node = node_;
    opts.autostart = true;
    opts.callback_group = callback_group_;

    timer_main_ =
        std::make_shared<TimerType>(opts, rclcpp::Rate(10.0), callback_fcn);
  }

  RCLCPP_INFO(node_->get_logger(), "initialized");

  is_initialized_ = true;
}

//}

/* destroy() //{ */

void MrsUavPx4Api::destroy() { timer_main_->stop(); }

//}

/* getStatus() //{ */

mrs_msgs::msg::HwApiStatus MrsUavPx4Api::getStatus() {
  mrs_msgs::msg::HwApiStatus status;

  status.stamp = clock_->now();

  {
    std::scoped_lock lock(mutex_status_);

    status.armed = armed_;
    status.offboard = offboard_;
    status.connected = connected_;
    status.mode = mode_;
  }

  return status;
}

//}

/* getCapabilities() //{ */

mrs_msgs::msg::HwApiCapabilities MrsUavPx4Api::getCapabilities() {
  _capabilities_.stamp = clock_->now();

  return _capabilities_;
}

//}

/* callbackControlActuatorCmd() //{ */

bool MrsUavPx4Api::callbackActuatorCmd(
    [[maybe_unused]] const mrs_msgs::msg::HwApiActuatorCmd::ConstSharedPtr
        msg) {
  RCLCPP_INFO_ONCE(node_->get_logger(), "getting actuator cmd");

  return false;
}

//}

/* callbackControlGroupCmd() //{ */

bool MrsUavPx4Api::callbackControlGroupCmd(
    const mrs_msgs::msg::HwApiControlGroupCmd::ConstSharedPtr msg) {
  RCLCPP_INFO_ONCE(node_->get_logger(), "getting control group cmd");

  if (!_capabilities_.accepts_control_group_cmd) {
    RCLCPP_ERROR(node_->get_logger(),
                 "the control group input is not enabled in the config file");
    return false;
  }

  mavros_msgs::msg::ActuatorControl msg_out;

  msg_out.header.frame_id = "base_link";
  msg_out.header.stamp = msg->stamp;

  msg_out.controls[0] = msg->roll;
  msg_out.controls[1] = -msg->pitch;
  msg_out.controls[2] = -msg->yaw;
  msg_out.controls[3] = msg->throttle;

  ph_mavros_actuator_control_.publish(msg_out);

  return true;
}

//}

/* callbackAttitudeRateCmd() //{ */

bool MrsUavPx4Api::callbackAttitudeRateCmd(
    const mrs_msgs::msg::HwApiAttitudeRateCmd::ConstSharedPtr msg) {
  RCLCPP_INFO_ONCE(node_->get_logger(), "getting attitude rate cmd");

  if (!_capabilities_.accepts_attitude_rate_cmd) {
    RCLCPP_ERROR_THROTTLE(
        node_->get_logger(), *clock_, 1000,
        "attitude rate input is not enabled in the config file");
    return false;
  }

  mavros_msgs::msg::AttitudeTarget attitude_target;

  attitude_target.header.frame_id = "base_link";
  attitude_target.header.stamp = msg->stamp;

  attitude_target.body_rate = msg->body_rate;
  attitude_target.thrust = msg->throttle;

  attitude_target.type_mask = attitude_target.IGNORE_ATTITUDE;

  ph_mavros_attitude_target_.publish(attitude_target);

  return true;
}

//}

/* callbackAttitudeCmd() //{ */

bool MrsUavPx4Api::callbackAttitudeCmd(
    const mrs_msgs::msg::HwApiAttitudeCmd::ConstSharedPtr msg) {
  RCLCPP_INFO_ONCE(node_->get_logger(), "getting attitude cmd");

  if (!_capabilities_.accepts_attitude_cmd) {
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000,
                          "attitude input is not enabled in the config file");
    return false;
  }

  mavros_msgs::msg::AttitudeTarget attitude_target;

  attitude_target.header.frame_id = "base_link";
  attitude_target.header.stamp = msg->stamp;

  attitude_target.orientation.x = msg->orientation.x;
  attitude_target.orientation.y = msg->orientation.y;
  attitude_target.orientation.z = msg->orientation.z;
  attitude_target.orientation.w = msg->orientation.w;

  attitude_target.thrust = msg->throttle;

  attitude_target.type_mask = attitude_target.IGNORE_YAW_RATE |
                              attitude_target.IGNORE_ROLL_RATE |
                              attitude_target.IGNORE_PITCH_RATE;

  ph_mavros_attitude_target_.publish(attitude_target);

  return true;
}

//}

/* callbackAccelerationHdgRateCmd() //{ */

bool MrsUavPx4Api::callbackAccelerationHdgRateCmd(
    [[maybe_unused]] const mrs_msgs::msg::HwApiAccelerationHdgRateCmd::
        ConstSharedPtr msg) {
  RCLCPP_INFO_ONCE(node_->get_logger(), "getting acceleration+hdg rate cmd");

  return false;
}

//}

/* callbackAccelerationHdgCmd() //{ */

bool MrsUavPx4Api::callbackAccelerationHdgCmd(
    [[maybe_unused]] const mrs_msgs::msg::HwApiAccelerationHdgCmd::
        ConstSharedPtr msg) {
  RCLCPP_INFO_ONCE(node_->get_logger(), "getting acceleration+hdg cmd");

  return false;
}

//}

/* callbackVelocityHdgRateCmd() //{ */

bool MrsUavPx4Api::callbackVelocityHdgRateCmd(
    [[maybe_unused]] const mrs_msgs::msg::HwApiVelocityHdgRateCmd::
        ConstSharedPtr msg) {
  RCLCPP_INFO_ONCE(node_->get_logger(), "getting velocity+hdg rate cmd");

  return false;
}

//}

/* callbackVelocityHdgCmd() //{ */

bool MrsUavPx4Api::callbackVelocityHdgCmd(
    [[maybe_unused]] const mrs_msgs::msg::HwApiVelocityHdgCmd::ConstSharedPtr
        msg) {
  RCLCPP_INFO_ONCE(node_->get_logger(), "getting velocity+hdg cmd");

  return false;
}

//}

/* callbackPositionCmd() //{ */

bool MrsUavPx4Api::callbackPositionCmd(
    [[maybe_unused]] const mrs_msgs::msg::HwApiPositionCmd::ConstSharedPtr
        msg) {
  RCLCPP_INFO_ONCE(node_->get_logger(), "getting position cmd");

  return false;
}

//}

/* callbackTrackerCmd() //{ */

void MrsUavPx4Api::callbackTrackerCmd(
    [[maybe_unused]] const mrs_msgs::msg::TrackerCommand::ConstSharedPtr msg) {}

//}

/* callbackArming() //{ */

std::tuple<bool, std::string> MrsUavPx4Api::callbackArming(
    [[maybe_unused]] const bool& request) {
  std::stringstream ss;

  auto srv_out = std::make_shared<mavros_msgs::srv::CommandLong::Request>();

  // when REALWORLD AND ARM:=TRUE
  if (!_simulation_ && request) {
    ss << "can not arm by service when not in simulation! You should arm the "
          "drone by the RC controller only!";
    RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000,
                                 "" << ss.str());

    return {false, ss.str()};
  }

  srv_out->broadcast = false;
  srv_out->command = 400;  // the code for arming
  srv_out->confirmation = true;

  srv_out->param1 = request ? 1 : 0;  // arm or disarm?
  srv_out->param2 =
      request ? 0 : 21196;  // 21196 allows to disarm even in mid-flight
  srv_out->param3 = 0;
  srv_out->param4 = 0;
  srv_out->param5 = 0;
  srv_out->param6 = 0;
  srv_out->param7 = 0;

  RCLCPP_INFO(node_->get_logger(), "calling for %s",
              request ? "arming" : "disarming");

  bool success = false;

  auto response = sch_mavros_command_long_.callSync(srv_out);

  if (response) {
    success = response.value()->success;

    if (success) {
      ss << "service call for " << (request ? "arming" : "disarming")
         << " was successful";
      RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000,
                                  "" << ss.str());

    } else {
      ss << "service call for " << (request ? "arming" : "disarming")
         << " failed";
      RCLCPP_ERROR_STREAM_THROTTLE(node_->get_logger(), *clock_, 1000,
                                   "" << ss.str());
    }

  } else {
    ss << "failed to call Mavros CommandLong service";
    RCLCPP_ERROR(node_->get_logger(), "%s", ss.str().c_str());
  }

  return {success, ss.str()};
}

//}

/* callbackOffboard() //{ */

std::tuple<bool, std::string> MrsUavPx4Api::callbackOffboard(void) {
  std::stringstream ss;

  auto srv_out = std::make_shared<mavros_msgs::srv::SetMode::Request>();

  srv_out->base_mode = 0;
  srv_out->custom_mode = "OFFBOARD";

  bool success = false;

  auto response = sch_mavros_mode_.callSync(srv_out);

  if (response) {
    if (response.value()->mode_sent != 1) {
      ss << "service call for offboard failed, returned "
         << response.value()->mode_sent;

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "%s",
                           ss.str().c_str());

    } else {
      ss << "switched to offboard mode";
      success = true;
    }
  } else {
    ss << "failed to call Mavros SetMode service";
    RCLCPP_ERROR_THROTTLE(node_->get_logger(), *clock_, 1000, "%s",
                          ss.str().c_str());
  }

  return {success, ss.str()};
}

//}

// | ------------------- additional methods ------------------- |

/* timeoutMavrosState() //{ */

void MrsUavPx4Api::timeoutMavrosState(void) {
  if (!is_initialized_) {
    return;
  }

  if (!sh_mavros_state_.hasMsg()) {
    return;
  }

  auto last_mavros_state_time = mrs_lib::get_mutexed(
      mutex_last_mavros_state_time_, last_mavros_state_time_);

  auto time = clock_->now() - last_mavros_state_time;

  if (time.seconds() > _mavros_timeout_) {
    {
      std::scoped_lock lock(mutex_status_);

      connected_ = false;
      offboard_ = false;
      armed_ = false;
      mode_ = "";
    }

    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *clock_, 1000,
        "Have not received Mavros state for more than '%.3f s'",
        time.seconds());
  }

  if (time.seconds() > _mavros_passable_delay_) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000,
                         "Not recieving Mavros state message for '%.3f s'! "
                         "Setup the PixHawk SD card!!",
                         time.seconds());
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *clock_, 1000,
        "This could be also caused by the not being PixHawk booted properly "
        "due to, e.g., antispark connector jerkyness.");
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *clock_, 1000,
        "The Mavros state should be supplied at 100 Hz to provided fast "
        "refresh rate on the state of the OFFBOARD mode.");
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *clock_, 1000,
        "If missing, the UAV could be disarmed by safety routines while not "
        "knowing it has switched to the MANUAL mode.");
  }
}

//}

/* RCChannelToRange() //{ */

double MrsUavPx4Api::RCChannelToRange(const double& rc_value) {
  double tmp_0_to_1 = (rc_value - double(PWM_MIN)) / (double(PWM_RANGE));

  if (tmp_0_to_1 > 1.0) {
    tmp_0_to_1 = 1.0;
  } else if (tmp_0_to_1 < 0.0) {
    tmp_0_to_1 = 0.0;
  }

  return tmp_0_to_1;
}

//}

// | ------------------------ callbacks ----------------------- |

/* //{ callbackMavrosState() */

void MrsUavPx4Api::callbackMavrosState(
    const mavros_msgs::msg::State::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting Mavros state");

  {
    std::scoped_lock lock(mutex_status_);

    offboard_ = msg->mode == "OFFBOARD";
    armed_ = msg->armed;
    connected_ = true;
    mode_ = msg->mode;
  }

  // | ----------------- publish the diagnostics ---------------- |

  rclcpp::Time timestamp = clock_->now();

  mrs_msgs::msg::HwApiStatus status;

  {
    std::scoped_lock lock(mutex_status_);

    status.stamp = timestamp;
    status.armed = armed_;
    status.offboard = offboard_;
    status.connected = connected_;
    status.mode = mode_;
  }

  mrs_lib::set_mutexed(mutex_last_mavros_state_time_, timestamp,
                       last_mavros_state_time_);

  common_handlers_->publishers.publishStatus(status);
}

//}

/* callbackOdometryLocal() //{ */

void MrsUavPx4Api::callbackOdometryLocal(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting Mavros's local odometry");

  auto odom = msg;

  // | -------------------- publish position -------------------- |

  if (_capabilities_.produces_position) {
    geometry_msgs::msg::PointStamped position;

    position.header.stamp = odom->header.stamp;
    position.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
    position.point = odom->pose.pose.position;

    common_handlers_->publishers.publishPosition(position);
  }

  // | -------------------- publish velocity -------------------- |

  if (_capabilities_.produces_velocity) {
    geometry_msgs::msg::Vector3Stamped velocity;

    velocity.header.stamp = odom->header.stamp;
    velocity.header.frame_id = _uav_name_ + "/" + _body_frame_name_;
    velocity.vector = odom->twist.twist.linear;

    common_handlers_->publishers.publishVelocity(velocity);
  }

  // | -------------------- publish odometry -------------------- |

  if (_capabilities_.produces_odometry) {
    common_handlers_->publishers.publishOdometry(*odom);
  }
}

//}

/* callbackOdometryIn() //{ */

void MrsUavPx4Api::callbackOdometryIn(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting Mavros's odometry in");

  auto odom = msg;

  // | ------------------- publish orientation ------------------ |

  if (_capabilities_.produces_orientation) {
    geometry_msgs::msg::QuaternionStamped orientation;

    orientation.header.stamp = odom->header.stamp;
    orientation.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
    orientation.quaternion = odom->pose.pose.orientation;

    common_handlers_->publishers.publishOrientation(orientation);
  }

  // | ---------------- publish angular velocity ---------------- |

  if (_capabilities_.produces_angular_velocity) {
    geometry_msgs::msg::Vector3Stamped angular_velocity;

    angular_velocity.header.stamp = odom->header.stamp;
    angular_velocity.header.frame_id = _uav_name_ + "/" + _body_frame_name_;
    angular_velocity.vector = odom->twist.twist.angular;

    common_handlers_->publishers.publishAngularVelocity(angular_velocity);
  }
}

//}

/* callbackNavsatFix() //{ */

void MrsUavPx4Api::callbackNavsatFix(
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting NavSat fix");

  if (_capabilities_.produces_gnss) {
    common_handlers_->publishers.publishGNSS(*msg);
  }
}

//}

/* callbackDistanceSensor() //{ */

void MrsUavPx4Api::callbackDistanceSensor(
    const sensor_msgs::msg::Range::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting distnace sensor");

  if (_capabilities_.produces_distance_sensor) {
    auto msg_out = *msg;

    msg_out.min_range = 0.001;

    common_handlers_->publishers.publishDistanceSensor(msg_out);
  }
}

//}

/* callbackImu() //{ */

void MrsUavPx4Api::callbackImu(
    const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting IMU");

  if (_capabilities_.produces_imu) {
    sensor_msgs::msg::Imu new_imu_msg = *msg;
    new_imu_msg.header.frame_id = _uav_name_ + "/" + _body_frame_name_;

    common_handlers_->publishers.publishIMU(new_imu_msg);
  }
}

//}

/* callbackCompass() //{ */

void MrsUavPx4Api::callbackMagnetometer(
    const std_msgs::msg::Float64::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting magnetometer heading");

  if (_capabilities_.produces_magnetometer_heading) {
    mrs_msgs::msg::Float64Stamped mag_out;
    mag_out.header.stamp = clock_->now();
    mag_out.header.frame_id = _uav_name_ + "/" + _world_frame_name_;
    mag_out.value = msg->data;

    common_handlers_->publishers.publishMagnetometerHeading(mag_out);
  }
}

//}

/* callbackMagneticField() //{ */

void MrsUavPx4Api::callbackMagneticField(
    const sensor_msgs::msg::MagneticField::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting magnetic field");

  if (_capabilities_.produces_magnetic_field) {
    common_handlers_->publishers.publishMagneticField(*msg);
  }
}

//}

/* callbackRC() //{ */

void MrsUavPx4Api::callbackRC(
    const mavros_msgs::msg::RCIn::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting RC");

  if (_capabilities_.produces_rc_channels) {
    mrs_msgs::msg::HwApiRcChannels rc_out;

    rc_out.stamp = msg->header.stamp;

    for (size_t i = 0; i < msg->channels.size(); i++) {
      rc_out.channels.push_back(RCChannelToRange(msg->channels[i]));
    }

    common_handlers_->publishers.publishRcChannels(rc_out);
  }
}

//}

/* callbackAltitude() //{ */

void MrsUavPx4Api::callbackAltitude(
    const mavros_msgs::msg::Altitude::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting Altitude");

  if (_capabilities_.produces_altitude) {
    mrs_msgs::msg::HwApiAltitude altitude_out;

    altitude_out.stamp = msg->header.stamp;
    altitude_out.amsl = msg->amsl;

    common_handlers_->publishers.publishAltitude(altitude_out);
  }
}

//}

/* callbackAltitude() //{ */

void MrsUavPx4Api::callbackGpsStatusRaw(
    const mavros_msgs::msg::GPSRAW::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting Gps Status Raw");

  if (_capabilities_.produces_gnss_status) {
    mrs_msgs::msg::GpsInfo gps_info_out;

    gps_info_out.stamp = msg->header.stamp;  // [GPS_FIX_TYPE] GPS fix type
    gps_info_out.fix_type = msg->fix_type;   // [GPS_FIX_TYPE] GPS fix type

    gps_info_out.lat =
        double(msg->lat) / 10000000;  // [deg] Latitude (WGS84, EGM96 ellipsoid)
    gps_info_out.lon = double(msg->lon) /
                       10000000;  // [deg] Longitude (WGS84, EGM96 ellipsoid)
    gps_info_out.alt =
        float(msg->alt) /
        1000;  // [m]  (MSL). Positive for up. Not WGS84 altitude.
    gps_info_out.eph = msg->eph;  // GPS HDOP horizontal dilution of position
                                  // (unitless). If unknown, set to: UINT16_MAX
    gps_info_out.epv = msg->epv;  // GPS VDOP vertical dilution of position
                                  // (unitless). If unknown, set to: UINT16_MAX
    gps_info_out.vel =
        float(msg->vel) /
        100;  // [m/s] GPS ground speed. If unknown, set to: UINT16_MAX
    gps_info_out.cog =
        float(msg->cog) / 100;  // [deg] Course over ground (NOT heading, but
                                // direction of movement), 0.0..359.99 degrees.
    gps_info_out.satellites_visible =
        msg->satellites_visible;  // Number of satellites visible. If unknown,
                                  // set to 255

    gps_info_out.alt_ellipsoid =
        float(msg->alt_ellipsoid) /
        1000;  // [m] Altitude (above WGS84, EGM96 ellipsoid). Positive for up.
    gps_info_out.h_acc =
        float(msg->h_acc) / 1000;  // [m] Position uncertainty. Positive for up.
    gps_info_out.v_acc =
        float(msg->v_acc) / 1000;  // [m] Altitude uncertainty. Positive for up.
    gps_info_out.vel_acc = float(msg->vel_acc) /
                           1000;  // [m/s] Speed uncertainty. Positive for up.
    gps_info_out.hdg_acc =
        float(msg->hdg_acc) / 1000;  // [deg] Heading / track uncertainty
    gps_info_out.yaw =
        float(msg->yaw) / 100;  // [deg] Yaw in earth frame from north.
    gps_info_out.dgps_num_sats = msg->dgps_numch;  // Number of DGPS satellites
    gps_info_out.dgps_age =
        float(msg->dgps_age) / 1000;  // [s] Age of DGPS info
    gps_info_out.baseline_dist = 0;   // [m] distance to the basestation, not
                                      // supported by the GPSRAW message

    common_handlers_->publishers.publishGNSSStatus(gps_info_out);
  }
}

//}

/* callbackBattery() //{ */

void MrsUavPx4Api::callbackBattery(
    const sensor_msgs::msg::BatteryState::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting battery");

  if (_capabilities_.produces_battery_state) {
    common_handlers_->publishers.publishBatteryState(*msg);
  }
}

//}

/* callbackGroundTruth() //{ */

void MrsUavPx4Api::callbackGroundTruth(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting ground truth");

  auto odom = msg;

  // | ------------------ publish ground truth ------------------ |

  if (_capabilities_.produces_ground_truth) {
    nav_msgs::msg::Odometry gt = *msg;

    // if frame_id is "/world", "world", "/map" or "map" gazebo reports
    // velocitites in global world frame so we need to transform them to body
    // frame
    if (msg->header.frame_id == "/world" || msg->header.frame_id == "world" ||
        msg->header.frame_id == "/map" || msg->header.frame_id == "map") {
      RCLCPP_INFO_ONCE(node_->get_logger(),
                       "transforming Gazebo ground truth velocities from world "
                       "to body frame");

      Eigen::Matrix3d R =
          mrs_lib::AttitudeConverter(msg->pose.pose.orientation);

      Eigen::Vector3d lin_vel_world(msg->twist.twist.linear.x,
                                    msg->twist.twist.linear.y,
                                    msg->twist.twist.linear.z);
      Eigen::Vector3d lin_vel_body = R.inverse() * lin_vel_world;

      Eigen::Vector3d angular_vel_world(msg->twist.twist.angular.x,
                                        msg->twist.twist.angular.y,
                                        msg->twist.twist.angular.z);
      Eigen::Vector3d angular_vel_body = R.inverse() * angular_vel_world;

      gt.twist.twist.linear.x = lin_vel_body[0];
      gt.twist.twist.linear.y = lin_vel_body[1];
      gt.twist.twist.linear.z = lin_vel_body[2];

      gt.twist.twist.angular.x = angular_vel_body[0];
      gt.twist.twist.angular.y = angular_vel_body[1];
      gt.twist.twist.angular.z = angular_vel_body[2];
    }

    common_handlers_->publishers.publishGroundTruth(gt);
  }

  if (_capabilities_.produces_rtk) {
    double lat;
    double lon;

    mrs_lib::UTMtoLL(msg->pose.pose.position.y + _sim_rtk_utm_y_,
                     msg->pose.pose.position.x + _sim_rtk_utm_x_,
                     _sim_rtk_utm_zone_, lat, lon);

    sensor_msgs::msg::NavSatFix gnss;

    gnss.header.stamp = msg->header.stamp;

    gnss.latitude = lat;
    gnss.longitude = lon;
    gnss.altitude = msg->pose.pose.position.z + _sim_rtk_amsl_;

    mrs_msgs::msg::RtkGps rtk;

    rtk.header.stamp = msg->header.stamp;
    rtk.header.frame_id = "gps";

    rtk.gps.latitude = lat;
    rtk.gps.longitude = lon;
    rtk.gps.altitude = msg->pose.pose.position.z + _sim_rtk_amsl_;
    rtk.gps.covariance[0] = std::pow(0.1, 2);
    rtk.gps.covariance[4] = std::pow(0.1, 2);
    rtk.gps.covariance[8] = std::pow(0.1, 2);

    rtk.fix_type.fix_type = rtk.fix_type.RTK_FIX;

    rtk.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;

    common_handlers_->publishers.publishRTK(rtk);
  }
}

//}

/* /1* callbackRTK() //{ *1/ */

/* void MrsUavPx4Api::callbackRTK(const
 * mrs_modules_msgs::msg::Bestpos::ConstSharedPtr msg) { */

/*   if (!is_initialized_) { */
/*     return; */
/*   } */

/*   ROS_INFO_ONCE("[MrsUavPx4Api]: getting rtk"); */

/*   mrs_msgs::msg::RtkGps rtk_msg_out; */

/*   rtk_msg_out.gps.latitude  = msg->latitude; */
/*   rtk_msg_out.gps.longitude = msg->longitude; */
/*   rtk_msg_out.gps.altitude  = msg->height; */

/*   rtk_msg_out.header.stamp    = clock_->now(); */
/*   rtk_msg_out.header.frame_id = _uav_name_ + "/" + _body_frame_name_; */

/*   if (msg->position_type == "L1_INT") { */
/*     rtk_msg_out.status.status     =
 * sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX; */
/*     rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.RTK_FIX; */

/*   } else if (msg->position_type == "L1_FLOAT") { */
/*     rtk_msg_out.status.status     =
 * sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX; */
/*     rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.RTK_FLOAT; */

/*   } else if (msg->position_type == "PSRDIFF") { */
/*     rtk_msg_out.status.status     =
 * sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX; */
/*     rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.DGPS; */

/*   } else if (msg->position_type == "SINGLE") { */
/*     rtk_msg_out.status.status     =
 * sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX; */
/*     rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.SPS; */

/*   } else if (msg->position_type == "NONE") { */
/*     rtk_msg_out.status.status     =
 * sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX; */
/*     rtk_msg_out.fix_type.fix_type = rtk_msg_out.fix_type.NO_FIX; */
/*   } */

/*   common_handlers_->publishers.publishRTK(rtk_msg_out); */
/* } */

/* //} */

/* timerMain() //{ */

void MrsUavPx4Api::timerMain() {
  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "main timer spinning");

  timeoutMavrosState();
}

//}

}  // namespace mrs_uav_px4_api

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_uav_px4_api::MrsUavPx4Api,
                       mrs_uav_hw_api::MrsUavHwApi)
