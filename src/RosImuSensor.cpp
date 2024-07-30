#include "RosImuSensor.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

RosImuSensor::~RosImuSensor() = default;

void RosImuSensor::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  // load config
  referenceFrame = config("reference_frame", (std::string) "");
  verbose = config("verbose", false);
  ros_imu_sensor_ = config("ros_imu_sensor", false);
  imu_sensor_topic_ = config("ros_topic_sensor", (std::string) "");
  // config loaded
  
  if(ros_imu_sensor_)
  {
    // Initializing ROS node
    nh_ = mc_rtc::ROSBridge::get_node_handle();
    spinThread_ = std::thread(std::bind(&RosImuSensor::rosSpinner, this));

    mc_rtc::log::info("[RosImuSensor][ROS] Subscribing to {}", imu_sensor_topic_);

    imu_sub_.subscribe(*nh_, imu_sensor_topic_);
    imu_sub_.maxTime(maxTime_);
  }
  mc_rtc::log::info("RosImuSensor::init called with configuration:\n{}", config.dump(true, true));
}

void RosImuSensor::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("RosImuSensor::reset called");
}

void RosImuSensor::before(mc_control::MCGlobalController &)
{
  //mc_rtc::log::info("RosImuSensor::before");
  mc_rtc::log::info("[RosImuSensor][ROS] Linear acceleration: {} | Angular velocity: {}",
    imu_sub_.data().value().linear().transpose(),
    imu_sub_.data().value().angular().transpose()
  );
}

void RosImuSensor::after(mc_control::MCGlobalController & controller)
{
  //mc_rtc::log::info("RosImuSensor::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration RosImuSensor::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

void RosImuSensor::rosSpinner(void)
{
  mc_rtc::log::info("[RosImuSensor][ROS Spinner] thread created for imu sensor reading");
  ros::Rate r(freq_);
  while(ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
  mc_rtc::log::info("[RosImuSensor][ROS Spinner] spinner destroyed");
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("RosImuSensor", mc_plugin::RosImuSensor)
