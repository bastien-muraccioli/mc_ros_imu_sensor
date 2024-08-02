#include "RosImuSensor.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_rbdyn/BodySensor.h>

namespace mc_plugin
{

RosImuSensor::~RosImuSensor() = default;

void RosImuSensor::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("RosImuSensor::init called with configuration:\n{}", config.dump(true, true));

  // load config
  referenceFrame = config("reference_frame", (std::string) "FT_sensor_imu");
  verbose = config("verbose", false);
  ros_imu_sensor_ = config("ros_imu_sensor", false);
  imu_sensor_topic_ = config("ros_topic_sensor", (std::string) "/bus0/ft_sensor0/ft_sensor_readings/imu");
  freq_ = config("imu_freq", (double) 1000);
  bodySensor_name_ = config("body_sensor_name", (std::string) "Accelerometer");
  // config loaded
  maxTime_ = 1/freq_;

  if(ros_imu_sensor_)
  {
    // Initializing ROS node
    nh_ = mc_rtc::ROSBridge::get_node_handle();
    spinThread_ = std::thread(std::bind(&RosImuSensor::rosSpinner, this));

    mc_rtc::log::info("[RosImuSensor][ROS] Subscribing to {}", imu_sensor_topic_);

    imu_sub_.subscribe(*nh_, imu_sensor_topic_);
    imu_sub_.maxTime(maxTime_);
  }

  // Check if the body sensor exists
  if(!controller.controller().robot().hasBodySensor(bodySensor_name_))
  {
    mc_rtc::log::info("[RosImuSensor] Body sensor {} does not exist in the robot. Creating Body Sensor...", bodySensor_name_);
    // Create the body sensor
    mc_rbdyn::BodySensor imu_sensor(bodySensor_name_, referenceFrame, sva::PTransformd::Identity());
    // Add the body sensor to the robot
    controller.controller().robot().addBodySensor(imu_sensor);
  }
  else
  {
    mc_rtc::log::info("[RosImuSensor] Body sensor {} found in the robot", bodySensor_name_);
  }
  
}

void RosImuSensor::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("RosImuSensor::reset called");
}

void RosImuSensor::before(mc_control::MCGlobalController & controller)
{

  //imuBodySensor = mc_rbdyn::BodySensor(bodySensor_name_, referenceFrame, sva::PTransformd::Identity());
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);//.controller();
  auto & robot = ctl.robot();
  ctl.setSensorLinearAccelerations({{bodySensor_name_, imu_sub_.data().value().linear()}});
  ctl.setSensorAngularVelocities({{bodySensor_name_, imu_sub_.data().value().angular()}});
  
  mc_rtc::log::info("[RosImuSensor][ROS] Linear acceleration: {} | Angular velocity: {}",
    robot.bodySensor(bodySensor_name_).linearAcceleration().transpose(),
    robot.bodySensor(bodySensor_name_).angularVelocity().transpose()
  );
}

void RosImuSensor::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("RosImuSensor::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration RosImuSensor::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true; // Not sure about this
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
