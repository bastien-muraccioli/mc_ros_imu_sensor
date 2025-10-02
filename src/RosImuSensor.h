/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <mc_rbdyn/BodySensor.h>
#ifdef MC_RTC_ROS_IS_ROS2
  #include "utils/ROS2Subscriber.h"
  #pragma message("Using ROS2Subscriber.h")
#else
  #include "utils/ROSSubscriber.h"
  #pragma message("Using ROSSubscriber.h")
#endif

namespace mc_plugin
{

struct RosImuSensor : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~RosImuSensor() override;

private:
  
  // IMU Sensor ROS
  bool ros_imu_sensor_;       // Flag to enable/disable the IMU sensor
  std::shared_ptr<rclcpp::Node> node; 
  std::thread spinThread_;              // Thread to spin the ROS node
  std::mutex mutex_;                    // Mutex to lock the IMU sensor data
  ROSImuSubscriber imu_sub_; // ROS IMU subscriber
  void rosSpinner(void);     // ROS spinner function

  // yaml config
  bool verbose; // Verbose flag
  std::string referenceFrame; // Reference frame for the IMU sensor
  double freq_;                 // Frequency of the IMU sensor
  std::string imu_sensor_topic_; // IMU sensor topic
  std::string bodySensor_name_; // Name of the body sensor
  Eigen::Vector3d linear_acceleration_; // Linear acceleration
  Eigen::Vector3d angular_velocity_; // Angular velocity

  double maxTime_;              // Maximum time for the IMU sensor
  // mc_rbdyn::BodySensor & imuBodySensor; // Body sensor for the IMU sensor
  
};

} // namespace mc_plugin
