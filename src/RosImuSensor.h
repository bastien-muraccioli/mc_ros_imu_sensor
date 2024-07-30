/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>

#include "utils/ROSSubscriber.h"

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

  void rosSpinner(void);

private:

  bool verbose; // Verbose flag
  
  // IMU Sensor ROS
  std::string referenceFrame; // Reference frame for the IMU sensor
  bool ros_imu_sensor_;       // Flag to enable/disable the IMU sensor
  std::shared_ptr<ros::NodeHandle> nh_; // ROS node handle
  std::thread spinThread_;              // Thread to spin the ROS node
  double maxTime_ = 0.001;              // Maximum time for the IMU sensor
  double freq_ = 1000;                 // Frequency of the IMU sensor
  std::string imu_sensor_topic_ = "/bus0/ft_sensor0/ft_sensor_readings/imu"; // IMU sensor topic
  ROSImuSubscriber imu_sub_; // ROS IMU subscriber
};

} // namespace mc_plugin
