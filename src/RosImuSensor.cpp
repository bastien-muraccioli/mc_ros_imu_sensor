#include "RosImuSensor.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_rbdyn/BodySensor.h>
#include <mc_rtc/gui/ArrayLabel.h>

namespace mc_plugin
{

RosImuSensor::~RosImuSensor() = default;

void RosImuSensor::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("RosImuSensor::init called with configuration:\n{}", config.dump(true, true));
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);

  if(!ctl.controller().datastore().has("ros_spin"))
  {
     ctl.controller().datastore().make<bool>("ros_spin", false);
  }
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
    node = mc_rtc::ROSBridge::get_node_handle();
    if(!ctl.controller().datastore().get<bool>("ros_spin"))
    {
      spinThread_ = std::thread(std::bind(&RosImuSensor::rosSpinner, this));
      ctl.controller().datastore().assign("ros_spin", true);
    }
    mc_rtc::log::info("[RosImuSensor][ROS] Subscribing to {}", imu_sensor_topic_);

    imu_sub_.subscribe(node, imu_sensor_topic_);
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

  linear_acceleration_ = imu_sub_.data().value().linear();
  angular_velocity_ = imu_sub_.data().value().angular();

  ctl.controller().gui()->addElement({"Plugins", "IMU"},
                                    mc_rtc::gui::ArrayLabel(
                                        "EndEffector",{"ax", "ay", "az", "ωx", "ωy", "ωz"},[this, &controller]()
                                        {
                                          auto linearAcceleration = controller.controller().robot().bodySensor(bodySensor_name_).linearAcceleration().transpose();
                                          auto angularVelocity = controller.controller().robot().bodySensor(bodySensor_name_).angularVelocity().transpose();
                                          return std::vector<double>{linearAcceleration.x(), linearAcceleration.y(), linearAcceleration.z(), angularVelocity.x(), angularVelocity.y(), angularVelocity.z()};
                                        }));

  // Log
  ctl.controller().logger().addLogEntry("RosImuSensor_LinearAcceleration", [this]() { return linear_acceleration_; });
  ctl.controller().logger().addLogEntry("RosImuSensor_AngularVelocity", [this]() { return angular_velocity_; });
  
}

void RosImuSensor::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("RosImuSensor::reset called");
}

void RosImuSensor::before(mc_control::MCGlobalController & controller)
{

  //imuBodySensor = mc_rbdyn::BodySensor(bodySensor_name_, referenceFrame, sva::PTransformd::Identity());
  auto & ctl = static_cast<mc_control::MCGlobalController &>(controller);
  auto & robot = ctl.robot();
  linear_acceleration_ = imu_sub_.data().value().linear();
  angular_velocity_ = imu_sub_.data().value().angular();
  ctl.setSensorLinearAccelerations({{bodySensor_name_, linear_acceleration_}});
  ctl.setSensorAngularVelocities({{bodySensor_name_, angular_velocity_}});
  
  // mc_rtc::log::info("[RosImuSensor][ROS] Linear acceleration: {} | Angular velocity: {}",
  //   robot.bodySensor(bodySensor_name_).linearAcceleration().transpose(),
  //   robot.bodySensor(bodySensor_name_).angularVelocity().transpose()
  // );
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
  #ifdef MC_RTC_ROS_IS_ROS2
    rclcpp::Rate r(freq_);
    while(rclcpp::ok())
    {
      rclcpp::spin_some(node);
      r.sleep();
    }
  #else
    ros::Rate r(freq_);
    while(ros::ok())
    {
      ros::spinOnce();
      r.sleep();
    }
  #endif
  mc_rtc::log::info("[RosImuSensor][ROS Spinner] spinner destroyed");
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("RosImuSensor", mc_plugin::RosImuSensor)
