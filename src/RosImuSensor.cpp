#include "RosImuSensor.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

RosImuSensor::~RosImuSensor() = default;

void RosImuSensor::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("RosImuSensor::init called with configuration:\n{}", config.dump(true, true));
}

void RosImuSensor::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("RosImuSensor::reset called");
}

void RosImuSensor::before(mc_control::MCGlobalController &)
{
  mc_rtc::log::info("RosImuSensor::before");
}

void RosImuSensor::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("RosImuSensor::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration RosImuSensor::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("RosImuSensor", mc_plugin::RosImuSensor)
