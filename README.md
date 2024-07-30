mc_ros_imu_sensor plugin
==

This project is a mc_rtc plugin, initially made to retrieve IMU data from Bota SensONE F/T sensor ROS topic.

Can be reused to retrieve any ROS IMU message with angular velocity and linear acceleration.

Quick start
--

1. Build and install the project.

	Note: If you are using [mc-rtc-superbuild](https://github.com/mc-rtc/mc-rtc-superbuild), create `mc_ros_imu_sensor.cmake`, inside mc-rtc-superbuild/extensions/plugins/.

	With the following content:    
	
	```cmake
	AddProject( mc_ros_imu_sensor
	  GITHUB bastien-muraccioli/mc_ros_imu_sensor
	  GIT_TAG origin/main
	  DEPENDS mc_rtc
	)
	```
	
	  Then, you need to add it in the extensions/local.cmake and build the superbuild.

3. Create `RosImuSensor.yaml` plugin config file, inside ~/.config/mc_rtc/plugins/.
   
    Precise in it, the reference frame of the IMU used in your xacro/urdf, and the name of the ROS topic.

   E.g.
    ```yaml
	reference_frame: FT_sensor_imu
	ros_imu_sensor: true
	ros_topic_sensor: /bus0/ft_sensor0/ft_sensor_readings/imu
    ``` 
4. Run using your [mc_rtc] interface of choice, add `RosImuSensor` to the `Plugins` configuration entry or enable the autoload option

[mc_rtc]: https://jrl-umi3218.github.io/mc_rtc/
