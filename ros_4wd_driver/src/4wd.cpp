// 
// ROS node for robot 4wd
//
//
// robocraft.ru
//

#include "ros/ros.h"
#include "ros_4wd_driver/drive_telemetry_4wd.h"
#include "ros_4wd_driver/sensors_telemetry_4wd.h"
#include "ros_4wd_driver/imu_raw_data.h"
#include "ros_4wd_driver/orcp_string.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_4wd_node");

  ros::NodeHandle n;

  ros::Publisher drive_telemetry_pub = n.advertise<ros_4wd_driver::drive_telemetry_4wd>("drive_telemetry_4wd", 1000);
  ros::Publisher sensors_telemetry_pub = n.advertise<ros_4wd_driver::sensors_telemetry_4wd>("sensors_telemetry_4wd", 1000);
  ros::Publisher imu_raw_data_pub = n.advertise<ros_4wd_driver::imu_raw_data>("imu_raw_data", 1000);

  ros::Rate loop_rate(50);

  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    ros_4wd_driver::drive_telemetry_4wd drive;
	ros_4wd_driver::sensors_telemetry_4wd sensors;
	ros_4wd_driver::imu_raw_data imuraw;

    std::stringstream ss;
    ss << "[i][DRIVE][" << count << "] bamper "<<drive.bamper
	<< "encoders " << drive.encoder1 << " " << drive.encoder2 << " " << drive.encoder3 << " " << drive.encoder4 
	<< "pwm " << drive.pwm1 << " " << drive.pwm2 << " " << drive.pwm3 << " " << drive.pwm4;

    ROS_INFO("%s", ss.str().c_str());

    drive_telemetry_pub.publish(drive);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

