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

#include "orcp2/orcp2.h"
#include "orcp2/serial.h"
#include "orcp2/times.h"
#include "orcp2/imu.h"
#include "orcp2/robot_4wd.h"

#include <string>
#include <sstream>
#include <boost/thread.hpp>

#define DEFAULT_DRIVE_SERIAL "/dev/ttyS0"
#define DEFAULT_IMU_SERIAL "/dev/ttyS1"
#define DEFAULT_SERIAL_RATE 57600

ros::Publisher drive_telemetry_pub;
ros::Publisher sensors_telemetry_pub;
ros::Publisher imu_raw_data_pub;

std::string drive_serial_name = DEFAULT_DRIVE_SERIAL;
std::string imu_serial_name = DEFAULT_IMU_SERIAL;
int serial_rate = DEFAULT_SERIAL_RATE;

Serial drive_serial;
Serial sensors_serial;

IMU3_data raw_imu_data;
Robot_4WD robot_data;

void spinThread()
{
  ros::Rate loop_rate(50);

  int count = 0;
  while (ros::ok())
  {
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
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_4wd_node");

  ros::NodeHandle n;

  drive_telemetry_pub = n.advertise<ros_4wd_driver::drive_telemetry_4wd>("drive_telemetry_4wd", 1000);
  sensors_telemetry_pub = n.advertise<ros_4wd_driver::sensors_telemetry_4wd>("sensors_telemetry_4wd", 1000);
  imu_raw_data_pub = n.advertise<ros_4wd_driver::imu_raw_data>("imu_raw_data", 1000);

  //boost::thread spin_thread(&spinThread);
  
  ROS_INFO("Start");
  
  ROS_INFO("Load params");
  if (n.hasParam("/robot_4wd_node/drive_port")) {
ROS_INFO("Load param: drive_port");  
		n.getParam("/robot_4wd_node/drive_port", drive_serial_name);
	} 
	  if (n.hasParam("/robot_4wd_node/sensor_port")) { 
ROS_INFO("Load param: sensor_port");  	  
		n.getParam("/robot_4wd_node/sensor_port", imu_serial_name);
	} 
	  if (n.hasParam("/robot_4wd_node/baud")) { 
ROS_INFO("Load param: baud"); 	  
		n.getParam("/robot_4wd_node/baud", serial_rate);
	} 

  ROS_INFO("drive_port: %s", drive_serial_name.c_str());
  ROS_INFO("sensor_port: %s", imu_serial_name.c_str());
  ROS_INFO("baud: %d", serial_rate);
  
  ROS_INFO("Open ports");
  if( drive_serial.open(drive_serial_name.c_str(), serial_rate) ) {
		ROS_ERROR("Cant open port: %s:%d", drive_serial_name.c_str(), serial_rate);
		return -1;
	}
	
#if 1
	int res = 0;
	TBuff<uint8_t> buff;
	orcp2::packet pkt;
	
	buff.resize(2048);
	pkt.message.resize(256);

	if(!buff.data || !pkt.message.data) {
		ROS_ERROR("[!] Error: cant allocate memory!\n");
		drive_serial.close();
		return -1;
	}

	#if 1
	orcp2::ORCP2 orcp(drive_serial);

	for(int i=0; i<7; i++) {
		int val = i%2;
		printf("%d\n", val);
		orcp.digitalWrite(13, val);
		orv::time::sleep(500);
	}
	#endif
#endif	
  
  while (ros::ok()) {
	ros::spinOnce();
  }
  
  drive_serial.close();
  sensors_serial.close();
  ROS_INFO("End");
  
  //spin_thread.join();
#if 0
  ros::Rate loop_rate(50);

  int count = 0;
  while (ros::ok())
  {
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
#endif
  


  return 0;
}

