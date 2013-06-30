// 
// ROS node for robot 4wd
//
//
// robocraft.ru
//

#include "ros/ros.h"

#include "nav_msgs/Odometry.h"				// odom
#include "geometry_msgs/Twist.h"			// cmd_vel

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

ros::Publisher odom_pub;
ros::Subscriber cmd_vel_sub;

std::string drive_serial_name = DEFAULT_DRIVE_SERIAL;
std::string imu_serial_name = DEFAULT_IMU_SERIAL;
int serial_rate = DEFAULT_SERIAL_RATE;

Serial drive_serial;
Serial sensors_serial;

boost::mutex drive_serial_write;
boost::mutex sensors_serial_write;

IMU3_data raw_imu_data;
Robot_4WD robot_data;

volatile bool global_stop = false;

void drive_thread()
{
    int res = 0;
    TBuff<uint8_t> buff;
    orcp2::packet pkt;

    buff.resize(2048);
    pkt.message.resize(256);

    if(!buff.data || !pkt.message.data) {
        ROS_ERROR("[!] Error: cant allocate memory!\n");
        return ;
    }

    orcp2::ORCP2 orcp(drive_serial);

    int val=0;
    int counter=0;
    std::stringstream ss;

    while (!global_stop) {
        ros_4wd_driver::drive_telemetry_4wd drive;

        if( res = drive_serial.waitInput(500) ) {
            //printf("[i] waitInput: %d\n", res);
            if( (res = drive_serial.available()) > 0 ) {
                //printf("[i] available: %d\n", res);
                //res = res > (buff.real_size-buff.size) ? (buff.real_size-buff.size) : res ;
                if( (res = drive_serial.read(buff.data+buff.size, buff.real_size-buff.size)) > 0 ) {
                    buff.size += res;

                    // get packet from data
                    while ( (res = orcp2::get_packet(buff.data, buff.size, &pkt)) > 0) {
                   //     ROS_INFO("[i] res: %02d message: %04X size: %04d CRC: %02X buff.size: %02d\n",
                   //              res, pkt.message_type, pkt.message_size, pkt.checksum, buff.size);

                        uint8_t crc = pkt.calc_checksum();
                        if(crc == pkt.checksum) {
                   //         ROS_INFO("[i] Good message CRC: %02X\n", pkt.checksum);

                            switch(pkt.message_type) {
                            case ORCP2_SEND_STRING:
                                pkt.message.data[pkt.message.size]=0;
                                ROS_INFO("[i] String: %s\n", pkt.message.data);
                                //printf("[i] counter: %d\n", counter);
#if 0
                                if(++counter > 50) {
                                    //printf("[i] digitalWrite: %d\n", val);
                                    val = !val;
                                    orcp.digitalWrite(13, val);
                                    counter = 0;
                                }
#endif
                                break;
                            case ORCP2_MESSAGE_IMU_RAW_DATA:
                                deserialize_imu3_data(pkt.message.data, pkt.message.size, &raw_imu_data);
                                ROS_INFO( "[i] IMU: acc: [%d %d %d] mag: [%d %d %d] gyro: [%d %d %d]\n",
                                          raw_imu_data.Accelerometer[0], raw_imu_data.Accelerometer[1], raw_imu_data.Accelerometer[2],
                                          raw_imu_data.Magnetometer[0], raw_imu_data.Magnetometer[1], raw_imu_data.Magnetometer[2],
                                          raw_imu_data.Gyro[0], raw_imu_data.Gyro[1], raw_imu_data.Gyro[2] );
                                break;
                            case ORCP2_MESSAGE_ROBOT_4WD_DRIVE_TELEMETRY:
                                deserialize_robot_4wd_drive_part(pkt.message.data, pkt.message.size, &robot_data);
                                ROS_INFO( "[i] Drive telemetry: bmp: %d enc: [%d %d %d %d] PWM: [%d %d %d %d]\n",
                                          robot_data.Bamper,
                                          robot_data.Encoder[0], robot_data.Encoder[1], robot_data.Encoder[2], robot_data.Encoder[3],
                                          robot_data.PWM[0], robot_data.PWM[1], robot_data.PWM[2], robot_data.PWM[3] );

                                drive.bamper = robot_data.Bamper;
                                drive.encoder1 = robot_data.Encoder[0];
                                drive.encoder2 = robot_data.Encoder[1];
                                drive.encoder3 = robot_data.Encoder[2];
                                drive.encoder4 = robot_data.Encoder[3];
                                drive.pwm1 = robot_data.PWM[0];
                                drive.pwm2 = robot_data.PWM[1];
                                drive.pwm3 = robot_data.PWM[2];
                                drive.pwm4 = robot_data.PWM[3];

                        //        ss << "[i][DRIVE][" << counter << "] bamper "<<drive.bamper
                        //           << "encoders " << drive.encoder1 << " " << drive.encoder2 << " " << drive.encoder3 << " " << drive.encoder4
                        //           << "pwm " << drive.pwm1 << " " << drive.pwm2 << " " << drive.pwm3 << " " << drive.pwm4;

                        //        ROS_INFO("%s", ss.str().c_str());

                                drive_telemetry_pub.publish(drive);
                                break;
                            case ORCP2_MESSAGE_ROBOT_4WD_SENSORS_TELEMETRY:
                                deserialize_robot_4wd_sensors_part(pkt.message.data, pkt.message.size, &robot_data);
                                ROS_INFO( "[i] Sensors telemetry: US: %d IR: [%d %d %d %d] V: %d\n",
                                          robot_data.US[0],
                                          robot_data.IR[0], robot_data.IR[1], robot_data.IR[2], robot_data.IR[3],
                                          robot_data.Voltage );
                                break;
                            case ORCP2_MESSAGE_ROBOT_4WD_TELEMETRY:
                                deserialize_robot_4wd(pkt.message.data, pkt.message.size, &robot_data);
                                ROS_INFO( "[i] Drive Telemetry: bmp: %d enc: [%d %d %d %d] PWM: [%d %d %d %d]\n",
                                          robot_data.Bamper,
                                          robot_data.Encoder[0], robot_data.Encoder[1], robot_data.Encoder[2], robot_data.Encoder[3],
                                          robot_data.PWM[0], robot_data.PWM[1], robot_data.PWM[2], robot_data.PWM[3] );
                                ROS_INFO( "[i] Sensors Telemetry: US: %d IR: [%d %d %d %d] V: %d\n",
                                          robot_data.US[0],
                                          robot_data.IR[0], robot_data.IR[1], robot_data.IR[2], robot_data.IR[3],
                                          robot_data.Voltage );
                                break;
                            default:
                                ROS_INFO("[i] Unknown message type: %04X!\n", pkt.message_type);
                                break;
                            }
                        }
                        else {
                            ROS_WARN("[!] Bad message CRC: %02X vs: %02X\n", crc, pkt.checksum);
                        }
                    }
                }
                else {
                    ROS_WARN("[!] too much data: %d (%d)\n", res, buff.size);
                    buff.size = 0;
                }
            }
            else {
                ROS_INFO("[!] no available: %d\n", res);
            }
        }
        else {
            ROS_INFO("[!] waitInput timeout: %d\n", res);
        }
    } //while (!global_stop) {
}

void cmd_vel_received(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    //roomba->drive(cmd_vel->linear.x, cmd_vel->angular.z);
    double linear_speed = cmd_vel->linear.x;
    double angular_speed = cmd_vel->angular.z;
    ROS_INFO("Velocity received: %.2f %.2f", linear_speed, angular_speed);

    //int left_speed_mm_s = (int)((linear_speed-ROOMBA_AXLE_LENGTH*angular_speed/2)*1e3);		// Left wheel velocity in mm/s
    //int right_speed_mm_s = (int)((linear_speed+ROOMBA_AXLE_LENGTH*angular_speed/2)*1e3);	// Right wheel velocity in mm/s
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_4wd_node");

    ros::NodeHandle n;

    drive_telemetry_pub = n.advertise<ros_4wd_driver::drive_telemetry_4wd>("drive_telemetry_4wd", 50);
    sensors_telemetry_pub = n.advertise<ros_4wd_driver::sensors_telemetry_4wd>("sensors_telemetry_4wd", 50);
    imu_raw_data_pub = n.advertise<ros_4wd_driver::imu_raw_data>("imu_raw_data", 50);

    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    cmd_vel_sub  = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, cmd_vel_received);

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

#if 0
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

    boost::thread drv_thread(&drive_thread);

    while (ros::ok()) {
        ros::spin();
    }

    global_stop = true;

    drv_thread.join();

    drive_serial.close();
    sensors_serial.close();
    ROS_INFO("End");

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

