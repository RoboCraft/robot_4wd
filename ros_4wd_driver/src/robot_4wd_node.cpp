// 
// ROS node for robot 4wd
//
//
// robocraft.ru
//

#include "ros/ros.h"

#include "geometry_msgs/Quaternion.h"
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

#define _USE_MATH_DEFINES
#include <math.h>

#include <string>
#include <limits>
#include <boost/thread.hpp>

#define DEFAULT_DRIVE_SERIAL "/dev/ttyS0"
#define DEFAULT_SENSORS_SERIAL "/dev/ttyS1"
#define DEFAULT_SERIAL_RATE 57600

ros::Publisher drive_telemetry_pub;
ros::Publisher sensors_telemetry_pub;
ros::Publisher imu_raw_data_pub;

ros::Publisher odom_pub;
ros::Subscriber cmd_vel_sub;

std::string drive_serial_name = DEFAULT_DRIVE_SERIAL;
std::string sensors_serial_name = DEFAULT_SENSORS_SERIAL;
int serial_rate = DEFAULT_SERIAL_RATE;

Serial drive_serial;
Serial sensors_serial;

boost::mutex drive_serial_write;
boost::mutex sensors_serial_write;

IMU3_data raw_imu_data;
Robot_4WD robot_data;

double wheel_diameter = 0.0685;
double wheel_track = 0.2152;
double gear_reduction = 75;
double encoder_resolution = 5;
double ticks_per_meter;
double speed_koef = 10;

volatile bool global_stop = false;

ros::Time odometry_prev_time;
int enc_right=0;
int enc_left=0;
double odom_x=0, odom_y=0, odom_th=0;

//$TODO
void calc_odometry(ros_4wd_driver::drive_telemetry_4wd &drive)
{
    ros::Time current_time = ros::Time::now();

    ros::Duration dur = current_time - odometry_prev_time;
    double dt = dur.toNSec()*1e9;

    ROS_INFO("[i] dt: %0.6f s\n", dt);

    double dright = (drive.encoder1 - enc_right) / ticks_per_meter;
    double dleft = (drive.encoder2 - enc_left) / ticks_per_meter;

    enc_right = drive.encoder1;
    enc_left = drive.encoder2;

    double dxy_ave = (dright + dleft) / 2.0;
    double dth = (dright - dleft) / wheel_track;
    double vxy = dxy_ave / dt;
    double vth = dth / dt;

    double dx=0, dy=0;

    //dxy_ave != 0
    if( dxy_ave >  std::numeric_limits<double>::epsilon() &&
            dxy_ave < -std::numeric_limits<double>::epsilon() ) {

        dx = cos(dth) * dxy_ave;
        dy = -sin(dth) * dxy_ave;
        odom_x += (cos(odom_th) * dx - sin(odom_th) * dy);
        odom_y += (sin(odom_th) * dx + cos(odom_th) * dy);
    }

    //dth != 0
    if( dth >  std::numeric_limits<double>::epsilon() &&
            dth < -std::numeric_limits<double>::epsilon() ) {
        odom_th += dth;
    }

    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(odom_th / 2.0);
    quaternion.w = cos(odom_th / 2.0);

    nav_msgs::Odometry odom;

    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.header.stamp = current_time;
    odom.pose.pose.position.x = odom_x;
    odom.pose.pose.position.y = odom_y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = quaternion;
    odom.twist.twist.linear.x = vxy;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = vth;

    odom_pub.publish(odom);

    odometry_prev_time = current_time;
}

void process_serial(Serial &serial, TBuff<uint8_t> &buff, orcp2::packet &pkt)
{
    int res = 0;

    if(!buff.data || !pkt.message.data) {
        ROS_ERROR("[!] Error: no memory!\n");
        return ;
    }

    ros::Time current_time;

    ros_4wd_driver::drive_telemetry_4wd drive;
    ros_4wd_driver::sensors_telemetry_4wd sensors;
    ros_4wd_driver::imu_raw_data imu;

    if( res = serial.waitInput(500) ) {
        //printf("[i] waitInput: %d\n", res);
        if( (res = serial.available()) > 0 ) {
            //printf("[i] available: %d\n", res);
            //res = res > (buff.real_size-buff.size) ? (buff.real_size-buff.size) : res ;
            if( (res = serial.read(buff.data+buff.size, buff.real_size-buff.size)) > 0 ) {
                buff.size += res;

                // get packet from data
                while ( (res = orcp2::get_packet(buff.data, buff.size, &pkt)) > 0) {
                    //     ROS_INFO("[i] res: %02d message: %04X size: %04d CRC: %02X buff.size: %02d\n",
                    //              res, pkt.message_type, pkt.message_size, pkt.checksum, buff.size);

                    uint8_t crc = pkt.calc_checksum();
                    if(crc == pkt.checksum) {
                        //         ROS_INFO("[i] Good message CRC: %02X\n", pkt.checksum);

                        current_time = ros::Time::now();

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

                            imu.header.stamp = current_time;
                            imu.accelerometer1 = raw_imu_data.Accelerometer[0];
                            imu.accelerometer2 = raw_imu_data.Accelerometer[1];
                            imu.accelerometer3 = raw_imu_data.Accelerometer[2];
                            imu.magnetometer1 = raw_imu_data.Magnetometer[0];
                            imu.magnetometer2 = raw_imu_data.Magnetometer[1];
                            imu.magnetometer3 = raw_imu_data.Magnetometer[2];
                            imu.gyro1 = raw_imu_data.Gyro[0];
                            imu.gyro2 = raw_imu_data.Gyro[1];
                            imu.gyro3 = raw_imu_data.Gyro[2];

                            imu_raw_data_pub.publish(imu);
                            break;
                        case ORCP2_MESSAGE_ROBOT_4WD_DRIVE_TELEMETRY:
                            deserialize_robot_4wd_drive_part(pkt.message.data, pkt.message.size, &robot_data);
                            ROS_INFO( "[i] Drive telemetry: bmp: %d enc: [%d %d %d %d] PWM: [%d %d %d %d]\n",
                                      robot_data.Bamper,
                                      robot_data.Encoder[0], robot_data.Encoder[1], robot_data.Encoder[2], robot_data.Encoder[3],
                                      robot_data.PWM[0], robot_data.PWM[1], robot_data.PWM[2], robot_data.PWM[3] );

                            drive.header.stamp = current_time;
                            drive.bamper = robot_data.Bamper;
                            drive.encoder1 = robot_data.Encoder[0];
                            drive.encoder2 = robot_data.Encoder[1];
                            drive.encoder3 = robot_data.Encoder[2];
                            drive.encoder4 = robot_data.Encoder[3];
                            drive.pwm1 = robot_data.PWM[0];
                            drive.pwm2 = robot_data.PWM[1];
                            drive.pwm3 = robot_data.PWM[2];
                            drive.pwm4 = robot_data.PWM[3];

                            drive_telemetry_pub.publish(drive);
                            calc_odometry(drive);
                            break;
                        case ORCP2_MESSAGE_ROBOT_4WD_SENSORS_TELEMETRY:
                            deserialize_robot_4wd_sensors_part(pkt.message.data, pkt.message.size, &robot_data);
                            ROS_INFO( "[i] Sensors telemetry: US: %d IR: [%d %d %d %d] V: %d\n",
                                      robot_data.US[0],
                                      robot_data.IR[0], robot_data.IR[1], robot_data.IR[2], robot_data.IR[3],
                                      robot_data.Voltage );

                            sensors.header.stamp = current_time;
                            sensors.us = robot_data.US[0];
                            sensors.ir1 = robot_data.IR[0];
                            sensors.ir2 = robot_data.IR[1];
                            sensors.ir3 = robot_data.IR[2];
                            sensors.ir4 = robot_data.IR[3];
                            sensors.voltage = robot_data.Voltage;

                            sensors_telemetry_pub.publish(sensors);
                            break;
                        case ORCP2_MESSAGE_ROBOT_4WD_TELEMETRY:
                            deserialize_robot_4wd(pkt.message.data, pkt.message.size, &robot_data);
                            ROS_INFO( "[i] Drive Telemetry: bmp: %d enc: [%d %d %d %d] PWM: [%d %d %d %d]\n",
                                      robot_data.Bamper,
                                      robot_data.Encoder[0], robot_data.Encoder[1], robot_data.Encoder[2], robot_data.Encoder[3],
                                      robot_data.PWM[0], robot_data.PWM[1], robot_data.PWM[2], robot_data.PWM[3] );

                            drive.header.stamp = current_time;
                            drive.bamper = robot_data.Bamper;
                            drive.encoder1 = robot_data.Encoder[0];
                            drive.encoder2 = robot_data.Encoder[1];
                            drive.encoder3 = robot_data.Encoder[2];
                            drive.encoder4 = robot_data.Encoder[3];
                            drive.pwm1 = robot_data.PWM[0];
                            drive.pwm2 = robot_data.PWM[1];
                            drive.pwm3 = robot_data.PWM[2];
                            drive.pwm4 = robot_data.PWM[3];

                            drive_telemetry_pub.publish(drive);
                            calc_odometry(drive);

                            ROS_INFO( "[i] Sensors Telemetry: US: %d IR: [%d %d %d %d] V: %d\n",
                                      robot_data.US[0],
                                      robot_data.IR[0], robot_data.IR[1], robot_data.IR[2], robot_data.IR[3],
                                      robot_data.Voltage );

                            sensors.header.stamp = current_time;
                            sensors.us = robot_data.US[0];
                            sensors.ir1 = robot_data.IR[0];
                            sensors.ir2 = robot_data.IR[1];
                            sensors.ir3 = robot_data.IR[2];
                            sensors.ir4 = robot_data.IR[3];
                            sensors.voltage = robot_data.Voltage;

                            sensors_telemetry_pub.publish(sensors);
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

}

void drive_thread()
{
    TBuff<uint8_t> buff;
    orcp2::packet pkt;

    buff.resize(2048);
    pkt.message.resize(256);

    if(!buff.data || !pkt.message.data) {
        ROS_ERROR("[!] Error: cant allocate memory!\n");
        return ;
    }

    if(!drive_serial.connected()) {
        ROS_WARN("Port not opened!\n");
        return;
    }

    while (!global_stop) {
        process_serial(drive_serial, buff, pkt);
    }
}

void sensors_thread()
{
    TBuff<uint8_t> buff;
    orcp2::packet pkt;

    buff.resize(2048);
    pkt.message.resize(256);

    if(!buff.data || !pkt.message.data) {
        ROS_ERROR("[!] Error: cant allocate memory!\n");
        return ;
    }

    if(!sensors_serial.connected()) {
        ROS_WARN("Port not opened!\n");
        return;
    }

    while (!global_stop) {
        process_serial(sensors_serial, buff, pkt);
    }
}

void cmd_vel_received(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    //roomba->drive(cmd_vel->linear.x, cmd_vel->angular.z);
    double linear_speed = cmd_vel->linear.x; // m/s
    double angular_speed = cmd_vel->angular.z; // rad/s
    ROS_INFO("Velocity received: %.2f %.2f", linear_speed, angular_speed);

    //int left_speed_mm_s = (int)((linear_speed-ROBOT_AXLE_LENGTH*angular_speed/2)*1e3);		// Left wheel velocity in mm/s
    //int right_speed_mm_s = (int)((linear_speed+ROBOT_AXLE_LENGTH*angular_speed/2)*1e3);	// Right wheel velocity in mm/s

    orcp2::ORCP2 orcp(drive_serial);

    drive_serial_write.lock();

    if( linear_speed <  std::numeric_limits<double>::epsilon() &&
            linear_speed > -std::numeric_limits<double>::epsilon() ) {
        // zero linear speed - turn in place
        uint16_t speed = speed_koef * angular_speed * wheel_track  * gear_reduction / 2.0;
        orcp.drive_4wd(speed, -speed, speed, -speed);
    }
    else if( angular_speed <  std::numeric_limits<double>::epsilon() &&
             angular_speed > -std::numeric_limits<double>::epsilon() ) {
        // zero angular speed - pure forward/backward motion
        orcp.motorsWrite(speed_koef * linear_speed * wheel_track  * gear_reduction / 2.0);
    }
    else {
        // Rotation about a point in space
        //$TODO
        uint16_t left = speed_koef * linear_speed - angular_speed * wheel_track  * gear_reduction / 2.0;
        uint16_t right = speed_koef * linear_speed + angular_speed * wheel_track  * gear_reduction / 2.0;

        orcp.drive_4wd(left, right, left, right);
    }

    drive_serial_write.unlock();
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
        n.getParam("/robot_4wd_node/sensor_port", sensors_serial_name);
    }
    if (n.hasParam("/robot_4wd_node/baud")) {
        ROS_INFO("Load param: baud");
        n.getParam("/robot_4wd_node/baud", serial_rate);
    }
    n.param("/robot_4wd_node/wheel_diameter", wheel_diameter, wheel_diameter);
    n.param("/robot_4wd_node/wheel_track", wheel_track, wheel_track);
    n.param("/robot_4wd_node/gear_reduction", gear_reduction, gear_reduction);
    n.param("/robot_4wd_node/encoder_resolution", encoder_resolution, encoder_resolution);

    ticks_per_meter = encoder_resolution * gear_reduction  / (wheel_diameter * M_PI);

    ROS_INFO("drive_port: %s", drive_serial_name.c_str());
    ROS_INFO("sensor_port: %s", sensors_serial_name.c_str());
    ROS_INFO("baud: %d", serial_rate);
    ROS_INFO("ticks_per_meter: %.2f", ticks_per_meter);

    ROS_INFO("Open ports");
    if( drive_serial.open(drive_serial_name.c_str(), serial_rate) ) {
        ROS_ERROR("Cant open port: %s:%d", drive_serial_name.c_str(), serial_rate);
        return -1;
    }
    if( sensors_serial.open(sensors_serial_name.c_str(), serial_rate) ) {
        ROS_WARN("Cant open port: %s:%d", sensors_serial_name.c_str(), serial_rate);
    }

#if 0
    orcp2::ORCP2 orcp(drive_serial);

    for(int i=0; i<7; i++) {
        int val = i%2;
        printf("%d\n", val);
        orcp.digitalWrite(13, val);
        orv::time::sleep(500);
    }
#endif

    boost::thread drv_thread(&drive_thread);
    boost::thread snsr_thread(&sensors_thread);

    while (ros::ok()) {
        ros::spin();
    }

    global_stop = true;

    drv_thread.join();
    snsr_thread.join();

    drive_serial.close();
    sensors_serial.close();
    ROS_INFO("End");

    return 0;
}

