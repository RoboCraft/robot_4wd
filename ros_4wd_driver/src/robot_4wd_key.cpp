//
// ROS node for robot 4wd keyboard teleoperation
//
//
// robocraft.ru
//

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"			// cmd_vel

#include "orcp2/console.h"

#define KEYCODE_ESC 0x1B
#define KEYCODE_SPACE 0x20
#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

double linear=0, angular=0;
ros::Time first_publish;
ros::Time last_publish;
double l_scale=1, a_scale=1;
ros::Publisher vel_pub;

void publish(double angular_, double linear_)
{
    geometry_msgs::Twist vel;
    vel.angular.z = a_scale*angular_;
    vel.linear.x = l_scale*linear_;

    vel_pub.publish(vel);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_4wd_teleop");

    ros::NodeHandle n;

    vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ROS_INFO("Start");

    ROS_INFO("Load params");
    ros::NodeHandle ph("~");
    ph.param("scale_angular", a_scale, a_scale);
    ph.param("scale_linear", l_scale, l_scale);

    ROS_INFO("scale_angular: %0.2f", a_scale);
    ROS_INFO("scale_linear: %0.2f", l_scale);

#if defined(LINUX)
    // Use termios to turn off line buffering
    termios term;
    tcgetattr(STDIN_FILENO, &term);
    term.c_lflag &= ~ICANON;
    tcsetattr(STDIN_FILENO, TCSANOW, &term);
    setbuf(stdin, NULL);
#endif

    printf("Reading from keyboard\n");
    printf("---------------------------\n");
    printf("Use WASD or arrow keys to move the robott.\n");

    while (ros::ok()) {
        int key = console::waitKey(30);
        if(key != 0 ) ROS_DEBUG( "[i] Key: %c (%d)\n", key ,key );
        if(key == KEYCODE_ESC) { //ESC
            linear = angular = 0;
            publish(linear, angular);
            break;
        }
        else if(key == KEYCODE_SPACE) { // SPACE
            linear = angular = 0;
            ROS_DEBUG("[i] stop\n");
            publish(linear, angular);
        }
        else if(key == 'w' || key == 'W' || key == KEYCODE_U) {
            linear = 1.0;
            angular = 0;
            ROS_DEBUG("[i] forward %.2f %.2f", linear, angular);
            publish(linear, angular);
        }
        else if(key == 's' || key == 'S' || key == KEYCODE_D) {
            linear = -1.0;
            angular = 0;
            ROS_DEBUG("[i] backward %.2f %.2f", linear, angular);
            publish(linear, angular);
        }
        else if(key == 'a' || key == 'A' || key == KEYCODE_L) {
            linear = 0;
            angular = 1.0;
            ROS_DEBUG("[i] left %.2f %.2f", linear, angular);

            publish(linear, angular);
        }
        else if(key == 'd' || key == 'D' || key == KEYCODE_R) {
            linear = 0;
            angular = -1.0;
            ROS_DEBUG("[i] right %.2f %.2f", linear, angular);

            publish(linear, angular);
        }
    }

    ROS_INFO("End");

    return 0;
}

