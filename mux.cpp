#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/Char.h"
#include <ackermann_msgs/AckermannDriveStamped.h>

float desired_speed1 = 0;
float desired_speed2 = 0;

float desired_steer_ang1 = 0;
float desired_steer_ang2 = 0;

std_msgs::Header header1;
std_msgs::Header header2;
ros::Publisher mux_drive;

char mode_msg;
char aeb_msg;

void callback1(const std_msgs::Char &msg)
{
    // Notice, use %c to print char, use %s to print string
    // ROS_INFO("Recv: %c", msg.data);

    // callback1 will be called everytime the keyboard was typed.
    // Since keyboard works discontinuously
    // callback1 won't be activated all the time
    mode_msg = msg.data;

}

void callback2(const ackermann_msgs::AckermannDriveStamped &msg)
{
    std_msgs::Header header;
    ackermann_msgs::AckermannDrive drive_msg;
    ackermann_msgs::AckermannDriveStamped drive_st_msg;

    header1 = msg.header;
    desired_speed1 = msg.drive.speed;
    desired_steer_ang1 = msg.drive.steering_angle;

    header.stamp = header1.stamp;
    drive_msg.speed = desired_speed1;
    drive_msg.steering_angle = desired_steer_ang1;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;

    // if 'k' and if AEB is not 's', keyboard drives
    if (mode_msg == 'k' && aeb_msg != 's')
    {
        mux_drive.publish(drive_st_msg);
    }
    // if 'f' and AEB wants to stop the car,
    // keyboard drives.
    if (mode_msg == 'f' && aeb_msg == 's')
    {
        mux_drive.publish(drive_st_msg);
    }
}
void callback3(const ackermann_msgs::AckermannDriveStamped &msg)
{
    std_msgs::Header header;
    ackermann_msgs::AckermannDrive drive_msg;
    ackermann_msgs::AckermannDriveStamped drive_st_msg;

    header2 = msg.header;
    desired_speed2 = msg.drive.speed;
    desired_steer_ang2 = msg.drive.steering_angle;

    header.stamp = header2.stamp;
    drive_msg.speed = desired_speed2;
    drive_msg.steering_angle = desired_steer_ang2;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;

    // if 'f' and AEB is not 's', controller drives
    if (mode_msg == 'f' && aeb_msg != 's')
    {
        mux_drive.publish(drive_st_msg);
    }

}

void callback4(const std_msgs::Char &msg){
    aeb_msg = msg.data;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "subscriber");
    ros::NodeHandle node;

    ros::Subscriber key_mode = node.subscribe("mode", 1, callback1);

    ros::Subscriber key_drive = node.subscribe("key_msg_stream", 1, callback2);
    ros::Subscriber con_drive = node.subscribe("con_msg_stream", 1, callback3);

    ros::Subscriber aeb_sub = node.subscribe("AEB", 1, callback4);

    mux_drive = node.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);

    ros::spin();
    return 0;
}
