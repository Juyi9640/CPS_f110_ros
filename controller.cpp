// this node subscribes to /scan topic, extract information and calculate 
// the steering angle
#include <iostream>
#include <stdlib.h>
#include <algorithm>
#include <map>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32MultiArray.h>
#include <ackermann_msgs/AckermannDriveStamped.h>


const float  PI_F = 3.141593;

// declear publisher as global variable
ros::Publisher steer_control;

// find the max value in map
std::pair<int, float> get_max(const std::map<int, float>& x){
    using pairtype= std::pair<int,float>;
    return *std::max_element(x.begin(), x.end(), [] (const pairtype & p1, const pairtype & p2) {
        return p1.second < p2.second;
    });
}
// find the min value in map
std::pair<int, float> get_min(const std::map<int, float>& x){
    using pairtype= std::pair<int,float>;
    return *std::min_element(x.begin(), x.end(), [] (const pairtype & p1, const pairtype & p2) {
        return p1.second < p2.second;
    });
}

// mapping from (0, 1080) to (-pi, pi)
float get_angle(int key_of_max_distance){
    float percent = (float)key_of_max_distance/1080;
    // 0.096 = 0.3/pi, steering limit
    return 0.096*(percent*PI_F*2 - PI_F);
}

// mapping from (135, 945) to (-3pi/4, 3pi/4)
float get_angle1(int key_of_max_distance){
    float percent = (float)key_of_max_distance/810;
    // 0.127 = 0.3/(3pi/4), steering limit
    return 0.127*0.75*(percent*PI_F*2 - PI_F);
}

// mapping from (270, 810) to (-2pi/4, 2pi/4)
float get_angle2(int key_of_max_distance){
    float percent = (float)key_of_max_distance/540;
    // 0.191 = 0.3/(pi/2), because the max value of
    // 0.5*(percent*PI_F*2 - PI_F) is pi/2
    // therefore the max steering angle is limited to 0.3
    return 0.191*0.5*(percent*PI_F*2 - PI_F);
}

// This function calculates the angle between front direction and wall
float front_wall_angle3(int key_of_min_distance){
    // 540 is head direction
    // if key > 540, the car nears left wall
    // if key < 540, the car nears right wall
    float front_wall_angle = 0;
    float front_wall_angle_radian = 0;
    if (key_of_min_distance >= 540){
        front_wall_angle = (float)(key_of_min_distance - 540)/1080;
        // if this angle < 90 degree, means car is heading left wall, must steer
        // if this angle > 90 degree, safe, do nothing
        if (front_wall_angle < 0.25){
            // right angle between min_distance and wall
            front_wall_angle_radian = PI_F/2 - front_wall_angle*2*PI_F;
        }
    } else {
        front_wall_angle = (float)(540 - key_of_min_distance)/1080;
        // if this angle < 90 degree, means car is heading right wall, must steer
        // if this angle > 90 degree, safe, do nothing    
        if (front_wall_angle < 0.25){
            front_wall_angle_radian = - (PI_F/2 - front_wall_angle*2*PI_F);
        }
    }
    // scale the angle to 0.3
    //printf("steer: %f\n", front_wall_angle_radian);
    return (float)0.3*front_wall_angle_radian/(PI_F/4);

}

// mapping from (405, 675) to (-pi/4, pi/4)
float get_angle3(int key_of_max_distance){
    float percent = (float)key_of_max_distance/270;
    // 0.382 = 0.3/(pi/4) steering limit
    return 0.382*0.25*(percent*PI_F*2 - PI_F);
}

void callback(const sensor_msgs::LaserScan& laserscan){ 
    // ConstPtr didn't used, but reference
    //printf("min_angle: %f", laserscan.angle_min);
    std::map<int, float> scan, scan_1, scan_2, scan_3;
    // Notice: from i=0 to i=1079 there are 1080 pairs generated
    // can't set it from 1 to 1080 because then get_min malfunction
    for (int i = 0; i <= 1079; i++){
        scan[i]=laserscan.ranges[i];
    }
    auto max_distance = get_max(scan);
    auto min_distance = get_min(scan);
    //printf("The key of the smallest distance: %d", min_distance.first);    
    //printf("The value of the largest distance: %f ", max_distance.second);
    

    // limit the lidar scan area to (-3pi/4, 3pi/4) (135,945)
    for (int j = 1; j <= 810; j++){
        scan_1[j] = laserscan.ranges[j+135];
    }
    auto max_distance_1 = get_max(scan_1);

    // limit the lidar scan area to (-2pi/4, 2pi/4) (270,810)
    for (int k = 1; k <= 540; k++){
        scan_2[k] = laserscan.ranges[k+270];
    }
    auto max_distance_2 = get_max(scan_2);

    // limit the lidar scan area to (-pi/4, pi/4) (405,675)
    for (int l = 1; l <= 270; l++){
        scan_3[l] = laserscan.ranges[l+405];
    }
    auto max_distance_3 = get_max(scan_3);


    float max_distance_angle = get_angle3(max_distance_3.first);
    //printf("max_distance_angle: %f\n", max_distance_angle);


    // the left, right, front and rear distance between car and wall
    float left_dis = scan[810];
    float front_dis = scan[540];
    //printf("Front distance: %f\n", front_dis);
    float right_dis = scan[270];
    float rear_dis = scan[1079];

    // When left distance is small, turn right
    // when right distance is small, turn left
    float steer_to_left = 0;
    float steer_to_right =0;
    if (left_dis < 1.0){
        steer_to_right = 0.3;
    }else if (right_dis < 1.0){
        steer_to_left = 0.3;
    }

    // boost the speed up if the car locates at long straight runway
    float boost_speed = 0;
    if (front_dis + rear_dis >= 13){
        boost_speed = 1;
    }

    
    
    // Controll
    // Adding a minus cause steer left is pos and steer right is negative
    // float steer_angle = -(0.0 - max_distance_angle + steer_to_right - steer_to_left);
    float steer_angle = -(0.0 - max_distance_angle + steer_to_right - steer_to_left + front_wall_angle3(min_distance.first));
    //float speed = 4.6;
    float speed = 5.6 + boost_speed;

   

    // Make and publish message
    // Header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    // Ackermann
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = speed;
    drive_msg.steering_angle = steer_angle;
    // AckermannStamped
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;
    // publish AckermannDriveStamped message to drive topic
    //steer_control.publish(drive_st_msg);


}

int main(int argc, char **argv){
    ros::init(argc, argv, "controller");
    ros::NodeHandle node;

    ros::Subscriber sub_to_scan = node.subscribe("/scan", 1, callback);
    steer_control = node.advertise<ackermann_msgs::AckermannDriveStamped>("con_msg_stream",1);


    ros::spin();
    return 0;
}
