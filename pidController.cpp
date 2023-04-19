// Kp part of pid
#include <map>
#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <fstream>

const float PI_F = 3.141593;
// set the target distance bet. car and right wall
float set_dis = 1.0;
float Kp = 0.5;
float Kd = 5;
float Ki = 0.0001;
float current_error;
float last_error;
float PID_i;
ros::Publisher steer_control;
std::map<int, float> scan, right_side_scan;

std::ofstream myFile;




// find the max value in map
std::pair<int, float> get_max(const std::map<int, float> &x)
{
    using pairtype = std::pair<int, float>;
    return *std::max_element(x.begin(), x.end(), [](const pairtype &p1, const pairtype &p2)
                             { return p1.second < p2.second; });
}
// find the min value in map
std::pair<int, float> get_min(const std::map<int, float> &x)
{
    using pairtype = std::pair<int, float>;
    return *std::min_element(x.begin(), x.end(), [](const pairtype &p1, const pairtype &p2)
                             { return p1.second < p2.second; });
}

// get error term at time t
// only works at small speed, when speed bigger than 3
// it will over steer and turns into full circle
float get_error_term(void)
{
    auto car_to_right_mindis = get_min(right_side_scan);

    float car_right_min = car_to_right_mindis.second;
    float front_dis = scan[540];
    float rear_dis = scan[1079];
    float right_dis = scan[270];
    float dis_to_wall = 0;
    float error_term = 0;

    if (car_right_min > set_dis)
    {
        // if car heading left, turns right
        // if car heading right, do nothing
        float alpha = atan2(right_dis, rear_dis);
        dis_to_wall = sin(alpha) * rear_dis;
        error_term = set_dis - dis_to_wall;

    }
    else if (car_right_min < set_dis)
    {
        // if car heading right, turns left
        // if car heading left, do nothing
        float alpha = atan2(right_dis, front_dis);
        dis_to_wall = sin(alpha) * front_dis;
        error_term = set_dis - dis_to_wall;

    }
    return error_term;
}

// get error term at time t+t0
float get_error_term_t0(float speed, float t0)
{
    auto car_to_right_mindis = get_min(right_side_scan);

    float car_right_min = car_to_right_mindis.second;
    float front_dis = scan[540];
    float rear_dis = scan[1];
    float right_dis = scan[270];
    float dis_to_wall = 0;
    float error_term = 0;

    if (car_right_min > set_dis)
    {
        // if car heading left, turns right
        // if car heading right, do nothing
        float alpha = atan2(right_dis, rear_dis);
        dis_to_wall = sin(alpha)*speed*t0 + rear_dis*sin(alpha);
        error_term = set_dis - dis_to_wall;
    }
    else if (car_right_min < set_dis)
    {
        // if car heading right, turns left
        // if car heading left, do nothing
        float alpha = atan2(right_dis, front_dis);
        dis_to_wall = sin(alpha) * (front_dis - speed*t0) ;

        error_term = set_dis - dis_to_wall;
    }
    return error_term;
}



float get_error_derivative(float error){
    current_error = error;
    float derivative = last_error - current_error;
    last_error = current_error;
    return derivative;
}

float get_error_integral(float error, float ki){
    PID_i = PID_i + ki*error;
    return PID_i;
}

void write_csv(float steer_angle){
    myFile << steer_angle << "\n";

}

void callback(const sensor_msgs::LaserScan &laserscan)
{
    for (int i = 0; i <= 1079; i++)
    {
        scan[i] = laserscan.ranges[i];
    }
    // focus on the right side of scan area
    for (int j = 0; j <= 539; j++)
    {
        right_side_scan[j] = scan[j];
    }

    float speed = 2.0;
    // use rostopic hz /drive to read out publishing rate
    // it stables at 355Hz, that means t0 = 1/355
    float t0 = 1/355;
    float error = get_error_term();
    float error_pid = get_error_term_t0(speed,t0);

    //float steer_angle = Kp * error;
    //float steer_angle = Kp * error_pid;

    //float steer_angle = Kp * error + Kd * get_error_derivative(error);
    //float steer_angle = Kp * error_pid + Kd * get_error_derivative(error_pid);

    float steer_angle = Kp * error_pid + Kd * get_error_derivative(error_pid) + get_error_integral(error_pid, Ki);



    write_csv(steer_angle);


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
    steer_control.publish(drive_st_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pidFollowWall");
    ros::NodeHandle node;

    ros::Subscriber scan_sub = node.subscribe("/scan", 1, callback);
    steer_control = node.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);

    // create a csv file
    myFile= std::ofstream("/home/cps-student-3/Desktop/steering angle.csv");



    ros::spin();
    myFile.close();
}
