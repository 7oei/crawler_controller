
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub_left,pub_right;

float
sign(float value)
{
    return (value>0)-(value<0);
}

std::vector<float> 
motor_command_gen(std::vector<float> robot_command,float base,float wheel_r,float reduction)
{
    std::vector<float> motor_command(2);
    float angular_velocity_left_drive_wheel = (robot_command[0] - robot_command[1] * base / 2.0f) / wheel_r;
    float angular_velocity_right_drive_wheel = (robot_command[0] + robot_command[1] * base / 2.0f) / wheel_r;
    float angular_velocity_left_motor = angular_velocity_left_drive_wheel * reduction;
    float angular_velocity_right_motor = angular_velocity_right_drive_wheel * reduction;
    float rps_left_motor = angular_velocity_left_motor / (M_PI * 2.0f);
    float rps_right_motor = angular_velocity_right_motor / (M_PI * 2.0f);
    motor_command[0] = rps_left_motor;
    motor_command[1] = rps_right_motor;
    return motor_command;
}

std::vector<float> 
keep_curvature_robot_command_gen(std::vector<float> motor_command,float base,float wheel_r,float reduction,float curvature_radius,bool singular)
{
    std::vector<float> robot_command(2);
    float angular_velocity_left_motor = motor_command[0] * M_PI * 2.0f;
    float angular_velocity_right_motor = motor_command[1] * M_PI * 2.0f;
    float angular_velocity_left_drive_wheel = angular_velocity_left_motor / reduction;
    float angular_velocity_right_drive_wheel = angular_velocity_right_motor / reduction;
    float translation_velocity_left_drive_wheel = angular_velocity_left_drive_wheel * wheel_r;
    float translation_velocity_right_drive_wheel = angular_velocity_right_drive_wheel * wheel_r;
    if(singular){
        robot_command[0] = translation_velocity_right_drive_wheel;
        robot_command[1] = 0.0f;
    }
    else {
        robot_command[1] = (translation_velocity_right_drive_wheel - translation_velocity_left_drive_wheel) / base;
        robot_command[0] = robot_command[1] * curvature_radius;
    }
    return robot_command;
}


void
CommandCallback (const geometry_msgs::TwistConstPtr& cmd_vel)
{
    float base = 0.9;
    float wheel_r = 0.12;
    float reduction = 36.0f;
    float motor_limit_rps = 72.0f;
    std::vector<float> input_robot_command(2);
    input_robot_command[0] = cmd_vel->linear.x;
    input_robot_command[1] = cmd_vel->angular.z;
    float curvature_radius;
    bool singular = false;
    if(abs(input_robot_command[1]) < 0.01f){ // curvature = 0
        curvature_radius = 0.0f;
        singular = true;
    } 
    else curvature_radius = input_robot_command[0] / input_robot_command[1];
    std::vector<float> pure_motor_command = motor_command_gen(input_robot_command, base, wheel_r, reduction);
    std::vector<float> limit_motor_command(2);
    limit_motor_command[0] = sign(pure_motor_command[0]) * std::min(motor_limit_rps, abs(pure_motor_command[0]));
    limit_motor_command[1] = sign(pure_motor_command[1]) * std::min(motor_limit_rps, abs(pure_motor_command[1]));
    std::vector<float> limit_robot_keep_curvature_command = keep_curvature_robot_command_gen(limit_motor_command, base, wheel_r, reduction, curvature_radius, singular);
    std::vector<float> limit_motor_keep_curvature_command = motor_command_gen(limit_robot_keep_curvature_command, base, wheel_r, reduction);
    std_msgs::Float32 left_rps,right_rps;
    left_rps.data = limit_motor_keep_curvature_command[0];
    right_rps.data = limit_motor_keep_curvature_command[1];
    pub_left.publish (left_rps);
    pub_right.publish (right_rps);
}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "crawler_controller");
    ros::NodeHandle nh;
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("cmd_vel", 1, CommandCallback);
    // Create a ROS publisher for the output point cloud
    pub_left = nh.advertise<std_msgs::Float32> ("left_rps", 1);
    pub_right = nh.advertise<std_msgs::Float32> ("right_rps", 1);
    // Spin
    ros::spin ();
}