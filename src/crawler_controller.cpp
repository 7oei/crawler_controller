
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <crawler_controller/Status.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

ros::Publisher pub_left,pub_right;
ros::Publisher odom_pub;

bool emergency = false;
bool initialize = true;
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

std::vector<float> 
robot_status_gen(std::vector<float> motor_status,float base,float wheel_r,float reduction)
{
    std::vector<float> robot_status(2);
    float angular_velocity_left_motor = motor_status[0] * M_PI * 2.0f;
    float angular_velocity_right_motor = motor_status[1] * M_PI * 2.0f;
    float angular_velocity_left_drive_wheel = angular_velocity_left_motor / reduction;
    float angular_velocity_right_drive_wheel = angular_velocity_right_motor / reduction;
    float translation_velocity_left_drive_wheel = angular_velocity_left_drive_wheel * wheel_r;
    float translation_velocity_right_drive_wheel = angular_velocity_right_drive_wheel * wheel_r;
    robot_status[1] = (translation_velocity_right_drive_wheel - translation_velocity_left_drive_wheel) / base;
    robot_status[0] = (translation_velocity_right_drive_wheel + translation_velocity_left_drive_wheel) / 2;
    return robot_status;
}

void
CommandCallback (const geometry_msgs::TwistConstPtr& cmd_vel)
{
    float base = 0.9;
    float wheel_r = 0.12;
    float reduction = 36.0f * 1.7;//1.7は駆動輪→クローラー減速分
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
    if(emergency){
        std::cout << "emergency"<< std::endl;
        left_rps.data = 0;
        right_rps.data = 0;
    }
    pub_left.publish (left_rps);
    pub_right.publish (right_rps);
}

std::vector<float> last_motor_pos(2,0);
std::vector<float> g_robot_pos(3,0);

void 
update_robot_position(std::vector<float>& robot_pos, const std::vector<float>& robot_move) {
    // ロボットの移動量の取得
    float path_length = robot_move[0];
    float yaw = robot_move[1];

    // 前回の位置情報の取得
    float x = robot_pos[0];
    float y = robot_pos[1];
    float current_yaw = robot_pos[2];

    // 位置の更新式
    float curvature_radius = path_length / yaw;

    // 曲率半径が非ゼロの場合は円弧に沿った位置の更新を行う
    if (std::abs(yaw) > 1e-6) {
        float delta_x = curvature_radius * (std::sin(current_yaw + yaw) - std::sin(current_yaw));
        float delta_y = curvature_radius * (std::cos(current_yaw) - std::cos(current_yaw + yaw));
        x += delta_x;
        y += delta_y;
    }
    // 曲率半径がゼロの場合は直進運動に基づいた位置の更新を行う
    else {
        x += path_length * std::cos(current_yaw);
        y += path_length * std::sin(current_yaw);
    }

    // 更新した位置情報を格納
    robot_pos[0] = x;
    robot_pos[1] = y;
    robot_pos[2] = current_yaw + yaw;
}

void
StatusCallback (const crawler_controller::StatusConstPtr& status)
{
    if(status->current>100) emergency = true;
    if(status->voltage<22.2) emergency = true;
    if(initialize){
        last_motor_pos[0] = status->left_pos;
        last_motor_pos[1] = status->right_pos;
        initialize = false;
    }
    float base = 0.9;
    float wheel_r = 0.12;
    float reduction = 36.0f;
    float motor_limit_rps = 72.0f;
    std::vector<float> current_motor_pos(2);
    current_motor_pos[0] = status->left_pos;
    current_motor_pos[1] = status->right_pos;
    std::vector<float> relative_motor_pos(2);
    relative_motor_pos[0] = current_motor_pos[0] - last_motor_pos[0];
    relative_motor_pos[1] = current_motor_pos[1] - last_motor_pos[1];
    //-4096←→4096　マタギ
    if(abs(relative_motor_pos[0])>4096) relative_motor_pos[0] = - sign(relative_motor_pos[0]) * (8192-abs(relative_motor_pos[0]));
    if(abs(relative_motor_pos[1])>4096) relative_motor_pos[1] = - sign(relative_motor_pos[1]) * (8192-abs(relative_motor_pos[1]));
    last_motor_pos[0] = current_motor_pos[0];
    last_motor_pos[1] = current_motor_pos[1];
    std::vector<float> robot_move = robot_status_gen(relative_motor_pos,base,wheel_r,reduction);
    update_robot_position(g_robot_pos, robot_move);
    std::vector<float> current_motor_rps(2);
    current_motor_rps[0] = status->left_rps;
    current_motor_rps[1] = status->right_rps;
    std::vector<float> robot_twist = robot_status_gen(current_motor_rps,base,wheel_r,reduction);
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = g_robot_pos[0];
    odom.pose.pose.position.y = g_robot_pos[1];
    odom.pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(g_robot_pos[2]);
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = robot_twist[0];
    odom.twist.twist.angular.z = robot_twist[1];
    odom_pub.publish(odom);
}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "crawler_controller");
    ros::NodeHandle nh;
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber cmd_sub = nh.subscribe ("cmd_vel", 1, CommandCallback);
    ros::Subscriber status_sub = nh.subscribe ("motors_status", 1, StatusCallback);
    // Create a ROS publisher for the output point cloud
    pub_left = nh.advertise<std_msgs::Float32> ("left_rps", 1);
    pub_right = nh.advertise<std_msgs::Float32> ("right_rps", 1);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0);
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.angular.z = 0;
    odom_pub.publish(odom);
    // Spin
    ros::spin ();
}