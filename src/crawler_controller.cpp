
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub_pwm1,pub_pwm2,pub_dir1,pub_dir2;

float b = 0.375 / 2.0f;
float r = 0.045;

void
CommandCallback (const geometry_msgs::TwistConstPtr& cmd_vel)
{
    float v = cmd_vel->linear.x;
    float omega = cmd_vel->angular.z;
    float v_l,v_r;
    v_l = (v - omega * b)/r;
    v_r = (v + omega * b)/r;
    std_msgs::Bool dir_l,dir_r;
    std_msgs::UInt8 pwm_l,pwm_r;

    if(v_l > 0) dir_l.data = true;
    else dir_l.data = false;

    if(v_r > 0) dir_r.data = true;
    else dir_r.data = false;

    float eta = ( 0.5 + 1.5 * b)/r;
    pwm_l.data = (abs(v_l)/eta) * 255;
    pwm_r.data = (abs(v_r)/eta) * 255;
    pwm_l.data = (abs(v_l)/eta) * 255;
    pwm_r.data = (abs(v_r)/eta) * 255;

    // pwm_l.data = (abs(v_l)) * 255;
    // pwm_r.data = (abs(v_r)) * 255;
    // if(pwm_l.data>255) pwm_l.data = 250;
    // if(pwm_r.data>255) pwm_r.data = 250;

    pub_pwm1.publish (pwm_l);
    pub_pwm2.publish (pwm_r);
    pub_dir1.publish (dir_l);
    pub_dir2.publish (dir_r);
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
    pub_pwm1 = nh.advertise<std_msgs::UInt8> ("pwm_r", 1);
    pub_pwm2 = nh.advertise<std_msgs::UInt8> ("pwm_l", 1);
    pub_dir1 = nh.advertise<std_msgs::Bool> ("dir_r", 1);
    pub_dir2 = nh.advertise<std_msgs::Bool> ("dir_l", 1);
    // Spin
    ros::spin ();
}