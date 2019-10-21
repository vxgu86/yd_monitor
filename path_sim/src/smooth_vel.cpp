#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

using namespace std;

class YdSmooth
{
private:
    double accel;  //加速度
    double dccel;  //减速度
    double last_v; // 上一次记录的速度

    YdSmooth();
    ~YdSmooth();

public:
    double speed_lim_v,
        accel_lim_v,
        decel_lim_v;
    double speed_lim_u, accel_lim_u, decel_lim_u;
    double speed_lim_w, accel_lim_w, decel_lim_w;
    double decel_factor;

    double frequency;

    geometry_msgs::Twist last_cmd_vel;
    geometry_msgs::Twist current_vel;
    geometry_msgs::Twist target_vel;
};

YdSmooth ::YdSmooth()
{
}

YdSmooth ::~YdSmooth()
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "smooth_vel");
    ros::NodeHandle n;
    YdSmooth ys;

    double period = 1.0 / frequency;
    ros::Rate spin_rate(frequency);
    while (ros::ok())
    {
    }

    return 0;
}
