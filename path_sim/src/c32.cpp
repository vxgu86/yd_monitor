#include <boost/thread.hpp>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

using namespace std;

double pos, t_pos, vel, t_vel, acc, t_acc, jeck;
double max_vel, min_vel, max_acc, min_acc;
int frequency;
double ek, ek1, ek2;
double e1_min, e1_max, e2_min, e2_max;
double qk2, qk1, qk, uk;

double sign(double x)
{
    if (x > 0)
    {
        return 1;
    }
    else if (x == 0)
    {
        return 0;
    }
    else
    {
        return -1;
    }
}

double power(double x, int n)
{
    double val = 1.0;
    while (n--)
        val *= x;
    return val;
}

//δ v (v) = ë k |ë k | + 2( ė k − v)
double cv_uvv(double v)
{
    return ek2 * abs(ek2) + 2 * (ek1 - v);
}
//u cv (v) = −U sign δ v (v) + (1 − |sign(δ v (v))|)ë k
double cv_ucv(double v)
{
    double v1 = cv_uvv(v);
    double v2 = 1 - abs(sign(cv_uvv(v)));
    return -1 * jeck * sign(v1 + v2);
}
//u a (a) = −U sign(ë k − a).
double cv_ua(double v)
{
    return -1 * jeck * sign(ek2 - v);
}
//u v (v) = max u a (ë min ), min{u cv (v), u a (ë max )}
double cv_uv(double v)
{
    double v1 = cv_ua(e2_min);
    double v2 = min(cv_ucv(v), cv_ua(e2_max));
    return max(v1, v2);
}

void c3_function()
{
    double c31 = ek1 + ek2 * abs(ek2) / 2;
    double s_c31 = sign(c31);
    double p1 = 2 * power(ek2 * ek2 + 2 * ek1 * s_c31, 3);
    double c32 = ek + ek1 * ek2 * s_c31 - ek2 * ek2 * ek2 / 6 * (1 - 3 * abs(s_c31)) + s_c31 / 4 * sqrt(p1);
    double v_add = ek - e2_max * (ek2 * ek2 - 2 * ek1) / 4 - (ek2 * ek2 - 2 * ek1) * (ek2 * ek2 - 2 * ek1) / e2_max / 8 - ek2 * (3 * ek1 - ek2 * ek2) / 3;
    double v_sub = ek - e2_min * (ek2 * ek2 + 2 * ek1) / 4 - (ek2 * ek2 + 2 * ek1) * (ek2 * ek2 + 2 * ek1) / e2_min / 8 + ek2 * (3 * ek1 + ek2 * ek2) / 3;
    double c35 = 0;
    if (ek2 <= e2_max && ek1 <= (ek2 * ek2 / 2 - e2_max * e2_max))
    {
        c35 = v_add;
    }
    else if (ek2 >= e2_min && ek1 >= (e2_min * e2_min - ek2 * ek2 / 2))
    {
        c35 = v_sub;
    }
    else
    {
        c35 = c32;
    }
    double uc_1 = 1 - abs(sign(c35));
    double uc_2 = c31 + (1 - abs(sign(c31))) * ek2;
    double uc = -1 * jeck * sign(c35 + uc_1 * uc_2);
    double uk_1 = cv_uv(e1_min);
    double uk_2 = min(uc, cv_uv(e1_max));
    uk = max(uk_1, uk_2);
    cout << "uk:" << uk << endl;
    // double uk = max(uk_1, uk_2);
    // return uk;
}
//q̈ k = q̈ k−1 + T s u k−1
//前一次时刻的加速度/速度/位置
void get_qk(double l_acc, double l_vel, double l_pos, double &acc_, double &vel_, double &pos_)
{
    cout << "frequency:" << frequency << " uk:" << uk << endl;
    double qk2 = l_acc + (double)1 / frequency * uk;
    double qk1 = l_vel + (double)1 / frequency / 2 * (qk2 + l_acc);
    double qk = l_pos + (double)1 / frequency / 2 * (qk1 + l_vel);
    cout << "qk:" << qk << " qk1:" << qk1 << " qk2:" << qk2 << endl;
    pos = qk;
    vel = qk1;
    acc = qk2;

    pos_ = qk;
    vel_ = qk1;
    acc_ = qk2;
}

void init(double c_pos, double target_pos, double c_vel, double target_vel, double c_acc, double target_acc)
{
    pos = c_pos;
    t_pos = target_pos;
    vel = c_vel;
    t_vel = target_vel;
    acc = c_acc;
    t_acc = target_acc;
}

void prepare()
{
    //1
    ek = (pos - t_pos) / jeck;
    ek1 = (vel - t_vel) / jeck;
    ek2 = (acc - t_acc) / jeck;
    //2
    e1_min = (min_vel - t_vel) / jeck;
    e1_max = (max_vel - t_vel) / jeck;
    e2_min = (min_acc - t_acc) / jeck;
    e2_max = (max_acc - t_acc) / jeck;

    c3_function();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ros_node_c3");
    ros::NodeHandle nh;

    nh.param<int>("/c3/frequency", frequency, 10);
    nh.param<double>("/c3/max_vel", max_vel, 0.3);
    nh.param<double>("/c3/min_vel", min_vel, 0.0);
    nh.param<double>("/c3/max_acc", max_acc, 1.0);
    nh.param<double>("/c3/min_acc", min_acc, -1.0);
    nh.param<double>("/c3/jeck", jeck, 10.0);

    cout << "frequency:" << frequency << endl;
    cout << "max_vel:" << max_vel << endl;
    cout << "min_vel:" << min_vel << endl;
    cout << "max_acc:" << max_acc << endl;
    cout << "min_acc:" << min_acc << endl;
    cout << "jeck:" << jeck << endl;

    //test publish
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/test/vel", 1, true);
    init(0, 1, 0, 0, 0, 0);
    prepare();

    double current_pos = 0;
    double current_vel = 0;
    double current_acc = 0;
    double last_pos = 0;
    double last_vel = 0;
    double last_acc = 0;

    get_qk(last_acc, last_vel, last_pos, current_acc, current_vel, current_pos);
    prepare();
    last_pos = current_pos;
    last_vel = current_vel;
    last_acc = current_acc;
    //cout << "pos:" << current_pos << " vel:" << current_vel << " acc:" << current_acc << endl;

    //set frequency
    ros::Rate rate(frequency);
    int count = 0;
    while (ros::ok())
    {
        count++;
        if (count < 1000)
        {
            //double secs = ros::Time::now().toSec();
            //cout << "time:" << secs << endl;
            get_qk(last_acc, last_vel, last_pos, current_acc, current_vel, current_pos);
            prepare();
            last_pos = current_pos;
            last_vel = current_vel;
            last_acc = current_acc;
            cout << "pos:" << current_pos << " vel:" << current_vel << " acc:" << current_acc << endl;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}