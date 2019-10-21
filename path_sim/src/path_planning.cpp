#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <boost/regex.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "Vect.cpp"
#include "utils.cpp"

#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"

using namespace std;
using namespace Eigen;

typedef Matrix<float, 4, 1> Vector4f;

enum PlanStage
{
    None,
    Await,
    Plan,
    ArriveArea,
    Rotate,
    Finish,
};

class PathPlaning
{
public:
    PathPlaning();
    ~PathPlaning();
    void path_callback(const nav_msgs::PathConstPtr &path_msg);
    void pose_callback(const nav_msgs::OdometryConstPtr &pose_msg);
    void goal_callback(const geometry_msgs::PoseStampedConstPtr &goal_msg);
    pair<double, double> calculate_distance_angle(geometry_msgs::PoseStamped self_pos, geometry_msgs::PoseStamped goal_pos);
    double radian_to_angle(double radian);
    pair<double, double> calc_twist(bool is_left, float angle, float distance);
    float dot(Vector3d v1, Vector3d v2);
    Vector3d normalized(Vector3d v1);
    double length(Vector3d v1);
    void deal_str(string topic, geometry_msgs::PoseStamped *goal_msg);
    void control_callback(const std_msgs::Int16::ConstPtr &msg);

private:
    ros::NodeHandle nh;
    ros::Publisher cmd_pub, forward_pub, right_pub, goal_pub;
    ros::Subscriber pose_sub, goal_sub, path_sub, control_sub;
    geometry_msgs::PoseStamped goal, robot_pose, next_pose, a_pose, b_pose, c_pose, d_pose;
    bool is_arrived;
    int pos_type;
    nav_msgs::Path path;
    Vector3d next_goal, dir, forward, right;
    Vector4f robot_qua, navigation_qua;
    std::string a_s, b_s, c_s, d_s;
    PlanStage plan_stage;
    float max_velocity, max_angular;
};

PathPlaning ::PathPlaning()
{
    path_sub = nh.subscribe("/move_base/GlobalPlanner/plan", 1, &PathPlaning::path_callback, this);
    pose_sub = nh.subscribe("/odom_localization", 1, &PathPlaning::pose_callback, this);
    goal_sub = nh.subscribe("/move_base_simple/goal", 1, &PathPlaning::goal_callback, this);
    control_sub = nh.subscribe<std_msgs::Int16>("/path_sim/go", 1, &PathPlaning::control_callback, this);
    cmd_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1, true);
    forward_pub = nh.advertise<nav_msgs::Odometry>("/path_sim/forward", 1, false);
    right_pub = nh.advertise<nav_msgs::Odometry>("/path_sim/right", 1, false);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, false);

    nh.param<std::string>("/path_planning/a_point", a_s, "dd");
    nh.param<std::string>("/path_planning/b_point", b_s, "dd");
    nh.param<std::string>("/path_planning/c_point", c_s, "dd");
    nh.param<std::string>("/path_planning/d_point", d_s, "dd");
    cout << "a_point:" << a_s << endl;
    cout << "b_point:" << b_s << endl;
    cout << "c_point:" << c_s << endl;
    cout << "d_point:" << d_s << endl;
    deal_str(a_s, &a_pose);
    cout << "a_pose:" << a_pose << endl;
    deal_str(b_s, &b_pose);
    cout << "b_pose:" << b_pose << endl;
    deal_str(c_s, &c_pose);
    cout << "c_pose:" << c_pose << endl;
    deal_str(d_s, &d_pose);
    cout << "d_pose:" << d_pose << endl;

    nh.param<float>("/path_planning/max_velocity", max_velocity, 0.2);
    nh.param<float>("/path_planning/max_angular", max_angular, 0.2);
    cout << "max_velocity:" << max_velocity << endl;
    cout << "max_angular:" << max_angular << endl;

    is_arrived = true;
    pos_type = 0;
    plan_stage = None;
}

PathPlaning ::~PathPlaning()
{
}

void PathPlaning::path_callback(const nav_msgs::PathConstPtr &path_msg)
{
    cout << "get planing path " << endl;
    if (path_msg->poses.size() > 0 && plan_stage != None && plan_stage != Finish)
    {
        path = *path_msg;
        int path_size = (50 < path_msg->poses.size()) ? 50 : path_msg->poses.size();
        next_goal = Vector3d(0, 0, 0);
        for (int i = 0; i < path_size; i++)
        {
            next_goal[0] += path.poses[i].pose.position.x;
            next_goal[1] += path.poses[i].pose.position.y;
            next_goal[2] += path.poses[i].pose.position.z;
        }
        next_goal = next_goal / path_size;
        if (plan_stage == Await)
        {
            plan_stage = Plan;
        }
    }
}

void PathPlaning::control_callback(const std_msgs::Int16::ConstPtr &msg)
{
    cout << " update goal " << endl;
    if (msg->data == 1)
    {
        goal_pub.publish(a_pose);
    }
    else if (msg->data == 2)
    {
        goal_pub.publish(b_pose);
    }
    else if (msg->data == 3)
    {
        goal_pub.publish(c_pose);
    }
    else if (msg->data == 4)
    {
        goal_pub.publish(d_pose);
    }
}

void PathPlaning::deal_str(string topic, geometry_msgs::PoseStamped *goal_msg)
{
    goal_msg->header.frame_id = "map";
    goal_msg->header.stamp = ros::Time::now();
    boost::regex rgx("\\s+");
    boost::sregex_token_iterator iter(topic.begin(), topic.end(), rgx, -1);
    boost::sregex_token_iterator end;
    std::vector<std::string> topics;
    int index = 0;
    for (; iter != end; ++iter)
    {
        index++;
        double x = stringToNum<double>(*iter);
        //cout << "index:" << index << " value:" << x << endl;
        switch (index)
        {
        case 1:
            goal_msg->pose.position.x = x;
            continue;
        case 2:
            goal_msg->pose.position.y = x;
            continue;
        case 3:
            goal_msg->pose.position.z = x;
            continue;
        case 4:
            goal_msg->pose.orientation.x = x;
            continue;
        case 5:
            goal_msg->pose.orientation.y = x;
            continue;
        case 6:
            goal_msg->pose.orientation.z = x;
            continue;
        case 7:
            goal_msg->pose.orientation.w = x;
            continue;
        default:
            continue;
        }
        //ROS_INFO_STREAM("" << *iter);
    }
}

pair<double, double> PathPlaning::calc_twist(bool is_left, float angle, float distance)
{
    float delta_rad = angle * M_PI / 180 * max_angular;
    delta_rad = delta_rad > 0.2 ? 0.2 : delta_rad;
    float delta_dis = 0;
    if (angle > 30)
    {
        delta_dis = 0;
    }
    else
    {
        delta_dis = (31.0 - angle) / 30.0 * max_velocity;
    }
    return make_pair(delta_dis, delta_rad);
    /*
    else if (angle <= 60 && angle > 35)
    {
        delta_dis = 0.05;
    }
    else if (angle <= 35 && angle > 20)
    {
        delta_dis = 0.1;
    }
    else if (angle <= 20 && angle > 10)
    {
        delta_dis = 0.15;
    }
    else
    {
        delta_dis = 0.2;
    }


    if (is_left)
    {
        cout << "向左旋转 " << delta_rad << " 移动 " << delta_dis << endl;
        return make_pair(delta_dis, delta_rad);
    }
    else
    {
        cout << "向右旋转 " << delta_rad << " 移动 " << delta_dis << endl;
        return make_pair(delta_dis, -delta_rad);
    }
    */
}

void PathPlaning ::pose_callback(const nav_msgs::OdometryConstPtr &pose_msg)
{
    //cout << "get pose" << endl;
    //save data
    Quaternionf quanternion = Quaternionf(pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x, pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z);
    robot_qua << pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z, radian_to_angle(quanternion.toRotationMatrix().eulerAngles(0, 1, 2)[2]);

    robot_pose.pose.position.x = pose_msg->pose.pose.position.x;
    robot_pose.pose.position.y = pose_msg->pose.pose.position.y;
    robot_pose.pose.position.z = pose_msg->pose.pose.position.z;
    robot_pose.pose.orientation.x = pose_msg->pose.pose.orientation.x;
    robot_pose.pose.orientation.y = pose_msg->pose.pose.orientation.y;
    robot_pose.pose.orientation.z = pose_msg->pose.pose.orientation.z;
    robot_pose.pose.orientation.w = pose_msg->pose.pose.orientation.w;
    geometry_msgs::Twist motor_control;
    if (plan_stage == Await)
    {
        cout << "等待路径规划" << endl;
        return;
    }
    if (plan_stage == Finish)
    {
        cout << "导航结束" << endl;
        motor_control.linear.x = 0;
        motor_control.angular.z = 0;
        cmd_pub.publish(motor_control);
        return;
    }

    //calc cmd
    if (plan_stage == ArriveArea)
    {
        motor_control.linear.x = 0;
        double delta_angle = robot_qua[3] - navigation_qua[3];
        if (abs(delta_angle) > 5)
        {
            if (pos_type == 1 || pos_type == 3)
            {
                cout << "到达区域，向右旋转" << endl;
                motor_control.angular.z = -0.3;
            }
            else
            {
                cout << "到达区域，向左旋转" << endl;
                motor_control.angular.z = 0.3;
            }
        }
        else
        {
            cout << "到达区域，停止旋转" << endl;
            plan_stage == Finish;
            motor_control.angular.z = 0;
        }
        cmd_pub.publish(motor_control);
    }
    else if (plan_stage == Plan)
    {
        next_pose.pose.position.x = next_goal[0];
        next_pose.pose.position.y = next_goal[1];
        next_pose.pose.position.z = next_goal[2];
        pair<double, double> dis_ang = calculate_distance_angle(robot_pose, next_pose);
        if (dis_ang.first > 0.1)
        {
            if (pos_type == 1 || pos_type == 3)
            {
                //right/*
                pair<double, double> twist = calc_twist(false, dis_ang.second, dis_ang.first);
                motor_control.linear.x = twist.first;
                motor_control.angular.z = -twist.second;
            }
            else
            {
                //left
                pair<double, double> twist = calc_twist(true, dis_ang.second, dis_ang.first);
                motor_control.linear.x = twist.first;
                motor_control.angular.z = twist.second;
            }
            cmd_pub.publish(motor_control);
        }
        else
        {
            cout << "flag_arriving_area " << endl;
            //calc rotate type
            Eigen::Quaterniond q(goal.pose.orientation.w, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z);
            Eigen::Vector3d v(1, 0, 0);
            Eigen::Vector3d f = q * v;

            next_pose.pose.position.x = goal.pose.position.x + f[0] * 5;
            next_pose.pose.position.y = goal.pose.position.y + f[1] * 5;
            next_pose.pose.position.z = goal.pose.position.z + f[2] * 5;

            pair<double, double> dis_ang = calculate_distance_angle(robot_pose, next_pose);

            plan_stage = ArriveArea;
        }
    }
}

double PathPlaning::radian_to_angle(double radian)
{
    return 180 * radian / M_PI;
}

void PathPlaning ::goal_callback(const geometry_msgs::PoseStampedConstPtr &goal_msg)
{
    cout << "get goal " << endl;
    goal = *goal_msg;
    plan_stage = Await;

    Quaternionf quanternion = Quaternionf(goal_msg->pose.orientation.w, goal_msg->pose.orientation.x, goal_msg->pose.orientation.y, goal_msg->pose.orientation.z);
    navigation_qua << goal_msg->pose.position.x, goal_msg->pose.position.y, goal_msg->pose.position.z, radian_to_angle(quanternion.toRotationMatrix().eulerAngles(0, 1, 2)[2]);
}

pair<double, double> PathPlaning::calculate_distance_angle(geometry_msgs::PoseStamped self_pos, geometry_msgs::PoseStamped goal_pos)
{
    //calc robot goal relative position
    //dir = Vect(, goal_pos.pose.position.y - self_pos.pose.position.y, goal_pos.pose.position.z - self_pos.pose.position.z);
    float delta_x = goal_pos.pose.position.x - self_pos.pose.position.x;
    float delta_y = goal_pos.pose.position.y - self_pos.pose.position.y;
    float delta_z = goal_pos.pose.position.z - self_pos.pose.position.z;
    dir << delta_x, delta_y, delta_z;
    //calc forward direction
    Eigen::Quaterniond q(self_pos.pose.orientation.w, self_pos.pose.orientation.x, self_pos.pose.orientation.y, self_pos.pose.orientation.z);
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d f = q * v;
    float d1 = dot(f, normalized(dir));

    // publish the transform
    nav_msgs::Odometry fodom;
    fodom.header.frame_id = "map";
    fodom.header.stamp = ros::Time::now();
    fodom.pose.pose.position.x = f[0] + self_pos.pose.position.x;
    fodom.pose.pose.position.y = f[1] + self_pos.pose.position.y;
    fodom.pose.pose.position.z = f[2] + self_pos.pose.position.z;

    forward_pub.publish(fodom);

    //calc right direction
    Eigen::AngleAxisd QX90(-M_PI / 2, Eigen::Vector3d(0, 0, 1));
    Eigen::Quaterniond t_Q(QX90);
    Eigen::Vector3d r = q * t_Q * v;

    nav_msgs::Odometry rodom;
    rodom.header.frame_id = "map";
    rodom.header.stamp = ros::Time::now();
    rodom.pose.pose.position.x = r[0] + self_pos.pose.position.x;
    rodom.pose.pose.position.y = r[1] + self_pos.pose.position.y;
    rodom.pose.pose.position.z = r[2] + self_pos.pose.position.z;

    right_pub.publish(rodom);

    float d2 = dot(r, normalized(dir));
    if (d1 > 0)
    {
        cout << "前 ";
        if (d2 > 0)
        {
            cout << "右 " << endl;
            pos_type = 1;
        }
        else
        {
            cout << "左 " << endl;
            pos_type = 2;
        }
    }
    else
    {
        cout << "后 ";
        if (d2 > 0)
        {
            cout << "右 " << endl;
            pos_type = 3;
        }
        else
        {
            cout << "左 " << endl;
            pos_type = 4;
        }
    }
    //calc angle
    double angle = acos(d1) * 180 / 3.1415926;
    cout << "angle:" << angle << endl;

    //calc distance
    double distance = sqrt(pow(goal_pos.pose.position.x - self_pos.pose.position.x, 2) + pow(goal_pos.pose.position.y - self_pos.pose.position.y, 2));
    return make_pair(distance, angle);
}

float PathPlaning::dot(Vector3d v1, Vector3d v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

Vector3d PathPlaning::normalized(Vector3d v1)
{
    double mag = length(v1);
    Eigen::Vector3d v(v1[0] / mag, v1[1] / mag, v1[2] / mag);
    return v;
}

double PathPlaning::length(Vector3d v1)
{
    return sqrt((v1[0] * v1[0]) + (v1[1] * v1[1]) + (v1[2] * v1[2]));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_sim");
    ros::start();

    PathPlaning pp;
    ros::spin();

    ros::shutdown();
    return 0;
}
