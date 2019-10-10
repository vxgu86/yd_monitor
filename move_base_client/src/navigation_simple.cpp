#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(1.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose.position.x = 2.67;
    goal.target_pose.pose.position.y = 6.0;
    goal.target_pose.pose.position.z = 1.0;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = -0.077;
    goal.target_pose.pose.orientation.w = 0.996;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("SUCCEEDED...");
    else if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
    {
        //std::count << ac.getState() << std::endl;
        ROS_INFO("ABORTED...");
    }
    else
    {
        ROS_INFO("failed...");
    }

    return 0;
}