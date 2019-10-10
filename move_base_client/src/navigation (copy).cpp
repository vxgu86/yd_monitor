#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "move_base_client/wali_go_to_position.h"

#include <iostream>

using namespace std;
using namespace move_base_msgs;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

class MyNode
{
public:
    MyNode() : client("move_base", true)
    {
        ROS_INFO("Waiting for action server to start.");
        client.waitForServer();
        ROS_INFO("Action server started.");

        ros::NodeHandle simple_nh("");
        nav_server = simple_nh.advertiseService("go_to_position", &MyNode::GoalCb, this);
    }

    void doStuff()
    {
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.pose.position.x = 2.67;
        goal.target_pose.pose.position.y = 6.0;
        goal.target_pose.pose.position.z = 1.0;
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = -0.077;
        goal.target_pose.pose.orientation.w = 0.996;
        // Need boost::bind to pass in the 'this' pointer
        client.sendGoal(goal,
                        boost::bind(&MyNode::DoneCb, this, _1, _2),
                        boost::bind(&MyNode::ActiveCb, this),
                        boost::bind(&MyNode::FeedbackCb, this, _1));
    }

    // Called once when the goal completes
    void DoneCb(const actionlib::SimpleClientGoalState &state,
                const move_base_msgs::MoveBaseResultConstPtr &result)
    {
        ROS_INFO("state [%s]", state.toString().c_str());
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
        }
        else if (state == actionlib::SimpleClientGoalState::ABORTED)
        {
            //std::count << ac.getState() << std::endl;
            ROS_INFO("ABORTED...");
        }
        else
        {
            ROS_INFO("failed...");
        }
        //ROS_INFO("Toal dish cleaned: %i", result->toal_dishes_cleaned);
        //ros::shutdown();
    }

    // 当目标激活的时候，会调用一次
    void ActiveCb()
    {
        ROS_INFO("Goal just went active");
    }

    // 接收服务器的反馈信息
    void FeedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
    {
        move_base_msgs::MoveBaseFeedback ori_feed = *feedback;
        ROS_INFO("result message [%s] [%d]", ori_feed.status.text, ori_feed.status.status);
        ROS_INFO("Got Feedback Complete Rate: %f", feedback);
    }

    bool GoalCb(move_base_client::wali_go_to_position::Request &req,
                move_base_client::wali_go_to_position::Response &res) //这个函数提供两个int值求和的服务，int值从request里面获取，而返回数据装入response内，这些数据类型都定义在srv文件内部，函数返回一个boolean值。
    {
        cout << "receive goal" << endl;
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.pose.position.x = 2.67;
        goal.target_pose.pose.position.y = 6.0;
        goal.target_pose.pose.position.z = 1.0;
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = -0.077;
        goal.target_pose.pose.orientation.w = 0.996;
        // Need boost::bind to pass in the 'this' pointer
        //client.sendGoal(goal);
        client.sendGoal(goal,
                        boost::bind(&MyNode::DoneCb, this, _1, _2),
                        boost::bind(&MyNode::ActiveCb, this),
                        boost::bind(&MyNode::FeedbackCb, this, _1));

        res.success = true;
        return true;
    }

private:
    Client client;
    ros::ServiceServer nav_server;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yd_navigation");
    MyNode my_node;
    //my_node.doStuff(10);
    ros::spin();
    return 0;
}
