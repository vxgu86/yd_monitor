#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace move_base_msgs;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> Client;

class MyNode
{
public:
    MyNode() : ac("move_base", true)
    {
        ROS_INFO("Waiting for action server to start.");
        ac.waitForServer();
        ROS_INFO("Action server started, sending goal.");
    }

    void doStuff(int order)
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
    }

    void doneCb(const actionlib::SimpleClientGoalState &state,
                const move_base_msgs::MoveBaseActionResultConstPtr &result)
    {
        ROS_INFO("Finished in state [%s]", state.toString().c_str());
        //ROS_INFO("Answer: %i", result->sequence.back());
        //ros::shutdown();
    }

private:
    Client ac;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation");
    MyNode my_node;
    my_node.doStuff(10);
    ros::spin();
    return 0;
}
