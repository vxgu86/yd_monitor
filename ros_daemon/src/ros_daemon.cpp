#include "ros/ros.h"
#include <cstdlib>
#include <iostream>
#include "std_srvs/SetBool.h"
#include <stdio.h>
#include <string.h>

using namespace std;

std::string topic;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "daemon");

    ros::param::get("/monitor/topic", topic);

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>(topic);

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        std_srvs::SetBool srv;
        if (client.call(srv))
        {
            cout << "send message ..." << srv.response.success << endl;
            count = 0;
        }
        else
        {
            cout << "Failed to call service add_two_ints" << count << endl;
            count++;
            if (count >= 30)
            {
                cout << "-----------restart node-----------" << count << endl;
                //system("roslaunch  unity_simulation_scene unity_sim_pointcloud.launch");
                FILE *fp;
                char buffer[80];
                memset(buffer, 0x00, sizeof(buffer));

                fp = popen("~/restart_hdl.sh", "r");
                fgets(buffer, sizeof(buffer), fp);

                printf("[%s]\n", buffer);

                pclose(fp);

                count = -50;
            }
        }
    }
    return 0;
}