#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "sensor_msgs/CompressedImage.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "yidamsg/pointcloud_color.h"

#include "boost/thread/thread.hpp"
#include "boost/bind.hpp"
#include "boost/thread/mutex.hpp"

using namespace std;

class message_sync_ros_node
{
private:
    ros::NodeHandle node_;
    ros::Publisher rgb_pub;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::CompressedImage, nav_msgs::Odometry> slamSyncPolicy;
    message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> *thermal_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *yt_sub_;
    message_filters::Synchronizer<slamSyncPolicy> *sync_;
    std::string odom_topic;
    std::string visible_topic;
    std::string thermal_topic;
    std::string yt_topic;

public:
    message_sync_ros_node();
    ~message_sync_ros_node();
    void callback(const nav_msgs::Odometry::ConstPtr &odom_data, const sensor_msgs::CompressedImage::ConstPtr &thermal_image, const nav_msgs::Odometry::ConstPtr &yt_data);
    void update();
};

message_sync_ros_node::message_sync_ros_node()
{
    ros::param::get("/message_sync/odom_topic", odom_topic);
    ros::param::get("/message_sync/visible_topic", visible_topic);
    ros::param::get("/message_sync/thermal_topic", thermal_topic);
    ros::param::get("/message_sync/yt_topic", yt_topic);
    std::cout << "odom_topic:" << odom_topic << std::endl;
    std::cout << "visible_topic:" << visible_topic << std::endl;
    std::cout << "thermal_topic:" << thermal_topic << std::endl;
    std::cout << "yt_topic:" << yt_topic << std::endl;
    rgb_pub = node_.advertise<yidamsg::pointcloud_color>("/yd/pointcloud/vt", 10);
    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(node_, odom_topic, 1);
    thermal_sub_ = new message_filters::Subscriber<sensor_msgs::CompressedImage>(node_, thermal_topic, 1);
    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(node_, yt_topic, 1);
    sync_ = new message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(20), *odom_sub_, *thermal_sub_, *odom_sub_);
    sync_->registerCallback(boost::bind(&message_sync_ros_node::callback, this, _1, _2, _3));
}

message_sync_ros_node::~message_sync_ros_node()
{
}

void message_sync_ros_node::update()
{
}

void message_sync_ros_node::callback(const nav_msgs::Odometry::ConstPtr &odom_data, const sensor_msgs::CompressedImage::ConstPtr &thermal_image, const nav_msgs::Odometry::ConstPtr &yt_data)
{
    nav_msgs::Odometry current_pose;
    current_pose = *odom_data;

    sensor_msgs::CompressedImage t_img;
    t_img = *thermal_image;

    nav_msgs::Odometry yt_pose;
    yt_pose = *yt_data;

    yidamsg::pointcloud_color data;
    data.t_format = t_img.format;
    data.t_data = t_img.data;

    data.pos_x = current_pose.pose.pose.position.x;
    data.pos_y = current_pose.pose.pose.position.y;
    data.pos_z = current_pose.pose.pose.position.z;
    data.qua_x = current_pose.pose.pose.orientation.x;
    data.qua_y = current_pose.pose.pose.orientation.y;
    data.qua_z = current_pose.pose.pose.orientation.z;
    data.qua_w = current_pose.pose.pose.orientation.w;

    data.horizontal = yt_pose.pose.pose.position.x;
    data.vertical = yt_pose.pose.pose.position.z;

    rgb_pub.publish(data);
    std::cout
        << "callback +1 " << std::endl;
}

void callback1(const sensor_msgs::CompressedImage::ConstPtr &image1, const sensor_msgs::CompressedImage::ConstPtr &image2, const nav_msgs::Odometry::ConstPtr &data)
{
    // Solve all of perception here...
    std::cout << "sync message" << std::endl;
}

void callback2(const sensor_msgs::CompressedImage::ConstPtr &image1, const sensor_msgs::CompressedImage::ConstPtr &image2, const nav_msgs::Odometry::ConstPtr &odom_data, const std_msgs::String::ConstPtr &yuntai_data)
{
    // Solve all of perception here...
    std::cout << "sync message" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "message_sync_node");
    message_sync_ros_node node;

    ROS_INFO("message_sync_node node started...");
    ros::Rate rate(10);
    while (ros::ok())
    {
        //node.update();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
