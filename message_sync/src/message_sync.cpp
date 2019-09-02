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
class message_sync_ros_node
{
private:
    ros::NodeHandle node_;
    ros::Publisher rgb_pub;
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> slamSyncPolicy;
    message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> *visible_sub_;
    message_filters::Subscriber<sensor_msgs::CompressedImage> *thermal_sub_;
    message_filters::Synchronizer<slamSyncPolicy> *sync_;

public:
    message_sync_ros_node();
    ~message_sync_ros_node();
    void callback(const nav_msgs::Odometry::ConstPtr &odom_data, const sensor_msgs::CompressedImage::ConstPtr &visible_image, const sensor_msgs::CompressedImage::ConstPtr &thermal_image);
};

message_sync_ros_node::message_sync_ros_node()
{
    rgb_pub = node_.advertise<yidamsg::pointcloud_color>("/yd/pointcloud/vt", 100);
    odom_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(node_, "/odom_localization", 1);
    visible_sub_ = new message_filters::Subscriber<sensor_msgs::CompressedImage>(node_, "/visible/image_proc/compressed", 1);
    thermal_sub_ = new message_filters::Subscriber<sensor_msgs::CompressedImage>(node_, "/visible/image_proc/compressed", 1);
    sync_ = new message_filters::Synchronizer<slamSyncPolicy>(slamSyncPolicy(10), *odom_sub_, *visible_sub_, *thermal_sub_);
    sync_->registerCallback(boost::bind(&message_sync_ros_node::callback, this, _1, _2, _3));
}

message_sync_ros_node::~message_sync_ros_node()
{
}

void message_sync_ros_node::callback(const nav_msgs::Odometry::ConstPtr &odom_data, const sensor_msgs::CompressedImage::ConstPtr &visible_image, const sensor_msgs::CompressedImage::ConstPtr &thermal_image)
{
    nav_msgs::Odometry::Ptr pose(new nav_msgs::Odometry);
    *pose = *odom_data;

    sensor_msgs::CompressedImage::Ptr v_img(new sensor_msgs::CompressedImage);
    *v_img = *visible_image;

    sensor_msgs::CompressedImage::Ptr t_img(new sensor_msgs::CompressedImage);
    *t_img = *thermal_image;

    yidamsg::pointcloud_color data;
    data.v_format = v_img->format;
    data.v_data = v_img->data;
    data.t_format = t_img->format;
    data.t_data = t_img->data;
    data.pose = pose->pose;

    rgb_pub.publish(data);
    std::cout
        << "Queue size 1 " << std::endl;
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
        ///node.update1();

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
