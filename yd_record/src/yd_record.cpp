
#include <rosbag_record_cpp/rosbag_record.h>
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
#include "std_msgs/Int16.h"

std::string topic, path_save, file_name;
ros::Time start_time, t;
ros::Duration duration;
bool isRecord, isStop;
int seconds;

void cmdCallback(const std_msgs::Int16::ConstPtr &msg)
{
  ROS_INFO_STREAM("msg: " << msg->data);
  //1 start 0 end
  if (msg->data == 1)
  {
    isStop = false;
  }
  else if (msg->data == 0)
  {
    isStop = true;
  }
}

int main(int argc, char **argv)
{
  ROS_INFO_STREAM_NAMED("yd_record", "yd_record");
  ros::init(argc, argv, "yd_record");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<std_msgs::Int16>("/yd_record/cmd", 1, cmdCallback);

  rosbag_record_cpp::ROSBagRecord recorder;

  //init params
  ros::param::get("/record/topic", topic);
  ros::param::get("/record/path_save", path_save);
  ros::param::get("/record/file_name", file_name);
  ros::param::get("/record/seconds", seconds);
  ROS_INFO_STREAM("topics:    " << topic);
  ROS_INFO_STREAM("name:      " << path_save);
  ROS_INFO_STREAM("file_name: " << file_name);
  ROS_INFO_STREAM("seconds: " << seconds);

  // Set the options for this recording
  rosbag_record_cpp::RecorderOptions options;
  //deal topis list
  boost::regex rgx("\\s+");
  boost::sregex_token_iterator iter(topic.begin(), topic.end(), rgx, -1);
  boost::sregex_token_iterator end;
  std::vector<std::string> topics;
  for (; iter != end; ++iter)
  {
    ROS_INFO_STREAM("topic: " << *iter);
    topics.push_back(*iter);
  }
  options.topics = topics;
  options.prefix = path_save + "/" + file_name;

  duration = ros::Duration(seconds);
  start_time = ros::Time::now();
  isRecord = false;
  isStop = false;
  ros::Rate loop_rate(1);
  while (ros::ok())
  {
    ROS_INFO_STREAM("yd_record Recording ... ");
    //stop
    if (isStop && isRecord)
    {
      ROS_INFO_STREAM("yd_record stopRecording ... ");
      recorder.stopRecording();
      isRecord = false;
    }

    // Start
    if (!isRecord && !isStop)
    {
      recorder.startRecording(options);
      start_time = ros::Time::now();
      isRecord = true;
    }

    //ros::Duration(1.0).sleep();
    if (t - start_time > duration && isRecord)
    {
      // Stop
      recorder.stopRecording();

      //delete
      boost::filesystem::recursive_directory_iterator beg_iter(path_save);
      boost::filesystem::recursive_directory_iterator end_iter;
      for (; beg_iter != end_iter; ++beg_iter)
      {
        if (boost::filesystem::is_directory(*beg_iter))
        {
          continue;
        }
        else
        {
          std::string strPath = beg_iter->path().string(); //遍历出来的文件名称
          ROS_INFO_STREAM("strPath " << strPath);
          std::string::size_type end_postion = strPath.find(".bag");
          std::string::size_type name_postion = strPath.find(file_name);
          if (end_postion != std::string::npos && name_postion != std::string::npos)
          {
            boost::filesystem::remove(strPath);
            break;
          }
        }
      }

      isRecord = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO_STREAM_NAMED("yd_record", "Shutting down...");

  return 0;
}
