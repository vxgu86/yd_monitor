
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
  ROS_INFO_STREAM("msg>>>>: " << msg->data);
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

void *update(void *tmp)
{
  ros::NodeHandle *p = (ros::NodeHandle *)tmp;
  ros::Subscriber sub = p->subscribe<std_msgs::Int16>("/yd_record/cmd", 1, cmdCallback);
  ros::spin();
}

int main(int argc, char **argv)
{
  ROS_INFO_STREAM_NAMED("yd_record", "yd_record");
  ros::init(argc, argv, "yd_record");
  ros::NodeHandle nh;

  pthread_t tid;
  pthread_create(&tid, NULL, update, (void *)&nh);
  //ros::Subscriber sub = nh.subscribe<std_msgs::Int16>("/yd_record/cmd", 1, cmdCallback);

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
  std::vector<std::string> delete_files;
  while (ros::ok())
  {
    t = ros::Time::now();
    ROS_INFO_STREAM("yd_record Recording ... ");
    //stop
    if (isStop && isRecord)
    {
      ROS_INFO_STREAM("yd_record isStop 1");
      recorder.stopRecording();
      isRecord = false;
    }

    // Start
    if (!isRecord && !isStop)
    {
      ROS_INFO_STREAM("yd_record Start 2");
      recorder.startRecording(options);
      start_time = ros::Time::now();
      isRecord = true;
    }

    //ros::Duration(1.0).sleep();
    if (t - start_time > duration && isRecord)
    {
      ROS_INFO_STREAM("yd_record Stop 3");
      // Stop
      recorder.stopRecording();

      //delete
      boost::filesystem::recursive_directory_iterator beg_iter(path_save);
      boost::filesystem::recursive_directory_iterator end_iter;
      delete_files.clear();
      for (; beg_iter != end_iter; ++beg_iter)
      {
        if (boost::filesystem::is_directory(*beg_iter))
        {
          continue;
        }
        else
        {
          std::string strPath = beg_iter->path().string(); //遍历出来的文件名称
          if (boost::filesystem::exists(strPath))
          {
            ROS_INFO_STREAM("exists strPath " << strPath);
            std::string::size_type end_postion = strPath.find(".bag");
            std::string::size_type name_postion = strPath.find(file_name);
            if (end_postion != std::string::npos && name_postion != std::string::npos)
            {

              delete_files.push_back(strPath);
            }
          }
        }
      }
      //delete
      int count = delete_files.size();
      for (int i = 0; i < count; i++)
      {
        ROS_INFO_STREAM("delete strPath " << delete_files[i]);
        boost::filesystem::remove(delete_files[i]);
      }
      isRecord = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO_STREAM_NAMED("yd_record", "Shutting down...");

  return 0;
}
