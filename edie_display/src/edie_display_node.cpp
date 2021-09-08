/*
 * edie_display_node.cpp
 *
 *  Author: jh9277
 *  Refactoring: jh9277 , 2020.06.25
 */

#include <cstdio>
#include <stdlib.h>
#include <vector>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int8.h>
#include "edie_display/Emotions.hpp"

int display_id;
int before_display_id;
std::string img_file_name = "";

// Subscribe display id from edie main node
void DisplayStateCallback(const std_msgs::Int8::ConstPtr &msg)
{
  display_id = static_cast<int>(msg->data);
  return;
}

// Check display id has changed
bool DisplayChange()
{
  if(display_id != before_display_id)
  {
    // Update before_display_id
    before_display_id = display_id;
    return true;
  }
  else
  {
    return false;
  }
}

int main(int argc, char** argv)
{
  sleep(5);
  ros::init(argc, argv, "edie_display_node");
  ros::NodeHandle nh;
  ros::Subscriber display_state_sub = nh.subscribe("/edie/display", 10, DisplayStateCallback);
  ros::Publisher display_done_pub = nh.advertise<std_msgs::Int8>("/edie/display_done", 10);

  std_msgs::Int8 done_msgs;
  done_msgs.data = 1;
  ros::Rate loop_rate(24);
  
  cv::Mat frame;
  std::string img_file_name;
  bool topic_error;

  cv::namedWindow("edie face", cv::WINDOW_NORMAL);
  cv::setWindowProperty("edie face", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

  // Create emotions
  Emotions emotions = Emotions();

  while(ros::ok())
  {
    // Check if display id is changed
    if(DisplayChange())
    {
      // Change display id and check topic error
      if(topic_error = emotions.Change(display_id))
      {
        std::cout << "[ERROR] emotion topic is not corrected. topic : " << display_id << std::endl;
      }
    }
    else
    {
      // Play emotions
      emotions.Play();
      // Check emotion done
      if(emotions.Done())
      {
        display_done_pub.publish(done_msgs);
      }
    }

    img_file_name = emotions.GetFileName();

    if(!topic_error)
    {
  //    std::cout << "file : " << img_file_name << std::endl;
    }

    // Read emotion png file
    frame = emotions.GetImage();
    if(frame.empty())
    {
      ROS_INFO("can not open or find the image");
      return -1;
    }
    // Show emotion frame
    cv::imshow("edie face", frame);
    int key = cv::waitKey(1);
    if(key == 'q')
      break;

    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
