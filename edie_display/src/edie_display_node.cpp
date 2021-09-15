/*
 * edie_display_node.cpp
 *
 *  Author: Park Jaehun
 *  Refactoring: Park Jaehun , 2021.09.15
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

// subscribe display id from edie main node
void displayStateCallback(const std_msgs::Int8::ConstPtr &msg)
{
  display_id = static_cast<int>(msg->data);
  return;
}

// check if display_id was changed
bool isDisplayChanged()
{
  if(display_id != before_display_id)
  {
    // update before_display_id
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
  ros::init(argc, argv, "edie_display_node");
  ros::NodeHandle nh;
  ros::Subscriber display_state_sub = nh.subscribe("/edie/display", 10, displayStateCallback);
  ros::Publisher display_done_pub = nh.advertise<std_msgs::Int8>("/edie/display_done", 10);

  std_msgs::Int8 done_msgs;
  done_msgs.data = 1;
  ros::Rate loop_rate(24);
  
  cv::Mat frame;
  std::string img_file_name;
  bool topic_error;

  // set output frame to full screen
  cv::namedWindow("edie face", cv::WINDOW_NORMAL);
  cv::setWindowProperty("edie face", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

  // create emotions
  Emotions emotions = Emotions();

  while(ros::ok())
  {
    if(isDisplayChanged())
    {
      // change display id and check topic error
      if(topic_error = emotions.change(display_id))
      {
        std::cout << "[ERROR] emotion topic is not corrected. topic : " << display_id << std::endl;
      }
    }
    else
    {
      // update a frame of the current emotion
      emotions.update();
      // check emotion done
      if(emotions.isDone())
      {
        display_done_pub.publish(done_msgs);
      }
    }

    // read a frame of the current emotion
    frame = emotions.getImage();
    if(frame.empty())
    {
      ROS_INFO("can not open or find the image");
      break;
    }

    // show emotion frame
    cv::imshow("edie face", frame);
    int key = cv::waitKey(1);
    if(key == 'q')
      break;

    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
}
