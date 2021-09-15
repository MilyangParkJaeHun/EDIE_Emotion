/*
 * edie_emotion_node.cpp
 *
 *  Author: Park Jaehun
 *  Refactoring: Park Jaehun , 2020.06.29
 */

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <iostream>

int display_done;
bool battery_low;
std_msgs::Int8 emotion_state, before_emotion_state;
std_msgs::Int8 battery_low_msgs;
ros::Publisher display_pub;
ros::Publisher sound_pub;

/**
 * @brief Publish emotions state by ros topic.
 * if the battery is low, not published. 
 *
 * @param emotion_state is emotion to express
 */
void publish_process(std_msgs::Int8 emotion_state)
{
  if(!battery_low) {
    display_pub.publish(emotion_state);

    if(emotion_state.data > 0)
    {
      sound_pub.publish(emotion_state);
    }
  }
}

// Callback of display done state
void DisplayDoneCallback(const std_msgs::Int8::ConstPtr &msg)
{
  display_done = msg->data;
}

/**
 * @brief Callback of emotion state
 * 
 * @param msg has emotion state to express
 */
void EmotionCallback(const std_msgs::Int8::ConstPtr &msg)
{
  emotion_state = *msg;
  publish_process(emotion_state);
}

/**
 * @brief Callback of battery state
 * 
 * @param msg has a value of true or false
 * true means battery is low
 */
void BatteryLowCallback(const std_msgs::Bool::ConstPtr &msg)
{
  battery_low = msg->data;
}

// Check if display output is finished.
bool isDone()
{
  if(display_done)
    return true;
  else
    return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "edie_emotion_node");
  ros::NodeHandle nh;
  ros::Subscriber display_done_sub = nh.subscribe("/edie/display_done", 10, DisplayDoneCallback);
  ros::Subscriber display = nh.subscribe("/edie/emotion", 10, EmotionCallback);
  ros::Subscriber battery_low_sub = nh.subscribe("/edie/battery_low", 10, BatteryLowCallback);
  ros::Publisher emotion_done_pub = nh.advertise<std_msgs::Int8>("/edie/emotion_done", 10);
  display_pub = nh.advertise<std_msgs::Int8>("/edie/display", 10);
  sound_pub = nh.advertise<std_msgs::Int8>("/edie/sound", 10);

  std_msgs::Int8 done_msgs;
  done_msgs.data = 1;

  ros::Rate loop_rate(100);

  while(ros::ok())
  {
    if(isDone())
    {
      display_done = 0;
      emotion_done_pub.publish(done_msgs);
    }
    if(battery_low) {
      battery_low_msgs.data = 5;
      display_pub.publish(battery_low_msgs);
    }
    loop_rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
