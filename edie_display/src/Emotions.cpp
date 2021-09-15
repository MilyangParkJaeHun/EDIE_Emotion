/*
 * Emotions.cpp
 *
 *  Author: Park Jaehun
 *  Refactoring: Park Jaehun , 2021.09.15
 */

#include "edie_display/Emotions.hpp"

Emotions::Emotions()
{
  // Initialize id
  now_id_ = 0;
  // Get total number of emotions
  total_emotion_count_ = static_cast<int>(EmotionState::COUNT);
  
  // Create instances for all emotions.
  for(int i=0; i<total_emotion_count_; i++)
  {
    emotion_list_.emplace_back(i);
  }
}

Emotions::~Emotions(){}

bool Emotions::change(int id)
{
  // Check if topic is in range
  bool topic_error = false;
  if(id < 0 || id >= total_emotion_count_)
  {
    now_id_ = 0;
    topic_error = true;
  }
  else
  {
    now_id_ = id;
  }

  Emotion& now_emotion = emotion_list_[now_id_];
  now_emotion.start();

  return topic_error;
}

void Emotions::update()
{
  Emotion& now_emotion = emotion_list_[now_id_];
  now_emotion.addIdx();

  return;
}

bool Emotions::isDone()
{
  Emotion& now_emotion = emotion_list_[now_id_];

  return now_emotion.isPlayDone();
}

std::string Emotions::getFileName()
{
  Emotion& now_emotion = emotion_list_[now_id_];
  std::string file_name = now_emotion.getImgFileName();

  return file_name;
}

cv::Mat Emotions::getImage()
{
  return emotion_list_[now_id_].getImage();
}

std::string Emotions::testGet(int id)
{
  Emotion emotion = emotion_list_[id];
  int size = emotion.getSize();
  std::string name = emotion.getName();
  return name;
}
