/*
 * Emotions.cpp
 *
 *  Author: jh9277
 *  Refactoring: jh9277 , 2020.06.25
 */

#include "edie_display/Emotions.hpp"


Emotions::Emotions()
{
  // Initialize id
  now_id_ = 0;
  // Get total emotion count
  total_emotion_count_ = static_cast<int>(EmotionState::COUNT);
  // Set emotion array
  for(int i=0; i<total_emotion_count_; i++)
  {
    Emotion now_emotion = Emotion();
    now_emotion.Set(i);
    contents_.push_back(now_emotion);
  }
}

Emotions::~Emotions(){}

// Change now emotion id
bool Emotions::Change(int id)
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

  Emotion& now_emotion = contents_[now_id_];
  now_emotion.Start();

  return topic_error;
}

// Emotion play
void Emotions::Play()
{
  Emotion& now_emotion = contents_[now_id_];
  now_emotion.AddIdx();

  return;
}

// Check emotion play done
bool Emotions::Done()
{
  Emotion& now_emotion = contents_[now_id_];

  return now_emotion.CheckPlayDone();
}

// Get current playing file name
std::string Emotions::GetFileName()
{
  Emotion& now_emotion = contents_[now_id_];
  std::string file_name = now_emotion.GetImgFileName();

  return file_name;
}

// Get current image
cv::Mat Emotions::GetImage()
{
  return contents_[now_id_].GetImage();
}

// Get current emotion name
std::string Emotions::TestGet(int id)
{
  Emotion emotion = contents_[id];
  int size = emotion.GetSize();
  std::string name = emotion.GetName();
  return name;
}
