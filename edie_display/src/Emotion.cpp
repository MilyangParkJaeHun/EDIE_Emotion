/*
 * Emotion.cpp
 *
 *  Author: Park Jaehun
 *  Refactoring: Park Jaehun , 2021.09.15
 */

#include "edie_display/Emotion.hpp"
#include <iostream>

std::ostream& operator<<(std::ostream& os, EmotionState state)
{
  switch(state)
  {
    case EmotionState::BLINK         : os << "BLINK"; break;
    case EmotionState::FLINCH        : os << "FLINCH"; break;
    case EmotionState::LAUGH         : os << "LAUGH"; break;
    case EmotionState::LOOK_AROUND   : os << "LOOK_AROUND"; break;
    case EmotionState::LOVE          : os << "LOVE"; break;
    case EmotionState::BATTERYLOW    : os << "BATTERYLOW"; break;
    default                          : os << "ERROR : " << static_cast<int>(state); break;
  }
  return os;
}

Emotion::Emotion(int emotionId)
{
  // Initialize variable
  size_ = 0;
  sequence_size_ = 0;
  now_idx_ = 0;
  format_ = ".png";
  emotion_ = EmotionState::BLINK;

  sequence_.clear();

  // Set img path and img file name
  img_path_ = ros::package::getPath("edie_display") + "/image/Emotions/";
  img_file_name_ = img_path_ + "emotions_0" + format_;

  set(emotionId);
}

Emotion::~Emotion(){}

void Emotion::set(int emotionId)
{
  emotion_ = static_cast<EmotionState>(emotionId);
  emotion_name_ = state2Str(emotion_);
  img_path_ += emotion_name_ + "/";
  size_ = countFiles(img_path_, format_);

  std::string img_file_name;
  cv::Mat frame;

  for(int i = 0; i < size_; i++)
  {
    img_file_name = getImgFileName(i);
    frame = cv::imread(img_file_name, cv::IMREAD_COLOR);
    image_.push_back(frame);
  }
  
  // Parser create
  Parser* parser = new Parser;
  std::cout << "emotion : " << emotion_name_ << std::endl;
  // Get emotion frame sequence
  sequence_ = parser->getSequence(emotion_name_);
  sequence_size_ = sequence_.size();
  
  delete parser;
}

int Emotion::countFiles(std::string directory, std::string ext)
{
  namespace fs = boost::filesystem;
  fs::path Path(directory);
  int Nb_ext = 0;
  fs::directory_iterator end_iter;

  for (fs::directory_iterator iter(Path); iter != end_iter; ++iter)
    if (iter->path().extension() == ext)
      ++Nb_ext;

  return Nb_ext;
}

std::string Emotion::state2Str(EmotionState emotion)
{
  std::ostringstream stream;
  stream << emotion;
  std::string str = stream.str();

  return str;
}

void Emotion::start()
{
  now_idx_ = 0;
}

void Emotion::addIdx()
{
  now_idx_ = (now_idx_ + 1) % sequence_size_;
}

int Emotion::getIdx()
{
  return now_idx_;
}

int Emotion::getSize()
{
  return size_;
}

std::string Emotion::getName()
{
  return emotion_name_;
}

std::string Emotion::getImgFileName()
{
  img_file_name_ = img_path_ + "emotions_" + std::to_string(now_idx_) + format_;
  return img_file_name_;
}

std::string Emotion::getImgFileName(int idx)
{
  img_file_name_ = img_path_ + "emotions_" + std::to_string(idx) + format_;
  return img_file_name_;
}

cv::Mat Emotion::getImage()
{
  return image_[sequence_[now_idx_]];
}

bool Emotion::isPlayDone()
{
  if(now_idx_ == 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}
