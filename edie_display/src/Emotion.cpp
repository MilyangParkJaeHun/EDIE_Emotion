/*
 * Emotion.cpp
 *
 *  Author: jh9277
 *  Refactoring: jh9277 , 2020.06.25
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
    case EmotionState::SURPRISE      : os << "SURPRISE"; break;
    case EmotionState::BATTERYLOW    : os << "BATTERYLOW"; break;
    default                          : os << "ERROR : " << static_cast<int>(state); break;
  }
  return os; 
}

Emotion::Emotion()
{
  // Initialize variable
  size_ = 0;
  sequence_size_ = 0;
  now_idx_ = 0;
  format_ = ".png";
  emotion_ = EmotionState::BLINK;

  sequence_.clear();

  // Get current user name
  std::ostringstream user_name;
  user_name << getpwuid(getuid())->pw_name;

  // Set img path and img file name
  img_path_ = ros::package::getPath("edie_display") + "/image/Emotions/";
  img_file_name_ = img_path_ + "emotions_0" + format_;
}  

Emotion::~Emotion(){}

// Count the total number of .png files in directory
int Emotion::CountFiles(std::string directory, std::string ext)
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

// Change emotion represented by enum class to string
std::string Emotion::State2Str(EmotionState emotion)
{
  std::ostringstream stream;
  stream << emotion;
  std::string str = stream.str();

  return str;
}

// Set current Emotion according to emotionId
void Emotion::Set(int emotionId)
{
  emotion_ = static_cast<EmotionState>(emotionId);
  emotion_name_ = State2Str(emotion_);
  img_path_ += emotion_name_ + "/";
  size_ = CountFiles(img_path_, format_);

  std::string img_file_name;
  cv::Mat frame;

  for(int i = 0; i < size_; i++)
  {
    img_file_name = GetImgFileName(i);
    frame = cv::imread(img_file_name, cv::IMREAD_COLOR);
    image_.push_back(frame);
  }
  
  // Parser create
  Parser parser = Parser();
  std::cout << "emotion : " << emotion_name_ << std::endl;
  // Get emotion frame sequence
  sequence_ = parser.GetSequence(emotion_name_);
  sequence_size_ = sequence_.size();
  return;
}

// Start current Emotion at 0
void Emotion::Start()
{
  now_idx_ = 0;
  return;
}

// Add index 
void Emotion::AddIdx()
{
  now_idx_ = (now_idx_ + 1) % sequence_size_;
  return;
}

// Get index
int Emotion::GetIdx()
{
  return now_idx_;
}

// Get total image file size
int Emotion::GetSize()
{
  return size_;
}

// Get current emotion name
std::string Emotion::GetName()
{
  return emotion_name_;
}

// Get current image file name
std::string Emotion::GetImgFileName()
{
  img_file_name_ = img_path_ + "emotions_" + std::to_string(now_idx_) + format_;
  return img_file_name_;
}

// Get current image file name according to idx
std::string Emotion::GetImgFileName(int idx)
{
  img_file_name_ = img_path_ + "emotions_" + std::to_string(idx) + format_;
  return img_file_name_;
}

// Get current image
cv::Mat Emotion::GetImage()
{
  return image_[sequence_[now_idx_]];
}

// Check play done
bool Emotion::CheckPlayDone()
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
