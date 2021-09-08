/*
 * Emotion.hpp
 *
 *  Author: jh9277
 *  Refactoring: jh9277 , 2020.06.25
 */

#ifndef EMOTION_HPP
#define EMOTION_HPP
#include <boost/filesystem.hpp>
#include <cstring>
#include <cstdio>
#include <sstream>
#include <unistd.h>
#include <pwd.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <edie_struct.h>
#include "edie_display/Parser.hpp"
#include <ros/package.h>

class Emotion
{
  private:
    // Total image file count
    int size_;
    // Total emotion sequence count
    int sequence_size_;
    // Now image index
    int now_idx_;
    // Image file format
    std::string format_;
    // Image file directory path
    std::string img_path_;
    // Image file path
    std::string img_file_name_;
    // EDIE emotion string
    std::string emotion_name_;
    // EDIE emotion enum class
    EmotionState emotion_;
    // Emotion sequence
    std::vector<int> sequence_;
    // Emotion image frame
    std::vector<cv::Mat> image_;

  public:
    // Constructor
    Emotion();
    // Destuctor
    ~Emotion();

    int CountFiles(std::string directory, std::string ext);
    std::string State2Str(EmotionState emotion);
    void Set(int emotionId);
    void Start();
    void AddIdx();
    int GetIdx();
    int GetSize();
    std::string GetName();
    std::string GetImgFileName();
    std::string GetImgFileName(int idx);
    cv::Mat GetImage();
    bool CheckPlayDone();
};

#endif
